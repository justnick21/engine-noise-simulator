/*
 * sim/motion.js — synthetic accelerometer generator.
 *
 * World frame: car forward = +X, left = +Y, up = +Z, gravity points -Z.
 * A phone mounted at orientation (yaw,pitch,roll) reports specific force
 * (linear accel minus gravity) rotated into its own frame — exactly what
 * DeviceMotion's accelerationIncludingGravity gives.
 *
 * A "drive profile" is a function t -> {ax, ay, az} of the car's linear
 * acceleration in the WORLD frame:
 *   ax = longitudinal (throttle/brake), ay = lateral (cornering), az = vertical (bumps).
 * That ax is the ground truth the mapper is judged against.
 */
"use strict";
const G = 9.80665;

// seeded PRNG so runs are reproducible
function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(rng) {
  // Box-Muller
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// Euler (yaw about Z, pitch about Y, roll about X) -> world->device matrix rows.
function eulerMatrix(yaw, pitch, roll) {
  const cz = Math.cos(yaw), sz = Math.sin(yaw);
  const cy = Math.cos(pitch), sy = Math.sin(pitch);
  const cx = Math.cos(roll), sx = Math.sin(roll);
  // R = Rx * Ry * Rz  (world vector -> device vector)
  const Rz = [[cz, sz, 0], [-sz, cz, 0], [0, 0, 1]];
  const Ry = [[cy, 0, -sy], [0, 1, 0], [sy, 0, cy]];
  const Rx = [[1, 0, 0], [0, cx, sx], [0, -sx, cx]];
  return mul(Rx, mul(Ry, Rz));
}
function mul(A, B) {
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
  return C;
}
function apply(R, v) {
  return {
    x: R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
    y: R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
    z: R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z,
  };
}
const deg = (d) => (d * Math.PI) / 180;

/*
 * Generate a stream of DeviceMotion-like samples.
 * @param opts {duration, hz, mount:{yaw,pitch,roll} (deg), profile(t)->{ax,ay,az},
 *              noise (m/s^2), provideLinear, seed}
 * @returns array of {t, dt, sample:{accelerationIncludingGravity, acceleration?}, truth:{ax,ay,az}}
 */
function generate(opts) {
  const hz = opts.hz || 60;
  const dt = 1 / hz;
  const n = Math.round((opts.duration || 4) * hz);
  const m = opts.mount || { yaw: 0, pitch: 0, roll: 0 };
  const R = eulerMatrix(deg(m.yaw), deg(m.pitch), deg(m.roll));
  const noise = opts.noise ?? 0.06;
  const rng = mulberry32(opts.seed ?? 1);
  const out = [];
  for (let i = 0; i < n; i++) {
    const t = i * dt;
    const a = opts.profile(t) || { ax: 0, ay: 0, az: 0 };
    // specific force in world = linear accel - gravity(-Z) = (ax, ay, az + G)
    const specWorld = { x: a.ax || 0, y: a.ay || 0, z: (a.az || 0) + G };
    const ig = apply(R, specWorld);
    ig.x += gauss(rng) * noise;
    ig.y += gauss(rng) * noise;
    ig.z += gauss(rng) * noise;
    const sample = { accelerationIncludingGravity: ig };
    if (opts.provideLinear) {
      const linWorld = { x: a.ax || 0, y: a.ay || 0, z: a.az || 0 };
      const lin = apply(R, linWorld);
      lin.x += gauss(rng) * noise;
      lin.y += gauss(rng) * noise;
      lin.z += gauss(rng) * noise;
      sample.acceleration = lin;
    }
    out.push({ t, dt, sample, truth: { ax: a.ax || 0, ay: a.ay || 0, az: a.az || 0 } });
  }
  return out;
}

// ---- drive-profile builders ----
const smoothstep = (e0, e1, x) => {
  const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0)));
  return t * t * (3 - 2 * t);
};
// a trapezoidal pulse on ax between [t0,t1] ramping over `ramp`
function pulse(t0, t1, amp, ramp = 0.4) {
  return (t) => amp * (smoothstep(t0, t0 + ramp, t) - smoothstep(t1 - ramp, t1, t));
}
function combine(...fns) {
  return (t) => {
    const r = { ax: 0, ay: 0, az: 0 };
    for (const f of fns) {
      const v = f(t);
      r.ax += v.ax || 0; r.ay += v.ay || 0; r.az += v.az || 0;
    }
    return r;
  };
}
const asAx = (fn) => (t) => ({ ax: fn(t), ay: 0, az: 0 });
const asAy = (fn) => (t) => ({ ax: 0, ay: fn(t), az: 0 });
// road bumps: band-ish vertical noise
function bumps(amp, hz) {
  return (t) => ({ ax: 0, ay: 0, az: amp * Math.sin(2 * Math.PI * hz * t) * (0.6 + 0.4 * Math.sin(t * 3.1)) });
}

/*
 * addGPS — attach synthetic GPS Doppler-speed fixes to a generated stream.
 * True forward speed is the integral of the profile's longitudinal accel (ax);
 * GPS reports it at ~1 Hz, delayed by `latency` and blurred by `noise`, exactly
 * like a phone. Each stream sample gains `gps` = { speed } on a fix, else null.
 * `dropout: [t0, t1]` simulates a tunnel / lost signal window.
 * Also tags every sample with `truth.speed` (ground-truth m/s) for assertions.
 */
function addGPS(stream, opts = {}) {
  const hz = opts.hz || 1;
  const latency = opts.latency ?? 0.6;
  const noise = opts.noise ?? 0.35;
  const dropout = opts.dropout || null;
  const rng = mulberry32(opts.seed ?? 7);

  // integrate true forward speed (m/s), clamped at 0 (can't reverse)
  const speed = new Array(stream.length);
  let s = 0;
  for (let i = 0; i < stream.length; i++) {
    s = Math.max(0, s + (stream[i].truth.ax || 0) * stream[i].dt);
    speed[i] = s;
    stream[i].truth.speed = s;
    stream[i].gps = null;
  }
  const speedAt = (t) => {
    if (t <= 0) return speed[0];
    const idx = Math.min(speed.length - 1, Math.round(t / stream[0].dt));
    return speed[idx];
  };

  const period = 1 / hz;
  let nextFix = 0;
  for (let i = 0; i < stream.length; i++) {
    const t = stream[i].t;
    if (t >= nextFix) {
      nextFix += period;
      if (dropout && t >= dropout[0] && t < dropout[1]) continue; // no signal
      const measured = Math.max(0, speedAt(t - latency) + gauss(rng) * noise);
      stream[i].gps = { speed: measured };
    }
  }
  return stream;
}

module.exports = {
  G, generate, addGPS, eulerMatrix, apply, deg,
  pulse, combine, asAx, asAy, bumps, smoothstep,
};
