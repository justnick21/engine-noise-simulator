/*
 * engine-core.js — shared, pure logic for the EV Engine Simulator.
 *
 * Runs in the browser (attaches to window.EngineCore) and in Node (module.exports).
 * NO Web Audio, NO DOM — just math, so the same code the app runs can be driven
 * headlessly by the test harness.
 *
 * Three pieces:
 *   MotionMapper — raw accelerometer  -> clean {throttle, brake, longitudinal, ...}
 *   DriveModel   -> throttle/brake    -> {speed, gear, rpm}
 *   voicing()    -> rpm/throttle/mode -> synth parameters (VoiceState)
 *
 * The genuinely hard part is MotionMapper: a phone is mounted at an unknown
 * orientation, and the car brakes, corners and hits bumps. We recover a believable
 * longitudinal-acceleration signal without a gyro or GPS.
 */
(function (root) {
  "use strict";

  // ---------- tiny vec3 helpers ----------
  const V = {
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    scale: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    cross: (a, b) => ({
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x,
    }),
    len: (a) => Math.hypot(a.x, a.y, a.z),
    norm: (a) => {
      const l = Math.hypot(a.x, a.y, a.z) || 1;
      return { x: a.x / l, y: a.y / l, z: a.z / l };
    },
  };
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
  const G = 9.80665;

  /*
   * MotionMapper
   * -----------------------------------------------------------------------
   * Feed it raw device motion samples; get back a throttle/brake signal that
   * is robust to mount orientation, cornering and road noise.
   *
   * Pipeline per sample:
   *   1. gravity  = slow low-pass of accelerationIncludingGravity  (finds "down")
   *   2. linear   = accel - gravity                                (motion only)
   *   3. horiz    = linear projected onto the ground plane         (drops bumps' DC,
   *                 keeps their AC — handled later)
   *   4. forward  = principal axis of horizontal accel via streaming 2x2 PCA
   *                 (accelerating/braking varies more along travel than sideways)
   *   5. long/lat = signed components along forward / sideways
   *   6. sign     = locked on the first sustained launch so +long == accelerate
   *   7. smooth   = asymmetric low-pass (fast attack, slower release) + deadzone
   */
  class MotionMapper {
    constructor(opts = {}) {
      this.o = Object.assign(
        {
          // The mount is fixed, so gravity in the device frame is ~constant.
          // Settle it fast at startup (car assumed near rest), then HOLD it with
          // a long time constant so multi-second accelerations aren't mistaken
          // for gravity and subtracted away.
          gravityWarmTau: 0.4, // s, fast settle during warmup
          gravityTau: 12.0, // s, slow hold afterwards
          warmup: 1.2, // s of fast settling on startup
          pcaTau: 2.0, // s, how fast the forward axis adapts
          pcaMinMag: 0.7, // m/s^2, ignore tiny accel for axis estimation
          attackTau: 0.12, // s, throttle rise smoothing
          releaseTau: 0.35, // s, throttle fall smoothing
          deadzone: 0.5, // m/s^2 of longitudinal accel before throttle moves
          fullScale: 3.2, // m/s^2 longitudinal that maps to throttle = 1
          brakeScale: 4.5, // m/s^2 of deceleration that maps to brake = 1
          latReject: 0.75, // how strongly lateral accel is discounted (0..1)
          signLockMag: 1.2, // m/s^2 sustained to lock the forward sign
          signLockTime: 0.35, // s the launch must persist to lock sign
        },
        opts
      );
      this.reset();
    }

    reset() {
      this.gravity = { x: 0, y: 0, z: G }; // assume screen-up until we learn
      this.gravityInit = false;
      this._age = 0;
      // streaming horizontal covariance (in a gravity-aligned 2D basis)
      this.cov = { uu: 0, uv: 0, vv: 0 };
      this.forward2d = { u: 1, v: 0 }; // forward axis in the (e1,e2) basis
      this.throttle = 0;
      this.brake = 0;
      this.longRaw = 0;
      this.lat = 0;
      this.vert = 0;
      this.signLocked = false;
      this.confident = false;
      this._signTimer = 0;
      this._e1 = { x: 1, y: 0, z: 0 };
      this._e2 = { x: 0, y: 1, z: 0 };
      this.forward3d = { x: 1, y: 0, z: 0 };
    }

    // Build an orthonormal horizontal basis (e1,e2) perpendicular to `down`.
    _horizontalBasis(down) {
      // pick the world axis least aligned with down to seed e1
      const ax = Math.abs(down.x),
        ay = Math.abs(down.y),
        az = Math.abs(down.z);
      let seed;
      if (ax <= ay && ax <= az) seed = { x: 1, y: 0, z: 0 };
      else if (ay <= az) seed = { x: 0, y: 1, z: 0 };
      else seed = { x: 0, y: 0, z: 1 };
      let e1 = V.sub(seed, V.scale(down, V.dot(seed, down)));
      e1 = V.norm(e1);
      const e2 = V.norm(V.cross(down, e1));
      return { e1, e2 };
    }

    /**
     * @param sample {accelerationIncludingGravity:{x,y,z}, acceleration?:{x,y,z}}
     * @param dt seconds since previous sample
     * @returns {throttle, brake, longitudinal, lateral, vertical, forward, gravity}
     */
    update(sample, dt) {
      dt = clamp(dt || 0.016, 0.001, 0.1);
      const ig = sample.accelerationIncludingGravity || { x: 0, y: 0, z: 0 };
      const acc = { x: ig.x || 0, y: ig.y || 0, z: ig.z || 0 };

      // 1. gravity: settle fast at startup, then hold with a long time constant
      this._age += dt;
      if (!this.gravityInit) {
        this.gravity = acc;
        this.gravityInit = true;
      } else {
        const tau = this._age < this.o.warmup ? this.o.gravityWarmTau : this.o.gravityTau;
        const ag = Math.exp(-dt / tau);
        this.gravity = V.add(V.scale(this.gravity, ag), V.scale(acc, 1 - ag));
      }
      const down = V.norm(this.gravity);

      // 2. linear acceleration (gravity removed). Prefer device-provided linear
      //    accel when available; otherwise subtract our gravity estimate.
      let lin;
      if (sample.acceleration && sample.acceleration.x != null) {
        lin = {
          x: sample.acceleration.x || 0,
          y: sample.acceleration.y || 0,
          z: sample.acceleration.z || 0,
        };
      } else {
        lin = V.sub(acc, this.gravity);
      }

      // 3. split into vertical (bumps) and horizontal (driving) parts
      const vAmt = V.dot(lin, down);
      this.vert = vAmt;
      const horiz = V.sub(lin, V.scale(down, vAmt));

      // 4. streaming PCA of horizontal accel -> forward axis
      const { e1, e2 } = this._horizontalBasis(down);
      this._e1 = e1;
      this._e2 = e2;
      const u = V.dot(horiz, e1);
      const v = V.dot(horiz, e2);
      const mag = Math.hypot(u, v);
      // The mount is fixed, so forward is a fixed device-frame axis. We estimate
      // it from early accel/brake, then FREEZE once confident — otherwise a hard
      // slalom (lateral variance > longitudinal) would re-point it onto the
      // cornering axis and corners would read as throttle.
      if (mag > this.o.pcaMinMag && !this.confident) {
        const w = Math.exp(-dt / this.o.pcaTau);
        const g = (1 - w) * Math.min(mag, 6);
        this.cov.uu = this.cov.uu * w + g * u * u;
        this.cov.uv = this.cov.uv * w + g * u * v;
        this.cov.vv = this.cov.vv * w + g * v * v;
        this.forward2d = principalAxis2x2(this.cov, this.forward2d);

        // confidence: sign locked AND covariance is strongly one-directional
        const tr = this.cov.uu + this.cov.vv;
        const det = this.cov.uu * this.cov.vv - this.cov.uv * this.cov.uv;
        const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
        const aniso = (2 * disc) / (tr + 1e-9); // (l1-l2)/(l1+l2), ->1 when 1D
        if (this.signLocked && aniso > 0.6 && tr > 2) this.confident = true;
      }

      // longitudinal / lateral in the horizontal plane
      const f = this.forward2d;
      let longitudinal = u * f.u + v * f.v; // signed, sign still ambiguous
      const lateral = u * -f.v + v * f.u; // perpendicular component
      this.lat = lateral;

      // 5. lock the forward SIGN on the first sustained launch so +long = go
      if (!this.signLocked) {
        if (Math.abs(longitudinal) > this.o.signLockMag) {
          this._signTimer += dt;
          if (this._signTimer >= this.o.signLockTime) {
            // whatever direction that sustained push points is "accelerate"
            if (longitudinal < 0) {
              this.forward2d = { u: -f.u, v: -f.v };
              longitudinal = -longitudinal;
            }
            this.signLocked = true;
          }
        } else {
          this._signTimer = 0;
        }
      }
      this.longRaw = longitudinal;

      // 6. discount cornering: subtract a fraction of |lateral| as "load" that
      //    isn't forward thrust, so hard corners don't read as throttle.
      const latPenalty = this.o.latReject * Math.abs(lateral);
      const effFwd = longitudinal - Math.sign(longitudinal) * Math.min(Math.abs(longitudinal), latPenalty);

      // 7. map to throttle (accelerate) and brake (decelerate) with deadzone
      const dz = this.o.deadzone;
      const throttleTarget = clamp((effFwd - dz) / (this.o.fullScale - dz), 0, 1);
      const brakeTarget = clamp((-effFwd - dz) / (this.o.brakeScale - dz), 0, 1);

      // asymmetric smoothing: quick to rev, slower to fall (feels like an engine)
      const aA = Math.exp(-dt / this.o.attackTau);
      const aR = Math.exp(-dt / this.o.releaseTau);
      this.throttle =
        throttleTarget > this.throttle
          ? this.throttle * aA + throttleTarget * (1 - aA)
          : this.throttle * aR + throttleTarget * (1 - aR);
      this.brake = this.brake * aA + brakeTarget * (1 - aA);

      this.forward3d = V.add(V.scale(e1, this.forward2d.u), V.scale(e2, this.forward2d.v));

      return {
        throttle: this.throttle,
        brake: this.brake,
        longitudinal: effFwd,
        lateral,
        vertical: vAmt,
        forward: this.forward3d,
        gravity: this.gravity,
        signLocked: this.signLocked,
      };
    }
  }

  // Principal eigenvector of a symmetric 2x2 covariance [[uu,uv],[uv,vv]].
  // `prev` keeps sign/continuity across frames.
  function principalAxis2x2(c, prev) {
    const { uu, uv, vv } = c;
    const tr = uu + vv;
    const det = uu * vv - uv * uv;
    const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
    const l1 = tr / 2 + disc; // larger eigenvalue
    let ex, ey;
    if (Math.abs(uv) > 1e-9) {
      ex = l1 - vv;
      ey = uv;
    } else {
      // diagonal: axis is whichever variance is larger
      ex = uu >= vv ? 1 : 0;
      ey = uu >= vv ? 0 : 1;
    }
    const len = Math.hypot(ex, ey) || 1;
    ex /= len;
    ey /= len;
    // keep continuity with previous frame (avoid 180° flips)
    if (prev && ex * prev.u + ey * prev.v < 0) {
      ex = -ex;
      ey = -ey;
    }
    return { u: ex, v: ey };
  }

  /*
   * DriveModel — throttle/brake -> speed, gear, rpm.
   * Pseudo-speed integrator (no true speed needed); geared modes sweep rpm and
   * reset on shifts, linear modes (turbine/warp) spool continuously.
   */
  class DriveModel {
    constructor() {
      this.reset();
    }
    reset() {
      this.speed = 0; // normalized 0..1
      this.rpm = 0;
      this.gear = 0;
    }
    step(mode, throttle, brake, dt) {
      dt = clamp(dt, 0.001, 0.1);
      const power = mode.power ?? 0.55;
      const drag = mode.drag ?? 0.45;
      const coast = mode.coast ?? 0.02;
      const braking = (mode.braking ?? 1.2) * brake;
      this.speed += (throttle * power - drag * this.speed - coast - braking * this.speed) * dt;
      this.speed = clamp(this.speed, 0, 1);

      let target;
      if (mode.transmission === "linear") {
        // continuous spool: rpm tracks speed + throttle directly
        const revBias = Math.max(throttle, this.speed);
        target = mode.idle + revBias * (mode.redline - mode.idle);
        this.gear = 0;
      } else {
        const gears = mode.gears ?? 6;
        const span = 1 / gears;
        this.gear = Math.min(gears - 1, Math.floor(this.speed / span));
        const within = (this.speed - this.gear * span) / span;
        const gearTop = mode.redline * 0.9;
        target = mode.idle + within * (gearTop - mode.idle);
        target += throttle * (mode.redline - target) * 0.35; // blip
      }
      target = clamp(target, mode.idle, mode.redline);

      const k = clamp(dt * (mode.responsiveness ?? 7), 0, 1);
      this.rpm += (target - this.rpm) * k;
      this.rpm *= 1 + (pseudoJitter() - 0.5) * 0.006;
      return { speed: this.speed, gear: this.gear, rpm: this.rpm };
    }
  }

  // deterministic-ish tiny jitter without Math.random dependence in tests
  let _jseed = 12345;
  function pseudoJitter() {
    _jseed = (_jseed * 1103515245 + 12345) & 0x7fffffff;
    return _jseed / 0x7fffffff;
  }

  /*
   * Engine / vehicle modes. Combustion modes fire a pulse train at
   * firing = rpm/60 * cylMult. Exotic modes synthesize differently but reuse
   * the rpm/speed state so they still respond to real acceleration.
   */
  const MODES = [
    {
      id: "inline4", name: "Inline-4", kind: "combustion", cylMult: 2,
      idle: 820, redline: 7200, brightness: 1.0, rumble: 0.35, hiss: 0.55,
      harmonics: 18, oddBias: 1.15, transmission: "geared", gears: 6,
    },
    {
      id: "v8", name: "V8", kind: "combustion", cylMult: 4,
      idle: 680, redline: 6600, brightness: 0.85, rumble: 0.7, hiss: 0.45,
      harmonics: 22, oddBias: 0.9, transmission: "geared", gears: 5,
    },
    {
      id: "vtwin", name: "V-Twin", kind: "combustion", cylMult: 1,
      idle: 980, redline: 8200, brightness: 1.15, rumble: 0.55, hiss: 0.65,
      harmonics: 14, oddBias: 1.4, transmission: "geared", gears: 6,
    },
    {
      id: "kart", name: "2-Stroke Kart", kind: "combustion", cylMult: 1,
      idle: 1600, redline: 13000, brightness: 1.4, rumble: 0.15, hiss: 0.8,
      harmonics: 12, oddBias: 1.8, transmission: "geared", gears: 4,
      responsiveness: 11,
    },
    {
      id: "turbine", name: "Jet Turbine", kind: "turbine",
      idle: 1200, redline: 9000, brightness: 1.0, rumble: 0.2, hiss: 1.0,
      transmission: "linear", responsiveness: 4, drag: 0.3,
    },
    {
      id: "spaceship", name: "Spaceship", kind: "spaceship",
      idle: 400, redline: 3200, brightness: 0.9, rumble: 0.6, hiss: 0.35,
      transmission: "linear", responsiveness: 5, drag: 0.35,
    },
    {
      id: "warp", name: "Warp Drive", kind: "warp",
      idle: 300, redline: 2600, brightness: 1.1, rumble: 0.8, hiss: 0.5,
      transmission: "geared", gears: 5, responsiveness: 6,
    },
  ];
  const modeById = (id) => MODES.find((m) => m.id === id) || MODES[0];

  /*
   * voicing(mode, rpm, throttle, speed) -> VoiceState
   * A fixed-shape parameter bundle the audio layer (browser or headless) renders.
   * osc[]: tonal oscillators; sub: low rumble; noise: grit; whine: high tone.
   */
  function voicing(mode, rpm, throttle, speed) {
    const rn = clamp((rpm - mode.idle) / (mode.redline - mode.idle), 0, 1);
    const load = clamp(throttle * 0.7 + rn * 0.5, 0, 1);

    if (mode.kind === "combustion") {
      const firing = (rpm / 60) * mode.cylMult;
      return {
        osc: [
          { wave: "engine", freq: firing, gain: 0.5 },
          { wave: "engine", freq: firing * 2, gain: 0.25 + 0.35 * rn, detune: 8 },
        ],
        sub: { wave: "triangle", freq: firing * 0.5, gain: mode.rumble * (0.5 + 0.5 * load) * 0.5 },
        noise: { gain: mode.hiss * load * 0.22, bp: 900 + rn * 2600, q: 0.8 },
        whine: { freq: 0, gain: 0 },
        lp: 220 + rn * 3200 * mode.brightness + throttle * 1400,
        master: (0.16 + 0.84 * load) * (0.55 + 0.45 * rn),
      };
    }
    if (mode.kind === "turbine") {
      // filtered-noise spool + rising resonant whine (compressor-blade tone)
      const whine = 300 + rn * 5200;
      return {
        osc: [{ wave: "sine", freq: whine * 0.5, gain: 0.12 + 0.2 * rn }],
        sub: { wave: "sine", freq: 60 + rn * 40, gain: mode.rumble * (0.4 + 0.6 * load) * 0.4 },
        noise: { gain: (0.25 + 0.6 * load) * 0.4, bp: 1200 + rn * 4000, q: 1.4 + rn * 3 },
        whine: { freq: whine, gain: (0.1 + 0.5 * rn) * 0.3 },
        lp: 1500 + rn * 6000,
        master: (0.2 + 0.8 * load) * (0.5 + 0.5 * rn),
      };
    }
    if (mode.kind === "spaceship") {
      // detuned saw drones a fifth apart + shimmer; sweeps with thrust
      const base = 70 + rn * 260;
      return {
        osc: [
          { wave: "saw", freq: base, gain: 0.3, detune: -6 },
          { wave: "saw", freq: base * 1.5, gain: 0.22, detune: 7 },
          { wave: "saw", freq: base * 2.01, gain: 0.12 * (0.5 + rn) },
        ],
        sub: { wave: "sine", freq: base * 0.5, gain: mode.rumble * (0.5 + 0.5 * load) * 0.5 },
        noise: { gain: mode.hiss * load * 0.12, bp: 2000 + rn * 3000, q: 2 },
        whine: { freq: base * 6, gain: 0.05 * rn },
        lp: 500 + rn * 4500 + throttle * 1500,
        master: (0.25 + 0.75 * load) * (0.6 + 0.4 * rn),
      };
    }
    // warp: deep pulsing sub + harmonic stack that climbs and "jumps" per gear
    const base = 45 + rn * 180;
    return {
      osc: [
        { wave: "square", freq: base, gain: 0.28 },
        { wave: "saw", freq: base * 2, gain: 0.18 + 0.2 * rn, detune: 4 },
        { wave: "saw", freq: base * 3.01, gain: 0.1 * rn },
      ],
      sub: { wave: "sine", freq: base * 0.5, gain: mode.rumble * (0.6 + 0.4 * load) * 0.6 },
      noise: { gain: mode.hiss * load * 0.15, bp: 1500 + rn * 3500, q: 3 },
      whine: { freq: base * 8, gain: 0.04 * rn },
      lp: 400 + rn * 5000 + throttle * 1200,
      master: (0.22 + 0.78 * load) * (0.55 + 0.45 * rn),
    };
  }

  const EngineCore = {
    V, clamp, G,
    MotionMapper, DriveModel,
    MODES, modeById, voicing,
    principalAxis2x2,
  };

  if (typeof module !== "undefined" && module.exports) module.exports = EngineCore;
  else root.EngineCore = EngineCore;
})(typeof self !== "undefined" ? self : this);
