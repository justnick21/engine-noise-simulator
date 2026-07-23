/*
 * sim/test-speed.js — GPS + accelerometer speed fusion.
 *
 * The point of GPS: at constant velocity the accelerometer reads ~0, so an
 * accel-only model sags to idle. Fusing GPS Doppler speed must hold the revs.
 * We also check fusion tracks true speed, survives a GPS dropout via dead
 * reckoning, and stays smooth despite noisy 1 Hz fixes.
 *
 *   node sim/test-speed.js
 */
"use strict";
const EC = require("../engine-core.js");
const M = require("./motion.js");

const GRN = "\x1b[32m", RED = "\x1b[31m", DIM = "\x1b[90m", RESET = "\x1b[0m";
let allPass = true;
const check = (name, ok, detail) => {
  allPass = allPass && ok;
  console.log(`    ${ok ? GRN + "PASS" : RED + "FAIL"}${RESET}  ${name}${detail ? `  ${DIM}(${detail})${RESET}` : ""}`);
};
const mean = (xs) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
const slice = (tr, a, b) => tr.filter((s) => s.t >= a && s.t < b);

// Run a stream through mapper + optional speed estimator + drive model.
function run(stream, { useGps, mode }) {
  mode = mode || EC.modeById("inline4");
  const mapper = new EC.MotionMapper();
  const est = new EC.SpeedEstimator();
  const drive = new EC.DriveModel();
  const tr = [];
  for (const s of stream) {
    const m = mapper.update(s.sample, s.dt);
    let absSpeed;
    if (useGps) {
      est.predict(m.longitudinal, s.dt);
      if (s.gps) est.correctGPS(s.gps.speed);
      absSpeed = est.norm();
    }
    const d = drive.step(mode, m.throttle, m.brake, s.dt, absSpeed);
    tr.push({
      t: s.t, rpm: d.rpm, speedNorm: d.speed,
      estSpeed: useGps ? est.speed : null,
      trueSpeed: s.truth.speed, throttle: m.throttle,
    });
  }
  return tr;
}

const LEAD = 1.6, P = 0.4;
const mode = EC.modeById("inline4");
const idleRpm = mode.idle;

console.log("\n  ENGINE SIM · GPS + accelerometer speed fusion\n  " + "─".repeat(52));

// ---- 1. Cruise: accelerate to speed, then hold constant velocity ----
// accel pulse to build speed, then ax=0 for a long stretch (constant velocity).
{
  const stream = M.addGPS(
    M.generate({
      duration: 14, mount: { yaw: 30, pitch: 16, roll: -6 },
      profile: M.asAx(M.pulse(LEAD, LEAD + 3, 2.6, P)), // then coasts at constant v
    }),
    { hz: 1, latency: 0.6, noise: 0.35 }
  );
  const cruiseWin = [8, 14]; // long after accel ends — pure constant velocity
  const withGps = run(stream, { useGps: true, mode });
  const noGps = run(stream, { useGps: false, mode });

  const trueV = mean(slice(withGps, ...cruiseWin).map((s) => s.trueSpeed));
  const rpmGps = mean(slice(withGps, ...cruiseWin).map((s) => s.rpm));
  const rpmNo = mean(slice(noGps, ...cruiseWin).map((s) => s.rpm));

  console.log(`\n  Constant-velocity cruise ${DIM}(true ~${trueV.toFixed(1)} m/s, accel ≈ 0)${RESET}`);
  check("GPS-fused revs HOLD above idle at cruise", rpmGps > idleRpm + 800, `rpm=${Math.round(rpmGps)} (idle ${idleRpm})`);
  check("accel-only revs SAG toward idle", rpmNo < idleRpm + 700, `rpm=${Math.round(rpmNo)}`);
  check("GPS holds meaningfully higher than accel-only", rpmGps > rpmNo + 800, `${Math.round(rpmNo)} → ${Math.round(rpmGps)}`);
}

// ---- 2. Fusion tracks true speed through accel + cruise + decel ----
{
  const stream = M.addGPS(
    M.generate({
      duration: 16, mount: { yaw: -40, pitch: 22, roll: 10 },
      profile: M.combine(
        M.asAx(M.pulse(LEAD, LEAD + 3, 3.0, P)),
        M.asAx(M.pulse(LEAD + 8, LEAD + 10.5, -3.2, 0.3)) // slow down later
      ),
    }),
    { hz: 1, latency: 0.6, noise: 0.35 }
  );
  const tr = run(stream, { useGps: true, mode });
  const err = slice(tr, 3, 16).map((s) => Math.abs(s.estSpeed - s.trueSpeed));
  check("fused speed tracks truth (mean err < 2 m/s)", mean(err) < 2.0, `meanErr=${mean(err).toFixed(2)} m/s`);
}

// ---- 3. GPS dropout: dead-reckoning keeps it sane, no collapse ----
{
  const stream = M.addGPS(
    M.generate({
      duration: 14, mount: { yaw: 15, pitch: 12, roll: 4 },
      profile: M.asAx(M.pulse(LEAD, LEAD + 3, 2.4, P)),
    }),
    { hz: 1, latency: 0.6, noise: 0.35, dropout: [7, 11] } // 4 s tunnel
  );
  const tr = run(stream, { useGps: true, mode });
  const during = slice(tr, 7.5, 11);
  const trueV = mean(during.map((s) => s.trueSpeed));
  const estV = mean(during.map((s) => s.estSpeed));
  check("survives 4 s GPS dropout (speed within 40%)", Math.abs(estV - trueV) < trueV * 0.4 + 1, `est=${estV.toFixed(1)} true=${trueV.toFixed(1)} m/s`);
  check("revs stay up during dropout", mean(during.map((s) => s.rpm)) > idleRpm + 500, `rpm=${Math.round(mean(during.map((s) => s.rpm)))}`);
}

// ---- 4. Smoothness: noisy 1 Hz fixes must not make rpm jump ----
{
  const stream = M.addGPS(
    M.generate({
      duration: 12, mount: { yaw: 50, pitch: 18, roll: -8 },
      profile: M.asAx(M.pulse(LEAD, LEAD + 3, 2.6, P)),
    }),
    { hz: 1, latency: 0.6, noise: 0.6 } // extra-noisy GPS
  );
  const tr = run(stream, { useGps: true, mode });
  const cruise = slice(tr, 8, 12);
  let jerk = [];
  for (let i = 1; i < cruise.length; i++) jerk.push(Math.abs(cruise[i].rpm - cruise[i - 1].rpm));
  check("rpm stays smooth under noisy fixes", mean(jerk) < 40, `mean Δrpm/frame=${mean(jerk).toFixed(1)}`);
}

console.log("\n  " + "─".repeat(52));
console.log(`  ${allPass ? GRN + "ALL PASS" : RED + "FAILURES PRESENT"}${RESET}\n`);
process.exit(allPass ? 0 : 1);
