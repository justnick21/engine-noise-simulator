/*
 * sim/test-motion.js — drives synthetic accelerometer streams through the
 * MotionMapper + DriveModel and asserts the throttle behaves correctly.
 *
 *   node sim/test-motion.js
 *
 * Exits non-zero if any assertion fails. Writes per-scenario traces to
 * sim/out/traces.json for plotting.
 */
"use strict";
const fs = require("fs");
const path = require("path");
const EngineCore = require("../engine-core.js");
const M = require("./motion.js");

const OUT = path.join(__dirname, "out");
fs.mkdirSync(OUT, { recursive: true });

// ---------- metrics ----------
const slice = (arr, a, b) => arr.filter((s) => s.t >= a && s.t < b);
const max = (xs) => xs.reduce((m, v) => Math.max(m, v), -Infinity);
const mean = (xs) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
const rms = (xs) => Math.sqrt(mean(xs.map((v) => v * v)));
function corr(a, b) {
  const n = Math.min(a.length, b.length);
  if (!n) return 0;
  const ma = mean(a), mb = mean(b);
  let num = 0, da = 0, db = 0;
  for (let i = 0; i < n; i++) {
    const x = a[i] - ma, y = b[i] - mb;
    num += x * y; da += x * x; db += y * y;
  }
  return da && db ? num / Math.sqrt(da * db) : 0;
}
// RMS of throttle's first difference — a chatter / smoothness measure
function jerkRms(trace) {
  const d = [];
  for (let i = 1; i < trace.length; i++) d.push((trace[i].throttle - trace[i - 1].throttle) / trace[i].dt);
  return rms(d);
}

// ---------- run one scenario through the pipeline ----------
function run(scn, mode) {
  mode = mode || EngineCore.modeById("inline4");
  const stream = M.generate(scn.gen);
  const mapper = new EngineCore.MotionMapper(scn.mapper || {});
  const drive = new EngineCore.DriveModel();
  const trace = [];
  for (const s of stream) {
    const m = mapper.update(s.sample, s.dt);
    const d = drive.step(mode, m.throttle, m.brake, s.dt);
    trace.push({
      t: s.t, dt: s.dt,
      throttle: m.throttle, brake: m.brake,
      longitudinal: m.longitudinal, lateral: m.lateral,
      signLocked: m.signLocked ? 1 : 0,
      rpm: d.rpm, speed: d.speed, gear: d.gear,
      truthAx: s.truth.ax, truthAy: s.truth.ay,
    });
  }
  return trace;
}

// ---------- scenarios ----------
// Every scenario has a stationary lead-in (LEAD) so gravity settles before any
// maneuver — this mirrors real use: mount the phone, tap Start, then drive.
const P = 0.4; // default ramp
const LEAD = 1.6; // s of idle before maneuvers
const scenarios = [];

// 1. Idle, phone flat, engine on — no phantom throttle
scenarios.push({
  id: "idle-flat", title: "Idle · phone flat",
  gen: { duration: 4, mount: { yaw: 0, pitch: 0, roll: 0 }, profile: () => ({ ax: 0, ay: 0, az: 0 }) },
  checks: (tr) => [
    ["throttle stays ~0", max(tr.map((s) => s.throttle)) < 0.05, `max=${max(tr.map((s) => s.throttle)).toFixed(3)}`],
  ],
});

// 2. Idle, phone tilted on a dash mount — gravity rejection
scenarios.push({
  id: "idle-tilt", title: "Idle · 32° pitch / 18° roll mount",
  gen: { duration: 4, mount: { yaw: 40, pitch: 32, roll: 18 }, profile: () => ({ ax: 0, ay: 0, az: 0 }) },
  checks: (tr) => [
    ["no phantom throttle under tilt", max(tr.map((s) => s.throttle)) < 0.08, `max=${max(tr.map((s) => s.throttle)).toFixed(3)}`],
  ],
});

// 3. Hard launch — throttle should rise strongly
scenarios.push({
  id: "launch", title: "Standing launch (+3.0 m/s²)",
  gen: {
    duration: 6, mount: { yaw: 25, pitch: 15, roll: -8 },
    profile: M.asAx(M.pulse(LEAD, LEAD + 3, 3.0, P)),
  },
  checks: (tr) => {
    const push = slice(tr, LEAD + 0.6, LEAD + 2.8);
    return [
      ["throttle exceeds 0.6 during launch", max(push.map((s) => s.throttle)) > 0.6, `max=${max(push.map((s) => s.throttle)).toFixed(2)}`],
      ["forward sign locks", tr.some((s) => s.signLocked), ""],
      ["rpm climbs", max(tr.map((s) => s.rpm)) > 2500, `rpmMax=${Math.round(max(tr.map((s) => s.rpm)))}`],
    ];
  },
});

// 4. Accelerate then brake — braking must NOT phantom-rev
scenarios.push({
  id: "brake", title: "Accelerate, then brake (−5 m/s²)",
  gen: {
    duration: 7, mount: { yaw: -30, pitch: 20, roll: 10 },
    profile: M.combine(M.asAx(M.pulse(LEAD, LEAD + 2, 2.6, P)), M.asAx(M.pulse(LEAD + 2.6, LEAD + 4.2, -5.0, 0.3))),
  },
  checks: (tr) => {
    const braking = slice(tr, LEAD + 2.9, LEAD + 4.0);
    return [
      ["throttle low while braking", max(braking.map((s) => s.throttle)) < 0.2, `max=${max(braking.map((s) => s.throttle)).toFixed(2)}`],
      ["brake signal registers", max(braking.map((s) => s.brake)) > 0.4, `brakeMax=${max(braking.map((s) => s.brake)).toFixed(2)}`],
      ["rpm falls during braking", mean(slice(tr, LEAD + 3.4, LEAD + 4.0).map((s) => s.rpm)) < mean(slice(tr, LEAD + 1.4, LEAD + 1.9).map((s) => s.rpm)), ""],
    ];
  },
});

// 5. Sustained cornering (after a launch establishes the axis) — no phantom rev
scenarios.push({
  id: "corner", title: "Launch, then hard cornering (±4 m/s² lateral)",
  gen: {
    duration: 9, mount: { yaw: 60, pitch: 12, roll: 5 },
    profile: M.combine(
      M.asAx(M.pulse(LEAD, LEAD + 1.7, 2.6, P)),
      M.asAy((t) => (t > LEAD + 2.4 ? 4.0 * Math.sin(2 * Math.PI * 0.25 * (t - LEAD - 2.4)) : 0))
    ),
  },
  checks: (tr) => {
    const cornering = slice(tr, LEAD + 3.4, 9);
    return [
      ["throttle stays low mid-corner", max(cornering.map((s) => s.throttle)) < 0.25, `max=${max(cornering.map((s) => s.throttle)).toFixed(2)}`],
    ];
  },
});

// 6. Rough road — vertical bumps must not cause throttle chatter
scenarios.push({
  id: "bumps", title: "Cruise on rough road (8 Hz vertical bumps)",
  gen: {
    duration: 7.5, mount: { yaw: 10, pitch: 25, roll: -12 },
    profile: M.combine(M.asAx(M.pulse(LEAD, 7.5, 1.7, P)), M.bumps(3.5, 8)),
  },
  checks: (tr) => {
    const cruise = slice(tr, LEAD + 1.5, 7.3);
    return [
      ["throttle roughly tracks cruise accel", mean(cruise.map((s) => s.throttle)) > 0.15 && mean(cruise.map((s) => s.throttle)) < 0.7, `mean=${mean(cruise.map((s) => s.throttle)).toFixed(2)}`],
      ["throttle is smooth (low jerk)", jerkRms(cruise) < 1.2, `jerkRms=${jerkRms(cruise).toFixed(2)}/s`],
    ];
  },
});

// 7. Full drive cycle — throttle should correlate with true forward accel
scenarios.push({
  id: "drive-cycle", title: "Full drive cycle (accel/cruise/brake/corner/bumps)",
  gen: {
    duration: 15.5, mount: { yaw: 75, pitch: 18, roll: 8 },
    profile: M.combine(
      M.asAx(M.pulse(LEAD, LEAD + 2.5, 2.8, P)),
      M.asAx(M.pulse(LEAD + 2.5, LEAD + 5.5, 0.8, P)), // cruise (light accel to hold speed)
      M.asAx(M.pulse(LEAD + 6, LEAD + 7.5, -3.5, 0.3)),
      M.asAx(M.pulse(LEAD + 8.5, LEAD + 11, 2.4, P)),
      M.asAy((t) => (t > LEAD + 8.5 && t < LEAD + 11 ? 3.0 * Math.sin(2 * Math.PI * 0.3 * (t - LEAD - 8.5)) : 0)),
      M.bumps(2.0, 7)
    ),
  },
  checks: (tr) => {
    const drv = slice(tr, LEAD, 15.5);
    const c = corr(drv.map((s) => s.throttle), drv.map((s) => Math.max(0, s.truthAx)));
    return [
      ["throttle correlates with forward accel", c > 0.6, `corr=${c.toFixed(2)}`],
    ];
  },
});

// 8. Orientation independence — same launch at 4 mount yaws
scenarios.push({
  id: "orientation", title: "Orientation independence (launch at yaw 0/60/120/180)",
  custom: () => {
    const peaks = [];
    for (const yaw of [0, 60, 120, 180]) {
      const tr = run({
        gen: { duration: 6, mount: { yaw, pitch: 14, roll: -6 }, profile: M.asAx(M.pulse(LEAD, LEAD + 3, 3.0, P)) },
      });
      peaks.push(max(slice(tr, LEAD + 0.6, LEAD + 2.8).map((s) => s.throttle)));
    }
    const spread = max(peaks) - Math.min(...peaks);
    return [
      ["all orientations reach throttle > 0.5", Math.min(...peaks) > 0.5, `peaks=[${peaks.map((p) => p.toFixed(2)).join(", ")}]`],
      ["peak spread across orientations < 0.15", spread < 0.15, `spread=${spread.toFixed(3)}`],
    ];
  },
});

// ---------- execute ----------
function color(ok) { return ok ? "\x1b[32m" : "\x1b[31m"; }
const RESET = "\x1b[0m";
let allPass = true;
const dump = {};

console.log("\n  ENGINE SIM · motion→throttle mapping tests\n" + "  " + "─".repeat(52));
for (const scn of scenarios) {
  let checks;
  if (scn.custom) {
    checks = scn.custom();
  } else {
    const tr = run(scn);
    dump[scn.id] = tr.filter((_, i) => i % 2 === 0); // decimate for file size
    checks = scn.checks(tr);
  }
  console.log(`\n  ${scn.title}`);
  for (const [name, ok, detail] of checks) {
    allPass = allPass && ok;
    console.log(`    ${color(ok)}${ok ? "PASS" : "FAIL"}${RESET}  ${name}${detail ? "  \x1b[90m(" + detail + ")\x1b[0m" : ""}`);
  }
}
fs.writeFileSync(path.join(OUT, "traces.json"), JSON.stringify(dump));
console.log("\n  " + "─".repeat(52));
console.log(`  ${allPass ? color(true) + "ALL PASS" : color(false) + "FAILURES PRESENT"}${RESET}   traces -> sim/out/traces.json\n`);
process.exit(allPass ? 0 : 1);
