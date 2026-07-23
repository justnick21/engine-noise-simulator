/*
 * sim/test-audio.js — parameter-level checks on voicing() for every mode.
 * We can't listen, so we assert the synth PARAMETERS behave: pitch rises with
 * rpm, loudness rises with load, nothing is NaN or silent, and each mode is
 * distinguishable. Run: node sim/test-audio.js
 */
"use strict";
const EC = require("../engine-core.js");

const RESET = "\x1b[0m", GRN = "\x1b[32m", RED = "\x1b[31m", DIM = "\x1b[90m";
let allPass = true;
const check = (name, ok, detail) => {
  allPass = allPass && ok;
  console.log(`    ${ok ? GRN + "PASS" : RED + "FAIL"}${RESET}  ${name}${detail ? `  ${DIM}(${detail})${RESET}` : ""}`);
};

// primary tone frequency of a VoiceState (first osc, or whine for turbine)
const primaryFreq = (vs) => (vs.osc && vs.osc[0] ? vs.osc[0].freq : vs.whine.freq);
const finite = (vs) => {
  const nums = [vs.lp, vs.master, ...vs.osc.map((o) => o.freq), ...vs.osc.map((o) => o.gain), vs.sub.freq, vs.sub.gain, vs.noise.gain];
  return nums.every((n) => Number.isFinite(n));
};

console.log("\n  ENGINE SIM · audio voicing parameter tests\n  " + "─".repeat(52));
const fingerprint = {}; // mode -> primary freq at 60% throttle, to check distinctness

for (const mode of EC.MODES) {
  console.log(`\n  ${mode.name} ${DIM}(${mode.kind})${RESET}`);
  const drive = new EC.DriveModel();
  // sweep throttle 0 -> 1 over a few seconds and sample voicing
  const samples = [];
  let throttle = 0;
  for (let i = 0; i < 400; i++) {
    throttle = Math.min(1, i / 250);
    const d = drive.step(mode, throttle, 0, 0.02);
    const vs = EC.voicing(mode, d.rpm, throttle, d.speed);
    samples.push({ throttle, rpm: d.rpm, vs, f: primaryFreq(vs), master: vs.master });
  }
  const lo = samples[20], hi = samples[samples.length - 1];

  check("all params finite (no NaN/Inf)", samples.every((s) => finite(s.vs)));
  check("non-silent at load", hi.master > 0.05, `master=${hi.master.toFixed(2)}`);
  check("primary pitch rises with rpm", hi.f > lo.f * 1.3, `${lo.f.toFixed(0)}→${hi.f.toFixed(0)} Hz`);
  check("loudness rises with load", hi.master > lo.master, `${lo.master.toFixed(2)}→${hi.master.toFixed(2)}`);
  check("filter opens under load", hi.vs.lp > lo.vs.lp, `${lo.vs.lp.toFixed(0)}→${hi.vs.lp.toFixed(0)} Hz`);
  // combustion: firing frequency must match rpm/60*cylMult
  if (mode.kind === "combustion") {
    const expect = (hi.rpm / 60) * mode.cylMult;
    check("firing freq = rpm/60·cyl", Math.abs(hi.f - expect) < 1, `got=${hi.f.toFixed(1)} exp=${expect.toFixed(1)}`);
  }
  fingerprint[mode.id] = hi.f;
}

// distinctness: no two modes should share a near-identical top pitch AND kind
const ids = Object.keys(fingerprint);
let distinct = true;
for (let i = 0; i < ids.length; i++)
  for (let j = i + 1; j < ids.length; j++)
    if (Math.abs(fingerprint[ids[i]] - fingerprint[ids[j]]) < 3) distinct = false;
console.log("\n  Cross-mode");
check("modes have distinct top-end pitch", distinct, ids.map((k) => `${k}:${fingerprint[k].toFixed(0)}`).join("  "));

console.log("\n  " + "─".repeat(52));
console.log(`  ${allPass ? GRN + "ALL PASS" : RED + "FAILURES PRESENT"}${RESET}\n`);
process.exit(allPass ? 0 : 1);
