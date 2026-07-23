/*
 * sim/render-audio.js — headless DSP renderer that turns voicing() into PCM,
 * faithfully mirroring the Web Audio graph (band-limited additive oscillators =
 * PeriodicWave, RBJ biquads, looped noise). Renders a rev sweep per mode to a
 * .wav, and asserts each is non-silent, un-clipped, and rises in pitch.
 *
 *   node sim/render-audio.js
 */
"use strict";
const fs = require("fs");
const path = require("path");
const EC = require("../engine-core.js");

const SR = 44100;
const OUT = path.join(__dirname, "out");
fs.mkdirSync(OUT, { recursive: true });

// ---------- wavetables (match Web Audio PeriodicWave harmonic content) ----------
const TLEN = 2048, HMAX = 40;
function tableFromImag(imag) {
  const t = new Float32Array(TLEN);
  for (let i = 0; i < TLEN; i++) {
    const ph = (i / TLEN) * 2 * Math.PI;
    let s = 0;
    for (let n = 1; n < imag.length; n++) s += imag[n] * Math.sin(n * ph);
    t[i] = s;
  }
  // normalize
  let peak = 0;
  for (let i = 0; i < TLEN; i++) peak = Math.max(peak, Math.abs(t[i]));
  if (peak > 0) for (let i = 0; i < TLEN; i++) t[i] /= peak;
  return t;
}
function engineImag(harmonics, oddBias) {
  const im = new Float32Array(harmonics + 1);
  for (let n = 1; n <= harmonics; n++) {
    let a = 1 / n;
    if (n % 2 === 1) a *= oddBias;
    a *= Math.exp(-n / (harmonics * 0.9));
    im[n] = a;
  }
  return im;
}
function basicImag(kind) {
  const im = new Float32Array(HMAX + 1);
  for (let n = 1; n <= HMAX; n++) {
    if (kind === "sine") im[n] = n === 1 ? 1 : 0;
    else if (kind === "saw") im[n] = 1 / n;
    else if (kind === "square") im[n] = n % 2 ? 1 / n : 0;
    else if (kind === "triangle") im[n] = n % 2 ? (1 / (n * n)) * (((n - 1) / 2) % 2 ? -1 : 1) : 0;
  }
  return im;
}
const TABLES = {
  sine: tableFromImag(basicImag("sine")),
  saw: tableFromImag(basicImag("saw")),
  square: tableFromImag(basicImag("square")),
  triangle: tableFromImag(basicImag("triangle")),
};
function engineTable(mode) {
  return tableFromImag(engineImag(mode.harmonics || 18, mode.oddBias || 1));
}

// ---------- oscillator ----------
function Osc() { return { phase: 0 }; }
function oscSample(o, table, freq) {
  o.phase += freq / SR;
  o.phase -= Math.floor(o.phase);
  const x = o.phase * TLEN;
  const i = x | 0, f = x - i;
  const a = table[i], b = table[(i + 1) & (TLEN - 1)];
  return a + (b - a) * f;
}

// ---------- RBJ biquad ----------
function Biquad() { return { b0: 1, b1: 0, b2: 0, a1: 0, a2: 0, z1: 0, z2: 0 }; }
function setLP(bq, f0, Q) {
  f0 = Math.min(f0, SR * 0.45);
  const w = (2 * Math.PI * f0) / SR, cw = Math.cos(w), sw = Math.sin(w);
  const al = sw / (2 * Q);
  const b0 = (1 - cw) / 2, b1 = 1 - cw, b2 = (1 - cw) / 2;
  const a0 = 1 + al, a1 = -2 * cw, a2 = 1 - al;
  bq.b0 = b0 / a0; bq.b1 = b1 / a0; bq.b2 = b2 / a0; bq.a1 = a1 / a0; bq.a2 = a2 / a0;
}
function setBP(bq, f0, Q) {
  f0 = Math.min(f0, SR * 0.45);
  const w = (2 * Math.PI * f0) / SR, cw = Math.cos(w), sw = Math.sin(w);
  const al = sw / (2 * Q);
  const b0 = al, b1 = 0, b2 = -al;
  const a0 = 1 + al, a1 = -2 * cw, a2 = 1 - al;
  bq.b0 = b0 / a0; bq.b1 = b1 / a0; bq.b2 = b2 / a0; bq.a1 = a1 / a0; bq.a2 = a2 / a0;
}
function bqSample(bq, x) {
  const y = bq.b0 * x + bq.z1;
  bq.z1 = bq.b1 * x - bq.a1 * y + bq.z2;
  bq.z2 = bq.b2 * x - bq.a2 * y;
  return y;
}

// one-pole smoother (emulates setTargetAtTime)
const smoothCoef = (tc) => Math.exp(-1 / (SR * tc));

// ---------- render one mode ----------
function render(mode, seconds) {
  const N = Math.round(seconds * SR);
  const buf = new Float32Array(N);
  const drive = new EC.DriveModel();
  const engT = engineTable(mode);
  const tableFor = (w) => (w === "engine" ? engT : TABLES[w === "saw" ? "saw" : w] || TABLES.saw);

  const osc = [Osc(), Osc(), Osc()];
  const sub = Osc(), whine = Osc();
  let noiseState = 0;
  const lp = Biquad(), bp = Biquad();

  // smoothed params
  const sc = smoothCoef(0.02);
  const sm = { of: [0, 0, 0], og: [0, 0, 0], subf: 0, subg: 0, nf: 1000, ng: 0, wf: 400, wg: 0, lp: 400, master: 0 };
  const CTRL = 64;
  let vs = null, waves = ["saw", "saw", "saw"];

  // throttle profile: idle, rev up, hold, release, blip
  function throttleAt(t) {
    if (t < 1) return 0;
    if (t < 3) return (t - 1) / 2;         // ramp to full
    if (t < 4) return 1;                    // hold
    if (t < 5.2) return 1 - (t - 4) / 1.2 * 0.8; // release toward 0.2
    return 0.2 + 0.6 * Math.max(0, Math.sin((t - 5.2) * 8)); // blips
  }

  for (let i = 0; i < N; i++) {
    const t = i / SR;
    if (i % CTRL === 0) {
      const thr = throttleAt(t);
      const d = drive.step(mode, thr, 0, CTRL / SR);
      vs = EC.voicing(mode, d.rpm, thr, d.speed);
      for (let k = 0; k < 3; k++) waves[k] = vs.osc[k] ? vs.osc[k].wave || "saw" : waves[k];
    }
    // update targets
    for (let k = 0; k < 3; k++) {
      const spec = vs.osc[k];
      const tf = spec ? spec.freq * Math.pow(2, (spec.detune || 0) / 1200) : sm.of[k];
      sm.of[k] += (tf - sm.of[k]) * (1 - sc);
      sm.og[k] += ((spec ? spec.gain : 0) - sm.og[k]) * (1 - sc);
    }
    sm.subf += (Math.max(8, vs.sub.freq) - sm.subf) * (1 - sc);
    sm.subg += (vs.sub.gain - sm.subg) * (1 - sc);
    sm.nf += (vs.noise.bp - sm.nf) * (1 - sc);
    sm.ng += (vs.noise.gain - sm.ng) * (1 - sc);
    sm.wf += (Math.max(20, vs.whine.freq) - sm.wf) * (1 - sc);
    sm.wg += (vs.whine.gain - sm.wg) * (1 - sc);
    sm.lp += (vs.lp - sm.lp) * (1 - sc);
    sm.master += (vs.master - sm.master) * (1 - sc);

    if (i % CTRL === 0) { setLP(lp, sm.lp, 0.7); setBP(bp, sm.nf, vs.noise.q || 1); }

    // tonal sum -> lowpass
    let tone = 0;
    for (let k = 0; k < 3; k++) tone += oscSample(osc[k], tableFor(waves[k]), sm.of[k]) * sm.og[k];
    let out = bqSample(lp, tone);
    // sub
    out += oscSample(sub, TABLES.sine, sm.subf) * sm.subg * 1.4;
    // noise
    noiseState = Math.random() * 2 - 1;
    out += bqSample(bp, noiseState) * sm.ng * 2.5;
    // whine
    out += oscSample(whine, TABLES.sine, sm.wf) * sm.wg;

    out *= sm.master * 0.9;
    buf[i] = Math.tanh(out * 1.2); // gentle limiter
  }
  return buf;
}

// ---------- WAV ----------
function writeWav(file, samples) {
  const N = samples.length;
  const b = Buffer.alloc(44 + N * 2);
  b.write("RIFF", 0); b.writeUInt32LE(36 + N * 2, 4); b.write("WAVE", 8);
  b.write("fmt ", 12); b.writeUInt32LE(16, 16); b.writeUInt16LE(1, 20); b.writeUInt16LE(1, 22);
  b.writeUInt32LE(SR, 24); b.writeUInt32LE(SR * 2, 28); b.writeUInt16LE(2, 32); b.writeUInt16LE(16, 34);
  b.write("data", 36); b.writeUInt32LE(N * 2, 40);
  for (let i = 0; i < N; i++) {
    const s = Math.max(-1, Math.min(1, samples[i]));
    b.writeInt16LE((s * 32767) | 0, 44 + i * 2);
  }
  fs.writeFileSync(file, b);
}

// ---------- naive DFT dominant freq over a window ----------
function dominantFreq(samples, start, len) {
  const lo = 20, hi = 2000, step = 5;
  let best = 0, bestMag = 0;
  for (let f = lo; f <= hi; f += step) {
    let re = 0, im = 0;
    for (let n = 0; n < len; n++) {
      const s = samples[start + n];
      const a = (2 * Math.PI * f * n) / SR;
      re += s * Math.cos(a); im += s * Math.sin(a);
    }
    const mag = re * re + im * im;
    if (mag > bestMag) { bestMag = mag; best = f; }
  }
  return best;
}
// energy-weighted mean frequency — a brightness measure that works for tonal
// engines AND noise-driven turbines alike
function spectralCentroid(samples, start, len) {
  let num = 0, den = 0;
  for (let f = 50; f <= 8000; f += 50) {
    let re = 0, im = 0;
    for (let n = 0; n < len; n++) {
      const a = (2 * Math.PI * f * n) / SR;
      re += samples[start + n] * Math.cos(a); im += samples[start + n] * Math.sin(a);
    }
    const mag = Math.sqrt(re * re + im * im);
    num += mag * f; den += mag;
  }
  return den ? num / den : 0;
}
const rms = (s, a, b) => { let x = 0; for (let i = a; i < b; i++) x += s[i] * s[i]; return Math.sqrt(x / (b - a)); };
const peak = (s) => { let p = 0; for (let i = 0; i < s.length; i++) p = Math.max(p, Math.abs(s[i])); return p; };

// ---------- run ----------
const GRN = "\x1b[32m", RED = "\x1b[31m", DIM = "\x1b[90m", RESET = "\x1b[0m";
let allPass = true;
console.log("\n  ENGINE SIM · audio render + spectral checks\n  " + "─".repeat(52));
for (const mode of EC.MODES) {
  const buf = render(mode, 6);
  const file = path.join(OUT, mode.id + ".wav");
  writeWav(file, buf);
  const cIdle = spectralCentroid(buf, Math.round(0.5 * SR), 4096);
  const cRev = spectralCentroid(buf, Math.round(3.3 * SR), 4096);
  const fRev = dominantFreq(buf, Math.round(3.3 * SR), 4096);
  const pk = peak(buf), rIdle = rms(buf, 0.4 * SR | 0, 0.9 * SR | 0), rRev = rms(buf, 3.1 * SR | 0, 3.6 * SR | 0);
  const checks = [
    ["renders non-silent", rRev > 0.02, `rms=${rRev.toFixed(3)}`],
    ["no clipping", pk <= 1.0, `peak=${pk.toFixed(3)}`],
    ["louder under load", rRev > rIdle, `${rIdle.toFixed(3)}→${rRev.toFixed(3)}`],
    ["brightens with revs (centroid)", cRev > cIdle * 1.05, `${cIdle.toFixed(0)}→${cRev.toFixed(0)} Hz`],
    ["tonal peak at rev", fRev > 30, `${fRev} Hz`],
  ];
  console.log(`\n  ${mode.name} ${DIM}→ ${mode.id}.wav${RESET}`);
  for (const [n, ok, d] of checks) {
    allPass = allPass && ok;
    console.log(`    ${ok ? GRN + "PASS" : RED + "FAIL"}${RESET}  ${n}  ${DIM}(${d})${RESET}`);
  }
}
console.log("\n  " + "─".repeat(52));
console.log(`  ${allPass ? GRN + "ALL PASS" : RED + "FAILURES"}${RESET}   wavs -> sim/out/*.wav\n`);
process.exit(allPass ? 0 : 1);
