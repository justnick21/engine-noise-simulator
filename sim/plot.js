/*
 * sim/plot.js — render sim/out/traces.json to an SVG dashboard so the mapping
 * behaviour can be eyeballed. Run after test-motion.js.
 *   node sim/plot.js
 */
"use strict";
const fs = require("fs");
const path = require("path");
const OUT = path.join(__dirname, "out");
const traces = JSON.parse(fs.readFileSync(path.join(OUT, "traces.json"), "utf8"));

const PANELS = [
  ["launch", "Standing launch — throttle rises with the pull, eases off after"],
  ["brake", "Accelerate then brake — braking reads as brake, never phantom rev"],
  ["corner", "Launch then hard slalom — corner stays classified as lateral (no rev)"],
  ["bumps", "Rough road — vertical bumps rejected, throttle stays smooth"],
  ["drive-cycle", "Full drive cycle — throttle tracks true forward acceleration"],
];

const W = 1040, PH = 150, PADX = 60, PADT = 34, PADB = 26, GAPY = 20;
const H = PANELS.length * (PH + GAPY) + 70;
const C = {
  bg: "#0a0c10", panel: "#0e1218", grid: "#1b2230", axis: "#3a4453",
  text: "#e7edf5", muted: "#8a94a6",
  truth: "#5b8cff", throttle: "#ff5a3c", brake: "#ff3b30", rpm: "#37e0c8",
};
const esc = (s) => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;");

function poly(pts, color, w, opacity = 1, dash = "") {
  const d = pts.map(([x, y], i) => (i ? "L" : "M") + x.toFixed(1) + " " + y.toFixed(1)).join(" ");
  return `<path d="${d}" fill="none" stroke="${color}" stroke-width="${w}" opacity="${opacity}"${dash ? ` stroke-dasharray="${dash}"` : ""} stroke-linejoin="round"/>`;
}

let svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" font-family="ui-monospace, Menlo, Consolas, monospace">`;
svg += `<rect width="${W}" height="${H}" fill="${C.bg}"/>`;
svg += `<text x="${PADX}" y="30" fill="${C.text}" font-size="17" font-weight="700">EV Engine · accelerometer→throttle mapping</text>`;
// legend
const leg = [["true forward accel", C.truth], ["throttle", C.throttle], ["brake", C.brake], ["rpm", C.rpm]];
let lx = W - 20;
for (let i = leg.length - 1; i >= 0; i--) {
  const [lab, col] = leg[i];
  const tw = lab.length * 6.4 + 24;
  lx -= tw;
  svg += `<line x1="${lx}" y1="26" x2="${lx + 16}" y2="26" stroke="${col}" stroke-width="3"/>`;
  svg += `<text x="${lx + 20}" y="30" fill="${C.muted}" font-size="11">${esc(lab)}</text>`;
}

PANELS.forEach(([id, caption], idx) => {
  const tr = traces[id];
  if (!tr || !tr.length) return;
  const y0 = 56 + idx * (PH + GAPY);
  const px = PADX, pw = W - PADX - 20, pt = y0 + PADT, ph = PH - PADT - PADB;
  const tMax = tr[tr.length - 1].t;
  const X = (t) => px + (t / tMax) * pw;
  const Y = (v) => pt + (1 - v) * ph; // v in 0..1
  const rpmMax = Math.max(...tr.map((s) => s.rpm), 1);
  const accMax = 6; // m/s^2 full-scale for display

  svg += `<rect x="${px}" y="${pt}" width="${pw}" height="${ph}" fill="${C.panel}" stroke="${C.grid}" rx="4"/>`;
  // gridlines at throttle 0/0.5/1
  [0, 0.5, 1].forEach((g) => {
    svg += `<line x1="${px}" y1="${Y(g)}" x2="${px + pw}" y2="${Y(g)}" stroke="${C.grid}" stroke-width="1"/>`;
    svg += `<text x="${px - 8}" y="${Y(g) + 3}" fill="${C.muted}" font-size="10" text-anchor="end">${g}</text>`;
  });
  // zero line for accel (maps to Y(0.5) baseline with accel scaled ±)
  svg += `<text x="${px + 6}" y="${pt + 12}" fill="${C.text}" font-size="11" font-weight="600">${esc(caption)}</text>`;

  // true forward accel (signed → scaled to 0..1 around 0.5? show only positive part above baseline, negative below)
  const accPts = tr.map((s) => [X(s.t), Y(0.5 + Math.max(-1, Math.min(1, s.truthAx / accMax)) * 0.5)]);
  svg += poly(accPts, C.truth, 1.5, 0.7);
  // rpm (scaled to 0..1)
  svg += poly(tr.map((s) => [X(s.t), Y(s.rpm / rpmMax)]), C.rpm, 1.2, 0.5, "3 3");
  // brake fill-ish line
  svg += poly(tr.map((s) => [X(s.t), Y(s.brake)]), C.brake, 1.5, 0.85);
  // throttle (headline)
  svg += poly(tr.map((s) => [X(s.t), Y(s.throttle)]), C.throttle, 2.4, 1);

  // x ticks
  for (let t = 0; t <= tMax + 0.001; t += Math.ceil(tMax / 6)) {
    svg += `<line x1="${X(t)}" y1="${pt + ph}" x2="${X(t)}" y2="${pt + ph + 4}" stroke="${C.axis}"/>`;
    svg += `<text x="${X(t)}" y="${pt + ph + 16}" fill="${C.muted}" font-size="10" text-anchor="middle">${t}s</text>`;
  }
});

svg += `</svg>`;
fs.writeFileSync(path.join(OUT, "mapping.svg"), svg);
console.log("wrote sim/out/mapping.svg");
