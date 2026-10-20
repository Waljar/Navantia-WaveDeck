#!/usr/bin/env node
/* Checks docs/assets/sea.js (waves, kinematics, wave mapping) against the C code.
 *   gcc -I firmware/Core/Inc -o /tmp/dump_ref sim/dump_ref.c firmware/Core/Src/plat_kin.c firmware/Core/Src/plat_waves.c -lm
 *   /tmp/dump_ref > /tmp/ref.txt && node tools/verify-js.js /tmp/ref.txt
 */
const fs = require('fs');
const SEA = require('../docs/assets/sea.js');
const lines = fs.readFileSync(process.argv[2] || '/tmp/ref.txt', 'utf8').trim().split('\n').map((l) => l.split(' '));
let bad = 0, checked = 0;
const near = (what, a, b, tol) => { checked++; if (!(Math.abs(a - b) <= tol)) { bad++; console.log('  MISMATCH', what, 'js', a, 'c', b); } };

const wv = {};
for (const ss of [1, 2, 3, 4]) wv[ss] = new SEA.Waves(ss, 1);
let worstW = 0;
const done = {};
for (const f of lines) {
  if (f[0] === 'stats') {
    const ss = +f[1], s = wv[ss].stats, v = f.slice(2).map(Number);
    const js = [s.rmsH, s.rmsP, s.rmsR, s.rmsHv, s.rmsPv, s.rmsRv, s.rmsHa, s.rmsPa, s.rmsRa];
    js.forEach((x, i) => near('stats ss' + ss + ' #' + i, x, v[i], 2e-5 + 1e-4 * Math.abs(v[i])));
  } else if (f[0] === 'wave') {
    const ss = +f[1], i = +f[2], w = wv[ss];
    while ((done[ss] || 0) < i) { w.advance(0.02); done[ss] = (done[ss] || 0) + 1; }
    const s = w.sample();
    worstW = Math.max(worstW, Math.abs(s.h - +f[3]), Math.abs(s.p - +f[4]) / 10, Math.abs(s.r - +f[5]) / 10);
    near('wave h ss' + ss + ' i' + i, s.h, +f[3], 2e-4); near('wave p', s.p, +f[4], 2e-3); near('wave r', s.r, +f[5], 2e-3);
  }
}
const cal = { min: [672, 700, 798], max: [8456, 8484, 8582], vmax: 1500, amax: 1500 };
const kin = new SEA.Kin(SEA.defaultGeom(), cal);
for (const f of lines) {
  if (f[0] === 'caps') {
    const v = f.slice(1).map(Number), c = kin.caps;
    near('heave range', c.heaveRange, v[0], 0.01); near('heave mid', c.heaveMid, v[1], 0.01);
    near('pitch cap', c.pitchCap, v[2], 0.01); near('roll cap', c.rollCap, v[3], 0.01);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) near('J' + i + j, c.J[i][j], v[4 + i * 3 + j], 0.05);
  } else if (f[0] === 'ik') {
    const v = f.slice(1).map(Number), s = kin.poseToSteps({ h: v[0], p: v[1], r: v[2] });
    if (v[3] !== 0) { checked++; if (s) { bad++; console.log('  MISMATCH ik should fail for', v.slice(0, 3)); } continue; }
    if (!s) { checked++; bad++; console.log('  MISMATCH ik failed in js for', v.slice(0, 3)); continue; }
    for (let i = 0; i < 3; i++) near('ik steps ' + v.slice(0, 3), s[i], v[4 + i], 0.05);
  } else if (f[0] === 'fit') {
    const v = f.slice(1).map(Number), c = kin.caps;
    const r = kin.fit({ h: c.heaveMid, p: 0, r: 0 }, { h: v[0], p: v[1], r: v[2] }, 30);
    near('fit keep ' + v.slice(0, 3), r.keep, v[3], 0.002);
    if (r.steps) for (let i = 0; i < 3; i++) near('fit steps', r.steps[i], v[4 + i], 0.6);
  }
}

/* the same checks with tilt and leg-difference limits switched on */
const lcal = Object.assign({}, cal, { pitchPos: 20, pitchNeg: 15, rollPos: 12, rollNeg: 25, maxDiff: 3000 });
const lkin = new SEA.Kin(SEA.defaultGeom(), lcal);
for (const f of lines) {
  if (f[0] === 'lcaps') {
    const v = f.slice(1).map(Number), c = lkin.caps;
    near('limited heave range', c.heaveRange, v[0], 0.02); near('limited heave mid', c.heaveMid, v[1], 0.02);
    near('limited pitch cap', c.pitchCap, v[2], 0.01); near('limited roll cap', c.rollCap, v[3], 0.01);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) near('limited J' + i + j, c.J[i][j], v[4 + i * 3 + j], 0.05);
  } else if (f[0] === 'lfeas') {
    const v = f.slice(1).map(Number), ok = lkin.feasible({ h: v[0], p: v[1], r: v[2] }, 0) ? 1 : 0;
    checked++; if (ok !== v[3]) { bad++; console.log('  MISMATCH feasible', v.slice(0, 3), 'js', ok, 'c', v[3]); }
  } else if (f[0] === 'lfit') {
    const v = f.slice(1).map(Number), c = lkin.caps;
    const r = lkin.fit({ h: c.heaveMid, p: 0, r: 0 }, { h: v[0], p: v[1], r: v[2] }, 30);
    near('limited fit keep ' + v.slice(0, 3), r.keep, v[3], 0.002);
    if (r.steps) for (let i = 0; i < 3; i++) near('limited fit steps', r.steps[i], v[4 + i], 0.6);
  }
}

/* wave mapping: the percentages and time scales the firmware prints for the same calibration */
const expect = { 1: [54, 'speed limit', 0.50], 2: [54, 'speed limit', 0.50], 3: [54, 'speed limit', 0.50], 4: [54, 'speed limit', 0.50] };
for (const ss of [1, 2, 3, 4]) {
  const m = new SEA.WaveMapper({ kin, ss, maxMode: false });
  const e = expect[ss];
  near('mapper pct ss' + ss, Math.round(m.sigma * 100), e[0], 0); near('mapper ts ss' + ss, m.ts, e[2], 0.006);
  checked++; if (m.limit !== e[1]) { bad++; console.log('  MISMATCH limit', m.limit, e[1]); }
}
const mm = new SEA.WaveMapper({ kin, ss: 4, maxMode: true });
near('MAX pct', Math.round(mm.sigma * 100), 95, 0); near('MAX ts', mm.ts, 0.28, 0.006);

console.log(`${checked} comparisons, ${bad} mismatches (worst wave difference ${worstW.toExponential(2)})`);
process.exit(bad ? 1 : 0);
