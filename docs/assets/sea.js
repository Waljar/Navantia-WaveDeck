/*!
 * sea.js - the platform's maths in JavaScript, line for line the same as the
 * firmware (plat_waves.c, plat_kin.c, mode_waves.c). tools/verify-js.js checks
 * it against the C code. Also holds the 3D drawing, the strip chart and the
 * Web Serial link used by the pages.
 */
(function (root) {
  'use strict';
  const SEA = {};
  const TWO_PI = 2 * Math.PI, DEG = Math.PI / 180, RAD = 180 / Math.PI;

  /* ---- settings that match firmware/Core/Inc/plat_config.h ------------------ */
  SEA.CFG = {
    stepsPerRev: 20000, stepsPerDeg: 20000 / 360,
    waveComponents: 24, seed: 1, hsExponent: 0.75, vesselWc: 2.0, pitchGain: 1.0, rollGain: 0.8,
    useNormal: 0.65, useMax: 0.95, speedUse: 0.85, timeFloorNormal: 0.5, timeFloorMax: 0.2,
    k98: 2.33, peak: 3.3, fitMargin: 30, ctrlHz: 100, g: 9.81
  };
  SEA.SEA_STATES = [null,
    { ss: 1, hs: 0.05, tp: 2.5, name: 'Sea state 1', word: 'calm (rippled)' },
    { ss: 2, hs: 0.30, tp: 4.0, name: 'Sea state 2', word: 'smooth' },
    { ss: 3, hs: 0.90, tp: 6.0, name: 'Sea state 3', word: 'slight' },
    { ss: 4, hs: 1.90, tp: 8.0, name: 'Sea state 4', word: 'moderate' }];

  /* geometry placeholders: replace with the measured numbers (mm, degrees) */
  SEA.defaultGeom = function () {
    return { r: 500, L: 620, Rb: 554.3, Rp: 207.8, thetaMinDeg: -68, crankAzDeg: 180, sense: 1,
             legAzDeg: [0, 240, 120], configured: true };
  };
  SEA.defaultCal = function () {
    /* pitchPos .. rollNeg: largest tilt found by hand in each direction, degrees (0 = no limit);
       maxDiff: largest difference between the heights of two legs, steps (0 = no limit) - see Setup 4 and 5 */
    return { min: [672, 700, 798], max: [8567, 8595, 8693], vmax: 1500, amax: 1500,
             pitchPos: 0, pitchNeg: 0, rollPos: 0, rollNeg: 0, maxDiff: 0 };
  };

  /* ---- irregular sea (plat_waves.c) --------------------------------------- */
  class Waves {
    constructor(ss, seed) { this.init(ss, seed === undefined ? SEA.CFG.seed : seed); }
    rand01() { this.rng = (Math.imul(this.rng, 1664525) + 1013904223) >>> 0; return (this.rng >>> 8) / 16777216; }
    init(ss, seed) {
      const C = SEA.CFG, n = C.waveComponents;
      ss = Math.min(4, Math.max(1, ss | 0));
      const s = SEA.SEA_STATES[ss], s4 = SEA.SEA_STATES[4];
      const hs = s4.hs * Math.pow(s.hs / s4.hs, C.hsExponent);
      const wp = TWO_PI / s.tp, wmin = 0.5 * wp, wmax = 3.5 * wp, dw = (wmax - wmin) / n;
      this.ss = ss; this.hs = hs; this.seed = seed >>> 0;
      this.rng = (seed >>> 0) || 1;
      this.W = new Float64Array(n); this.AH = new Float64Array(n); this.AP = new Float64Array(n); this.AR = new Float64Array(n);
      this.PH0 = new Float64Array(n); this.PR0 = new Float64Array(n); this.PH = new Float64Array(n); this.PR = new Float64Array(n);
      let sum = 0;
      for (let k = 0; k < n; k++) {
        const jit = this.rand01() - 0.5;
        const w = wmin + (k + 0.5 + 0.8 * jit) * dw;
        const spec = (5 / 16) * hs * hs * Math.pow(wp, 4) * Math.pow(w, -5) * Math.exp(-1.25 * Math.pow(wp / w, 4));
        this.W[k] = w; this.AH[k] = Math.sqrt(2 * spec * dw);
        this.PH0[k] = TWO_PI * this.rand01(); this.PR0[k] = TWO_PI * this.rand01();
        sum += 0.5 * this.AH[k] * this.AH[k];
      }
      const norm = sum > 0 ? Math.sqrt((hs / 4) * (hs / 4) / sum) : 1;
      const st = { rmsH: 0, rmsP: 0, rmsR: 0, rmsHv: 0, rmsPv: 0, rmsRv: 0, rmsHa: 0, rmsPa: 0, rmsRa: 0 };
      for (let k = 0; k < n; k++) {
        const w = this.W[k];
        const rao = 1 / Math.sqrt(1 + Math.pow(w / C.vesselWc, 4));
        const a = this.AH[k] * norm;
        const slope = a * (w * w / C.g) * rao * RAD;
        this.AH[k] = a * rao; this.AP[k] = C.pitchGain * slope; this.AR[k] = C.rollGain * slope;
        const w2 = w * w, w4 = w2 * w2;
        st.rmsH += 0.5 * this.AH[k] ** 2; st.rmsP += 0.5 * this.AP[k] ** 2; st.rmsR += 0.5 * this.AR[k] ** 2;
        st.rmsHv += 0.5 * w2 * this.AH[k] ** 2; st.rmsPv += 0.5 * w2 * this.AP[k] ** 2; st.rmsRv += 0.5 * w2 * this.AR[k] ** 2;
        st.rmsHa += 0.5 * w4 * this.AH[k] ** 2; st.rmsPa += 0.5 * w4 * this.AP[k] ** 2; st.rmsRa += 0.5 * w4 * this.AR[k] ** 2;
      }
      for (const key in st) st[key] = Math.sqrt(st[key]);
      this.stats = st;
      this.reset();
    }
    reset() { this.PH.set(this.PH0); this.PR.set(this.PR0); }
    advance(dt) {
      for (let k = 0; k < this.W.length; k++) {
        const d = this.W[k] * dt;
        this.PH[k] = (this.PH[k] + d) % TWO_PI; this.PR[k] = (this.PR[k] + d) % TWO_PI;
      }
    }
    /* heave (m), pitch (deg), roll (deg) of the vessel */
    sample() {
      let h = 0, p = 0, r = 0;
      for (let k = 0; k < this.W.length; k++) {
        h += this.AH[k] * Math.cos(this.PH[k]);
        p += this.AP[k] * Math.sin(this.PH[k]);
        r += this.AR[k] * Math.sin(this.PR[k]);
      }
      return { h, p, r };
    }
  }
  SEA.Waves = Waves;

  /* ---- kinematics (plat_kin.c) ---------------------------------------------- */
  const wrapPi = (a) => { while (a > Math.PI) a -= TWO_PI; while (a < -Math.PI) a += TWO_PI; return a; };

  class Kin {
    constructor(geom, cal) {
      this.G = Object.assign({}, geom, { legAzDeg: geom.legAzDeg.slice() });
      this.cal = cal;
      this.ok = this.setup();
    }
    setup() {
      const G = this.G, n = 3;
      this.thetaM = G.thetaMinDeg * DEG;
      this.B = []; this.U = []; this.J = []; this.branch = [0, 0, 0];
      for (let i = 0; i < n; i++) {
        const b = G.legAzDeg[i] * DEG, c = (G.legAzDeg[i] + G.crankAzDeg) * DEG;
        this.B.push([G.Rb * Math.cos(b), G.Rb * Math.sin(b)]);
        this.U.push([Math.cos(c), Math.sin(c)]);
        this.J.push([G.Rp * Math.cos(b), G.Rp * Math.sin(b)]);
      }
      { /* joint height at the level minimum (leg 2) */
        const Cx = this.B[2][0] + G.r * Math.cos(this.thetaM) * this.U[2][0];
        const Cy = this.B[2][1] + G.r * Math.cos(this.thetaM) * this.U[2][1];
        const Cz = G.r * Math.sin(this.thetaM);
        const h2 = (this.J[2][0] - Cx) ** 2 + (this.J[2][1] - Cy) ** 2;
        if (G.L * G.L <= h2) return false;
        this.z0 = Cz + Math.sqrt(G.L * G.L - h2);
      }
      for (let i = 0; i < n; i++) {
        const Dx = this.J[i][0] - this.B[i][0], Dy = this.J[i][1] - this.B[i][1], Dz = this.z0;
        const A = Dx * this.U[i][0] + Dy * this.U[i][1];
        const D2 = Dx * Dx + Dy * Dy + Dz * Dz;
        const Cc = (D2 + G.r * G.r - G.L * G.L) / (2 * G.r);
        const R = Math.hypot(A, Dz);
        if (R < 1e-6 || Math.abs(Cc) > R) return false;
        const base = Math.atan2(Dz, A), d = Math.acos(Cc / R);
        this.branch[i] = Math.abs(wrapPi(base + d - this.thetaM)) <= Math.abs(wrapPi(base - d - this.thetaM)) ? 1 : -1;
      }
      this.ready = true;
      const heaveOk = (h) => this.feasible({ h, p: 0, r: 0 }, 0);
      let hi = 20, guard = 0;
      while (heaveOk(hi) && guard++ < 12) hi *= 2;
      let lo = 0;
      for (let it = 0; it < 30; it++) { const m = 0.5 * (lo + hi); if (heaveOk(m)) lo = m; else hi = m; }
      const caps = { heaveRange: lo, heaveMid: 0.5 * lo, J: [[0, 0, 0], [0, 0, 0], [0, 0, 0]] };
      this.caps = caps;
      const tilt = (roll) => {
        let a = 0, b = 45;
        for (let it = 0; it < 24; it++) {
          const m = 0.5 * (a + b);
          const p1 = { h: caps.heaveMid, p: roll ? 0 : m, r: roll ? m : 0 }, p2 = { h: caps.heaveMid, p: roll ? 0 : -m, r: roll ? -m : 0 };
          if (this.feasible(p1, 0) && this.feasible(p2, 0)) a = m; else b = m;
        }
        return a;
      };
      caps.pitchCap = tilt(false); caps.rollCap = tilt(true);
      const mid = caps.heaveMid, col = (pa, pb, den, c) => {
        const sa = this.poseToSteps(pa), sb = this.poseToSteps(pb);
        if (!sa || !sb) return false;
        for (let i = 0; i < 3; i++) caps.J[i][c] = (sa[i] - sb[i]) / den;
        return true;
      };
      return col({ h: mid + 1, p: 0, r: 0 }, { h: mid - 1, p: 0, r: 0 }, 2, 0) &&
             col({ h: mid, p: 0.25, r: 0 }, { h: mid, p: -0.25, r: 0 }, 0.5, 1) &&
             col({ h: mid, p: 0, r: 0.25 }, { h: mid, p: 0, r: -0.25 }, 0.5, 2);
    }
    legSolve(i, Px, Py, Pz) {
      const G = this.G, Dx = Px - this.B[i][0], Dy = Py - this.B[i][1], Dz = Pz;
      const A = Dx * this.U[i][0] + Dy * this.U[i][1];
      const D2 = Dx * Dx + Dy * Dy + Dz * Dz;
      const Cc = (D2 + G.r * G.r - G.L * G.L) / (2 * G.r);
      const R = Math.hypot(A, Dz);
      if (R < 1e-6 || Math.abs(Cc) > R) return null;
      return Math.atan2(Dz, A) + this.branch[i] * Math.acos(Cc / R);
    }
    /* full solution for drawing: joint positions, crank angles, crank pins, steps */
    solve(pose) {
      if (!this.ready) return null;
      const sp = Math.sin(pose.p * DEG), cp = Math.cos(pose.p * DEG), sr = Math.sin(pose.r * DEG), cr = Math.cos(pose.r * DEG);
      const z = this.z0 + pose.h, G = this.G, out = { P: [], C: [], theta: [], steps: [] };
      for (let i = 0; i < 3; i++) {
        const x = this.J[i][0], y = this.J[i][1];
        const P = [x * cp, y * cr - x * sp * sr, y * sr + x * sp * cr + z];
        const th = this.legSolve(i, P[0], P[1], P[2]);
        if (th === null) return null;
        out.P.push(P); out.theta.push(th);
        out.C.push([this.B[i][0] + G.r * Math.cos(th) * this.U[i][0], this.B[i][1] + G.r * Math.cos(th) * this.U[i][1], G.r * Math.sin(th)]);
        out.steps.push(this.cal.min[i] + G.sense * wrapPi(th - this.thetaM) * RAD * SEA.CFG.stepsPerDeg);
      }
      return out;
    }
    poseToSteps(pose) { const s = this.solve(pose); return s ? s.steps : null; }
    feasible(pose, margin) {
      const c = this.cal;
      if (c.pitchPos > 0 && pose.p > c.pitchPos + 1e-4) return false;
      if (c.pitchNeg > 0 && pose.p < -c.pitchNeg - 1e-4) return false;
      if (c.rollPos > 0 && pose.r > c.rollPos + 1e-4) return false;
      if (c.rollNeg > 0 && pose.r < -c.rollNeg - 1e-4) return false;
      const s = this.poseToSteps(pose);
      if (!s) return false;
      let lo = 1e9, hi = -1e9;
      for (let i = 0; i < 3; i++) {
        if (s[i] < c.min[i] + margin - 0.5 || s[i] > c.max[i] - margin + 0.5) return false;
        const rel = s[i] - c.min[i];
        if (rel < lo) lo = rel;
        if (rel > hi) hi = rel;
      }
      if (c.maxDiff > 0 && hi - lo > c.maxDiff + 0.5) return false;
      return true;
    }
    /* shrink `dev` about `center` until every leg is inside its limits (kin_fit) */
    fit(center, dev, margin) {
      const at = (k) => ({ h: center.h + k * dev.h, p: center.p + k * dev.p, r: center.r + k * dev.r });
      let c = at(1);
      if (this.feasible(c, margin)) { const s = this.poseToSteps(c); return s ? { keep: 1, pose: c, steps: s } : { keep: -1 }; }
      if (!this.feasible(center, margin)) return { keep: -1, pose: center, steps: this.poseToSteps(center) };
      let lo = 0, hi = 1;
      for (let it = 0; it < 12; it++) { const m = 0.5 * (lo + hi); if (this.feasible(at(m), margin)) lo = m; else hi = m; }
      const out = at(lo), s = this.poseToSteps(out);
      return s ? { keep: lo, pose: out, steps: s } : { keep: -1 };
    }
  }
  SEA.Kin = Kin;

  /* ---- sea -> platform mapping (mode_waves.c) ------------------------------- */
  const smooth = (x) => { x = x < 0 ? 0 : x > 1 ? 1 : x; return x * x * (3 - 2 * x); };

  class WaveMapper {
    /* opts: {kin, ss, maxMode, seed, tscale, trim} */
    constructor(opts) {
      this.kin = opts.kin; this.maxMode = !!opts.maxMode; this.ss = opts.ss || 3;
      this.seed = opts.seed === undefined ? SEA.CFG.seed : opts.seed;
      this.tscale = opts.tscale || 1; this.trim = opts.trim || 1;
      this.sweep = false; this.seg = 0; this.sweepT = 0; this.rampT = 0;
      this.prepare();
    }
    prepare() {
      const C = SEA.CFG, caps = this.kin.caps, cal = this.kin.cal;
      const ref = new Waves(this.maxMode ? this.ss : 4, this.seed).stats;
      this.waves = new Waves(this.ss, this.seed);
      this.refH = C.k98 * ref.rmsH; this.refP = C.k98 * ref.rmsP; this.refR = C.k98 * ref.rmsR;
      const capH = 0.5 * caps.heaveRange, capP = caps.pitchCap, capR = caps.rollCap;
      const vH = capH * ref.rmsHv / this.refH, vP = capP * ref.rmsPv / this.refP, vR = capR * ref.rmsRv / this.refR;
      const aH = capH * ref.rmsHa / this.refH, aP = capP * ref.rmsPa / this.refP, aR = capR * ref.rmsRa / this.refR;
      let maxv = 1e-3, maxa = 1e-3;
      for (let i = 0; i < 3; i++) {
        maxv = Math.max(maxv, Math.hypot(caps.J[i][0] * vH, caps.J[i][1] * vP, caps.J[i][2] * vR));
        maxa = Math.max(maxa, Math.hypot(caps.J[i][0] * aH, caps.J[i][1] * aP, caps.J[i][2] * aR));
      }
      const sAmp = this.maxMode ? C.useMax : C.useNormal;
      const kV = C.speedUse * cal.vmax / (C.peak * maxv), kA = C.speedUse * cal.amax / (C.peak * maxa);
      const tsLim = Math.min(kV / sAmp, Math.sqrt(kA / sAmp));
      const floor = this.maxMode ? C.timeFloorMax : C.timeFloorNormal;
      const auto = tsLim > 1 ? 1 : tsLim < floor ? floor : tsLim;
      this.ts = auto * this.tscale;
      const sV = kV / this.ts, sA = kA / (this.ts * this.ts);
      this.sigma = sAmp; this.limit = 'amplitude';
      if (sV < this.sigma) { this.sigma = sV; this.limit = 'speed limit'; }
      if (sA < this.sigma) { this.sigma = sA; this.limit = 'acceleration limit'; }
      this.caps = { capH, capP, capR };
    }
    setSeaState(ss) { this.ss = ss; this.rampT = 0; this.prepare(); }
    newSeed() { this.seed = (Math.imul(this.seed, 1664525) + 1013904223) >>> 0; this.rampT = 0; this.prepare(); }
    setTimeScale(t) { const w = this.waves; this.tscale = t; this.prepare(); this.waves = w; }
    setSweep(on) { this.sweep = on; this.sweepT = 0; this.seg = 0; this.rampT = 0; }
    /* one control tick (dt seconds) -> deviation about the mid-heave pose */
    step(dt) {
      const dev = { h: 0, p: 0, r: 0 }, k = this.kin, caps = k.caps, C = SEA.CFG;
      this.rampT += dt;
      const ramp = smooth(this.rampT / 4);
      if (!this.sweep) {
        this.waves.advance(dt * this.ts);
        const s = this.waves.sample(), g = this.sigma * this.trim * ramp;
        dev.h = g * this.caps.capH * s.h / this.refH;
        dev.p = g * this.caps.capP * s.p / this.refP;
        dev.r = g * this.caps.capR * s.r / this.refR;
      } else {
        const seg = this.seg, cap = seg === 0 ? this.caps.capH : seg === 1 ? this.caps.capP : this.caps.capR, amp = 0.95 * cap;
        let steps = 1;
        for (let i = 0; i < 3; i++) steps = Math.max(steps, Math.abs(caps.J[i][seg]) * amp);
        let om = TWO_PI / 12;
        om = Math.min(om, C.speedUse * k.cal.vmax / steps, Math.sqrt(C.speedUse * k.cal.amax / steps));
        this.sweepT += dt;
        if (om * this.sweepT >= 2 * TWO_PI) { this.sweepT = 0; this.seg = (this.seg + 1) % 3; }
        const v = amp * Math.sin(om * this.sweepT);
        if (seg === 0) dev.h = v; else if (seg === 1) dev.p = v; else dev.r = v;
      }
      const center = { h: caps.heaveMid, p: 0, r: 0 };
      const f = k.fit(center, dev, C.fitMargin);
      return f.keep < 0 ? { pose: center, keep: -1, steps: k.poseToSteps(center) } : f;
    }
    get label() { return SEA.SEA_STATES[this.ss].name + ' - ' + SEA.SEA_STATES[this.ss].word; }
  }
  SEA.WaveMapper = WaveMapper;

  /* ---- theme colours read from CSS ------------------------------------------- */
  function css(name, el) { return getComputedStyle(el || document.documentElement).getPropertyValue(name).trim(); }
  SEA.palette = function () {
    return { paper: css('--paper'), grid: css('--grid'), gridStrong: css('--grid-strong'), ink: css('--ink'), mute: css('--mute'),
             heave: css('--heave'), pitch: css('--pitch'), roll: css('--roll'), accent: css('--accent'), water: css('--water'), surface: css('--surface') };
  };

  /* ---- strip chart ----------------------------------------------------------- */
  /* series: [{label, unit, color, range (+-), decimals}] ; push(values[]) at `hz` */
  class StripChart {
    constructor(canvas, series, opts) {
      this.c = canvas; this.s = series; this.o = Object.assign({ seconds: 30, hz: 30, readout: true, labelW: 0 }, opts || {});
      this.n = Math.round(this.o.seconds * this.o.hz);
      this.buf = series.map(() => new Float32Array(this.n)); this.fill = 0; this.head = 0;
      this.dpr = 1; this.resize();
      if (window.ResizeObserver) new ResizeObserver(() => { this.resize(); this.draw(); }).observe(canvas);
    }
    resize() {
      const r = this.c.getBoundingClientRect(); this.dpr = Math.min(2, window.devicePixelRatio || 1);
      this.W = Math.max(50, r.width); this.H = Math.max(40, r.height);
      this.c.width = Math.round(this.W * this.dpr); this.c.height = Math.round(this.H * this.dpr);
    }
    clear() { this.fill = 0; this.head = 0; }
    push(vals) {
      for (let i = 0; i < this.s.length; i++) this.buf[i][this.head] = vals[i];
      this.head = (this.head + 1) % this.n; if (this.fill < this.n) this.fill++;
    }
    draw() {
      const ctx = this.c.getContext('2d'), P = SEA.palette(), W = this.W, H = this.H, d = this.dpr;
      ctx.setTransform(d, 0, 0, d, 0, 0);
      ctx.fillStyle = P.paper; ctx.fillRect(0, 0, W, H);
      const rows = this.s.length, rowH = H / rows, right = this.o.readout ? 92 : 8, left = 8, plotW = W - left - right;
      ctx.font = '600 12px "Archivo Variable", system-ui, sans-serif';
      ctx.textBaseline = 'middle';
      /* vertical time grid: one line per 5 s, scrolling with the pen */
      const secPx = plotW / this.o.seconds;
      for (let t = 0; t <= this.o.seconds; t += 5) {
        const x = left + plotW - t * secPx;
        ctx.strokeStyle = t % 10 === 0 ? P.gridStrong : P.grid; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
      }
      for (let r = 0; r < rows; r++) {
        const s = this.s[r], top = r * rowH, mid = top + rowH / 2, amp = rowH * 0.42, rg = s.range || 1;
        ctx.strokeStyle = P.gridStrong; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(left, mid + 0.5); ctx.lineTo(left + plotW, mid + 0.5); ctx.stroke();
        if (r > 0) { ctx.strokeStyle = P.grid; ctx.beginPath(); ctx.moveTo(0, top + 0.5); ctx.lineTo(W, top + 0.5); ctx.stroke(); }
        ctx.fillStyle = P.mute; ctx.textAlign = 'left';
        ctx.fillText(s.label, left + 4, top + 12);
        /* trace */
        ctx.strokeStyle = s.color; ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.beginPath();
        let last = 0;
        for (let k = 0; k < this.fill; k++) {
          const idx = (this.head - this.fill + k + this.n * 2) % this.n, v = this.buf[r][idx];
          const x = left + plotW - (this.fill - 1 - k) * (plotW / (this.n - 1)), y = mid - Math.max(-1.15, Math.min(1.15, v / rg)) * amp;
          if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
          last = v;
        }
        ctx.stroke();
        if (this.o.readout && this.fill) {
          const y = mid - Math.max(-1.15, Math.min(1.15, last / rg)) * amp;
          ctx.fillStyle = s.color; ctx.beginPath(); ctx.arc(left + plotW, y, 3.5, 0, TWO_PI); ctx.fill();
          ctx.textAlign = 'right'; ctx.font = '700 17px "Archivo Variable", system-ui, sans-serif';
          ctx.fillStyle = P.ink;
          ctx.fillText((last >= 0 ? '' : '−') + Math.abs(last).toFixed(s.decimals === undefined ? 1 : s.decimals), W - 28, mid - 4);
          ctx.font = '500 11px "Archivo Variable", system-ui, sans-serif'; ctx.fillStyle = P.mute;
          ctx.fillText(s.unit, W - 10, mid + 12);
          ctx.font = '600 12px "Archivo Variable", system-ui, sans-serif';
        }
      }
    }
  }
  SEA.StripChart = StripChart;

  /* ---- 3D view of the platform ------------------------------------------------ */
  /* world: x forward (toward the Z leg), y left, z up; mm */
  class PlatformView {
    constructor(canvas, kin) {
      this.c = canvas; this.kin = kin; this.yaw = -35 * DEG; this.elev = 24 * DEG; this.zoom = 1;
      this.pose = null; this.ghost = null; this.dpr = 1; this.resize();
      let drag = null;
      canvas.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, yaw: this.yaw, elev: this.elev }; canvas.setPointerCapture(e.pointerId); });
      canvas.addEventListener('pointermove', (e) => {
        if (!drag) return;
        this.yaw = drag.yaw + (e.clientX - drag.x) * 0.008;
        this.elev = Math.max(-0.2, Math.min(1.45, drag.elev + (e.clientY - drag.y) * 0.006)); this.draw();
      });
      const end = () => { drag = null; };
      canvas.addEventListener('pointerup', end); canvas.addEventListener('pointercancel', end);
      canvas.addEventListener('wheel', (e) => { e.preventDefault(); this.zoom = Math.max(0.5, Math.min(2.5, this.zoom * (e.deltaY < 0 ? 1.08 : 0.92))); this.draw(); }, { passive: false });
      if (window.ResizeObserver) new ResizeObserver(() => { this.resize(); this.draw(); }).observe(canvas);
    }
    resize() {
      const r = this.c.getBoundingClientRect(); this.dpr = Math.min(2, window.devicePixelRatio || 1);
      this.W = Math.max(50, r.width); this.H = Math.max(50, r.height);
      this.c.width = Math.round(this.W * this.dpr); this.c.height = Math.round(this.H * this.dpr);
    }
    setPose(p) { this.pose = p; }
    proj(p) {
      /* rotate about z by yaw, tilt by elev, perspective */
      const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), ce = Math.cos(this.elev), se = Math.sin(this.elev);
      const mid = this.centerZ;
      const x = p[0] * cy - p[1] * sy, y = p[0] * sy + p[1] * cy, z = p[2] - mid;
      const depth = y * ce + z * se;                       /* toward the viewer is +y' after the tilt */
      const up = z * ce - y * se;
      const dist = 900 * 2.6, f = dist / (dist - depth);
      const scale = Math.min(this.W * 0.72, this.H * 0.86) / (this.R * 2.1) * this.zoom;
      return [this.W / 2 + x * f * scale, this.H * 0.5 - up * f * scale, depth];
    }
    draw() {
      const kin = this.kin, ctx = this.c.getContext('2d'), P = SEA.palette(), d = this.dpr;
      ctx.setTransform(d, 0, 0, d, 0, 0);
      ctx.fillStyle = P.paper; ctx.fillRect(0, 0, this.W, this.H);
      if (!kin || !kin.ready) return;
      const G = kin.G;
      this.R = Math.max(G.Rb, G.Rp) + G.r;
      this.centerZ = kin.z0 + kin.caps.heaveMid * 0.6;
      const pose = this.pose || { h: kin.caps.heaveMid, p: 0, r: 0 };
      const sol = kin.solve(pose);
      /* floor: base ring and a polar grid */
      ctx.lineWidth = 1;
      for (let rr = 1; rr <= 3; rr++) {
        ctx.strokeStyle = rr === 3 ? P.gridStrong : P.grid; ctx.beginPath();
        for (let a = 0; a <= 72; a++) { const q = this.proj([Math.cos(a * 5 * DEG) * this.R * rr / 2.4, Math.sin(a * 5 * DEG) * this.R * rr / 2.4, 0]); if (a) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]); }
        ctx.stroke();
      }
      ctx.strokeStyle = P.grid;
      for (let a = 0; a < 360; a += 30) { const q0 = this.proj([0, 0, 0]), q1 = this.proj([Math.cos(a * DEG) * this.R * 1.25, Math.sin(a * DEG) * this.R * 1.25, 0]); ctx.beginPath(); ctx.moveTo(q0[0], q0[1]); ctx.lineTo(q1[0], q1[1]); ctx.stroke(); }
      /* north marker on the floor: the front (Z leg) direction */
      { const a = this.proj([this.R * 1.25, 0, 0]), b = this.proj([this.R * 1.45, 0, 0]);
        ctx.strokeStyle = P.accent; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        ctx.fillStyle = P.accent; ctx.font = '700 12px "Archivo Variable", system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.fillText('front', b[0], b[1] - 8); }
      if (!sol) { ctx.fillStyle = P.mute; ctx.textAlign = 'center'; ctx.fillText('pose out of reach', this.W / 2, this.H / 2); return; }
      const names = ['X', 'Y', 'Z'], cols = [P.heave, P.pitch, P.roll];
      const parts = [];
      for (let i = 0; i < 3; i++) {
        const B = [kin.B[i][0], kin.B[i][1], 0];
        parts.push({ kind: 'crank', i, a: B, b: sol.C[i] });
        parts.push({ kind: 'link', i, a: sol.C[i], b: sol.P[i] });
      }
      parts.sort((u, v) => (this.proj(u.a)[2] + this.proj(u.b)[2]) - (this.proj(v.a)[2] + this.proj(v.b)[2]));
      /* motor posts */
      for (let i = 0; i < 3; i++) {
        const B0 = this.proj([kin.B[i][0], kin.B[i][1], 0]), B1 = this.proj([kin.B[i][0], kin.B[i][1], -G.r * 0.5]);
        ctx.strokeStyle = P.mute; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(B0[0], B0[1]); ctx.lineTo(B1[0], B1[1]); ctx.stroke();
      }
      const drawPlat = () => {
        const pts = [];
        for (let a = 0; a < 360; a += 6) {
          const x = G.Rp * Math.cos(a * DEG), y = G.Rp * Math.sin(a * DEG);
          const sp = Math.sin(pose.p * DEG), cp = Math.cos(pose.p * DEG), sr = Math.sin(pose.r * DEG), cr = Math.cos(pose.r * DEG);
          pts.push(this.proj([x * cp, y * cr - x * sp * sr, y * sr + x * sp * cr + kin.z0 + pose.h]));
        }
        ctx.beginPath(); pts.forEach((q, k) => (k ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]))); ctx.closePath();
        ctx.fillStyle = P.surface; ctx.globalAlpha = 0.78; ctx.fill(); ctx.globalAlpha = 1;
        ctx.strokeStyle = P.ink; ctx.lineWidth = 2; ctx.stroke();
        /* triangle through the three joints */
        ctx.beginPath(); sol.P.forEach((p, k) => { const q = this.proj(p); if (k) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]); }); ctx.closePath();
        ctx.strokeStyle = P.mute; ctx.lineWidth = 1; ctx.stroke();
      };
      /* draw links and cranks behind the deck first when we look from above, in front of it when from below */
      const above = this.elev > 0;
      if (!above) drawPlat();
      for (const part of parts) {
        const a = this.proj(part.a), b = this.proj(part.b);
        ctx.strokeStyle = part.kind === 'crank' ? cols[part.i] : P.ink; ctx.lineWidth = part.kind === 'crank' ? 5 : 2.2; ctx.lineCap = 'round';
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      }
      if (above) drawPlat();
      for (let i = 0; i < 3; i++) {
        const c = this.proj(sol.C[i]), j = this.proj(sol.P[i]), b = this.proj([kin.B[i][0], kin.B[i][1], 0]);
        ctx.fillStyle = P.paper; ctx.strokeStyle = P.ink; ctx.lineWidth = 2;
        for (const q of [c, j]) { ctx.beginPath(); ctx.arc(q[0], q[1], 4, 0, TWO_PI); ctx.fill(); ctx.stroke(); }
        ctx.fillStyle = cols[i]; ctx.beginPath(); ctx.arc(b[0], b[1], 7, 0, TWO_PI); ctx.fill();
        ctx.fillStyle = P.paper; ctx.font = '700 10px "Archivo Variable", system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(names[i], b[0], b[1] + 0.5);
      }
      ctx.textBaseline = 'alphabetic';
    }
  }
  SEA.PlatformView = PlatformView;

  /* ---- Web Serial link (Chrome / Edge) ------------------------------------------ */
  class SeaSerial {
    constructor() {
      this.port = null; this.reader = null; this.writer = null; this.onLine = null; this.onTelemetry = null; this.onState = null; this.onDrop = null;
      this.buf = ''; this.open = false;
      /* Everything typed at the board goes through one paced queue. The board reads its serial port between motor
         steps, so a burst of characters (a held key, 'qqqq', 'h12.5<Enter>') can overrun it and drop letters. */
      this.gapMs = 18;            /* pause between two characters */
      this.maxQueued = 40;        /* characters waiting; more than this and new keys are refused instead of piling up */
      this.q = []; this.pumping = false; this.lastWrite = 0;
    }
    static supported() { return 'serial' in navigator; }
    async connect(baud) {
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: baud || 115200, dataBits: 8, stopBits: 1, parity: 'none' });
      this.writer = this.port.writable.getWriter(); this.open = true; this.buf = ''; this.q = [];
      if (this.onState) this.onState(true);
      this.readLoop();
    }
    async readLoop() {
      const dec = new TextDecoder();
      try {
        while (this.port && this.port.readable) {
          this.reader = this.port.readable.getReader();
          try {
            for (;;) {
              const { value, done } = await this.reader.read();
              if (done) break;
              if (value) this.feed(dec.decode(value, { stream: true }));
            }
          } finally { this.reader.releaseLock(); }
        }
      } catch (e) { if (this.onLine) this.onLine('[serial] ' + e.message, true); }
      this.open = false; if (this.onState) this.onState(false);
    }
    feed(text) {
      this.buf += text;
      let i;
      while ((i = this.buf.search(/[\r\n]/)) >= 0) {
        const line = this.buf.slice(0, i); this.buf = this.buf.slice(i + 1);
        if (!line) continue;
        const t = SeaSerial.parseTelemetry(line);
        if (t) { if (this.onTelemetry) this.onTelemetry(t); } else if (this.onLine) this.onLine(line);
      }
      /* a prompt without a newline ("Select: ") is shown after a short quiet time */
      clearTimeout(this.flushTimer);
      this.flushTimer = setTimeout(() => { if (this.buf && this.onLine) { this.onLine(this.buf, false, true); this.buf = ''; } }, 120);
    }
    static parseTelemetry(line) {
      if (line.charCodeAt(0) !== 84 || line.charCodeAt(1) !== 44) return null;       /* "T," */
      const f = line.split(',');
      if (f.length < 10) return null;
      const n = f.slice(1).map(Number);
      if (n.some(Number.isNaN)) return null;
      return { ms: n[0], mode: n[1], h: n[2] / 10, p: n[3] / 100, r: n[4] / 100, steps: [n[5], n[6], n[7]], homed: !!(n[8] & 1), fkFailed: !!(n[8] & 4), locked: !!(n[8] & 8) };
    }
    /* Characters waiting to go out. */
    queued() { return this.q.reduce((n, it) => n + it.text.length - it.sent, 0); }
    /* send(text)            queue it, one character every gapMs
       send(text, {tag})     a waiting message with the same tag is replaced (sliders: only the latest position matters)
       send(text, {now})     STOP: forget everything waiting and write this immediately
       Resolves true when written, false when dropped. */
    send(text, opts) {
      opts = opts || {};
      if (!this.writer || !this.open || !text) return Promise.resolve(false);
      if (opts.now) {
        this.q.forEach((it) => it.res(false)); this.q = [];
        return this.rawWrite(text);
      }
      if (opts.tag) {
        const i = this.q.findIndex((it) => it.tag === opts.tag && it.sent === 0);
        if (i >= 0) { this.q[i].res(false); this.q.splice(i, 1); }
      }
      if (this.queued() + text.length > this.maxQueued) { if (this.onDrop) this.onDrop(text); return Promise.resolve(false); }
      return new Promise((res) => { this.q.push({ text, tag: opts.tag, sent: 0, res }); this.pump(); });
    }
    async rawWrite(text) {
      try { await this.writer.write(new TextEncoder().encode(text)); this.lastWrite = performance.now(); return true; }
      catch (e) { if (this.onLine) this.onLine('[serial] write failed: ' + e.message, true); return false; }
    }
    async pump() {
      if (this.pumping) return;
      this.pumping = true;
      try {
        while (this.q.length && this.open) {
          const it = this.q[0];
          while (it.sent < it.text.length) {
            const wait = this.lastWrite + this.gapMs - performance.now();
            if (wait > 0) await new Promise((r) => setTimeout(r, wait));
            if (this.q[0] !== it || !this.open) break;                 /* a STOP cleared the queue meanwhile */
            const ch = it.text[it.sent++];
            if (!(await this.rawWrite(ch))) { it.sent = it.text.length; break; }
          }
          if (this.q[0] === it) this.q.shift();
          it.res(it.sent >= it.text.length);
        }
      } finally { this.pumping = false; }
    }
    async disconnect() {
      try { if (this.reader) await this.reader.cancel(); } catch (e) {}
      try { if (this.writer) { this.writer.releaseLock(); } } catch (e) {}
      try { if (this.port) await this.port.close(); } catch (e) {}
      this.port = null; this.open = false; this.q.forEach((it) => it.res(false)); this.q = []; if (this.onState) this.onState(false);
    }
  }
  SEA.Serial = SeaSerial;
  SEA.MODE_NAMES = ['Menu', 'Setup', 'Heave cycle', 'Pitch / roll circles', 'Waves', 'MAX platform', 'Manual'];

  SEA.themeToggle = function (btn) {
    const apply = (t) => { if (t) document.documentElement.setAttribute('data-theme', t); else document.documentElement.removeAttribute('data-theme'); };
    let saved = null; try { saved = localStorage.getItem('sea-theme'); } catch (e) {}
    apply(saved);
    if (btn) btn.addEventListener('click', () => {
      const cur = document.documentElement.getAttribute('data-theme') || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
      const next = cur === 'dark' ? 'light' : 'dark'; apply(next);
      try { localStorage.setItem('sea-theme', next); } catch (e) {}
      document.dispatchEvent(new Event('sea-theme'));
    });
  };

  if (typeof module !== 'undefined' && module.exports) module.exports = SEA; else root.SEA = SEA;
})(typeof window !== 'undefined' ? window : globalThis);
