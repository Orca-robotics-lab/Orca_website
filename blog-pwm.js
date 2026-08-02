/* ========================================================================
   PWM — from the ground up  ·  Orca Robotics
   Six canvas-2D interactive scopes for the interactive blog.
   No dependencies. Each demo is a "scene" with a draw(t) fn; a single
   rAF loop drives all of them, gated by an IntersectionObserver so
   off-screen canvases don't burn cycles.
   ====================================================================== */
(() => {
'use strict';

const C = {
  orange: '#E87722',
  orangeSoft: 'rgba(232,119,34,0.14)',
  ink: '#141414',
  blue: '#2A6FDB',
  green: '#1F8A5B',
  gray: '#9c9c95',
  grid: '#ececE6',
  faint: '#b6b6ae',
  paper: '#fdfdfc'
};
const MONO = '"IBM Plex Mono", ui-monospace, monospace';
const DISP = '"Syne", system-ui, sans-serif';

/* ── canvas / scene plumbing ─────────────────────────────────────────── */
const scenes = [];
function makeScene(id) {
  const cv = document.getElementById(id);
  const ctx = cv.getContext('2d');
  const s = { cv, ctx, w: 0, h: 0, visible: true, lastT: 0 };
  const fit = () => {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    s.w = cv.clientWidth; s.h = cv.clientHeight;
    cv.width = Math.max(1, s.w * dpr); cv.height = Math.max(1, s.h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  };
  fit();
  if (window.ResizeObserver) new ResizeObserver(fit).observe(cv);
  if (window.IntersectionObserver)
    new IntersectionObserver(es => es.forEach(e => s.visible = e.isIntersecting), { rootMargin: '160px' }).observe(cv);
  scenes.push(s);
  return s;
}
function frame(t) {
  for (const s of scenes) {
    if (!s.visible || !s.draw || s.w < 2) continue;
    const dt = s.lastT ? Math.min(0.05, (t - s.lastT) / 1000) : 0;
    s.lastT = t;
    s.ctx.clearRect(0, 0, s.w, s.h);
    try { s.draw(t / 1000, dt); } catch (e) { /* keep loop alive */ }
  }
  requestAnimationFrame(frame);
}

function bind(id, cb) {
  const el = document.getElementById(id);
  const out = document.getElementById(id + '-val');
  if (!el) return;
  const f = () => cb(parseFloat(el.value), out);
  el.addEventListener('input', f);
  f();
}

/* ── drawing helpers ─────────────────────────────────────────────────── */
function rr(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}
function panel(ctx, R, label) {
  ctx.fillStyle = C.paper; rr(ctx, R.x, R.y, R.w, R.h, 9); ctx.fill();
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1; rr(ctx, R.x + .5, R.y + .5, R.w - 1, R.h - 1, 9); ctx.stroke();
  if (label) {
    ctx.fillStyle = C.gray; ctx.font = '11px ' + MONO; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
    ctx.fillText(label, R.x + 13, R.y + 19);
  }
}
function vy(R, v, pad) { pad = pad == null ? R.h * 0.16 : pad; return R.y + R.h - pad - v * (R.h - 2 * pad); }
function midline(ctx, R) {
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
  const y = vy(R, 0.5); ctx.beginPath(); ctx.moveTo(R.x + 8, y); ctx.lineTo(R.x + R.w - 8, y); ctx.stroke();
  ctx.setLineDash([]);
}
function txt(ctx, str, x, y, { font = '12px ' + MONO, color = C.gray, align = 'left', baseline = 'alphabetic' } = {}) {
  ctx.font = font; ctx.fillStyle = color; ctx.textAlign = align; ctx.textBaseline = baseline;
  ctx.fillText(str, x, y);
}
// stat chip
function chip(ctx, x, y, label, value, color) {
  ctx.font = '10px ' + MONO; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
  ctx.fillStyle = C.gray; ctx.fillText(label, x, y);
  ctx.font = '600 20px ' + DISP; ctx.fillStyle = color || C.ink;
  ctx.fillText(value, x, y + 24);
}

// square wave: duty 0..1, nP periods across R, phase in periods
function square(ctx, R, duty, nP, phase, color, { lw = 2.6, fill = null, hi = 0.14, lo = 0.86 } = {}) {
  const yH = R.y + R.h * hi, yL = R.y + R.h * lo;
  ctx.beginPath();
  let prev = null;
  const N = Math.ceil(R.w);
  for (let px = 0; px <= N; px++) {
    const u = px / R.w;
    const ph = u * nP + phase;
    const frac = ph - Math.floor(ph);
    const isHi = frac < duty;
    const X = R.x + px, Y = isHi ? yH : yL;
    if (prev === null) ctx.moveTo(X, Y);
    else if (prev !== isHi) { ctx.lineTo(X, prev ? yH : yL); ctx.lineTo(X, Y); }
    else ctx.lineTo(X, Y);
    prev = isHi;
  }
  if (fill) {
    ctx.save(); ctx.lineTo(R.x + R.w, yL); ctx.lineTo(R.x, yL); ctx.closePath();
    ctx.fillStyle = fill; ctx.fill(); ctx.restore();
    ctx.beginPath(); prev = null;
    for (let px = 0; px <= N; px++) {
      const u = px / R.w, ph = u * nP + phase, frac = ph - Math.floor(ph), isHi = frac < duty;
      const X = R.x + px, Y = isHi ? yH : yL;
      if (prev === null) ctx.moveTo(X, Y);
      else if (prev !== isHi) { ctx.lineTo(X, prev ? yH : yL); ctx.lineTo(X, Y); }
      else ctx.lineTo(X, Y);
      prev = isHi;
    }
  }
  ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.stroke();
}
// plot a v01 sample array across R
function plotArr(ctx, R, arr, color, { lw = 2.4, dash = null } = {}) {
  ctx.beginPath();
  const n = arr.length;
  for (let i = 0; i < n; i++) {
    const X = R.x + (i / (n - 1)) * R.w, Y = vy(R, arr[i]);
    i === 0 ? ctx.moveTo(X, Y) : ctx.lineTo(X, Y);
  }
  if (dash) ctx.setLineDash(dash);
  ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.lineJoin = 'round'; ctx.stroke();
  ctx.setLineDash([]);
}
function dashY(ctx, R, v, color, label) {
  const y = vy(R, v);
  ctx.setLineDash([5, 5]); ctx.strokeStyle = color; ctx.lineWidth = 1.4;
  ctx.beginPath(); ctx.moveTo(R.x + 8, y); ctx.lineTo(R.x + R.w - 8, y); ctx.stroke();
  ctx.setLineDash([]);
  if (label) txt(ctx, label, R.x + R.w - 10, y - 6, { align: 'right', color });
}

// LED with glow. bright 0..1
function led(ctx, cx, cy, r, bright) {
  const b = Math.max(0, Math.min(1, bright));
  const g = ctx.createRadialGradient(cx, cy, 0, cx, cy, r * 3);
  g.addColorStop(0, `rgba(255,150,50,${0.55 * b})`);
  g.addColorStop(1, 'rgba(255,150,50,0)');
  ctx.fillStyle = g; ctx.beginPath(); ctx.arc(cx, cy, r * 3, 0, 7); ctx.fill();
  const lerp = (a, z) => Math.round(a + (z - a) * b);
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, 7);
  ctx.fillStyle = `rgb(${lerp(44, 255)},${lerp(42, 176)},${lerp(38, 92)})`; ctx.fill();
  ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(0,0,0,0.18)'; ctx.stroke();
  // specular
  ctx.beginPath(); ctx.arc(cx - r * 0.3, cy - r * 0.32, r * 0.28, 0, 7);
  ctx.fillStyle = `rgba(255,255,255,${0.15 + 0.4 * b})`; ctx.fill();
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 1 — Duty cycle
   ════════════════════════════════════════════════════════════════════ */
function demoDuty() {
  const s = makeScene('pwm-duty');
  const p = { duty: 0.35 };
  bind('duty-slider', (v, out) => { p.duty = v / 100; if (out) out.textContent = Math.round(v) + '%'; });
  s.draw = (t) => {
    const { ctx, w, h } = s;
    const pad = 20;
    const wave = { x: pad, y: pad, w: w - pad * 2, h: h * 0.52 };
    panel(ctx, wave, 'PWM SIGNAL · 5 V logic');
    midline(ctx, wave);
    const phase = -t * 0.35;
    square(ctx, wave, p.duty, 4, phase, C.orange, { fill: C.orangeSoft });
    // average line
    const avgV = 0.14 + (0.86 - 0.14 - 0) * 0; // placeholder to keep hi/lo mapping
    const yAvg = wave.y + wave.h * 0.86 - p.duty * wave.h * (0.86 - 0.14);
    ctx.setLineDash([5, 5]); ctx.strokeStyle = C.ink; ctx.lineWidth = 1.6;
    ctx.beginPath(); ctx.moveTo(wave.x + 8, yAvg); ctx.lineTo(wave.x + wave.w - 8, yAvg); ctx.stroke();
    ctx.setLineDash([]);
    txt(ctx, 'average', wave.x + wave.w - 10, yAvg - 7, { align: 'right', color: C.ink });
    // labels HIGH/LOW time on first period
    const pw = wave.w / 4;
    ctx.fillStyle = 'rgba(232,119,34,0.10)';
    // on/off brackets
    txt(ctx, 'ON', wave.x + pw * p.duty / 2, wave.y + wave.h - 8, { align: 'center', color: C.orange, font: '10px ' + MONO });
    txt(ctx, 'OFF', wave.x + pw * p.duty + pw * (1 - p.duty) / 2, wave.y + wave.h - 8, { align: 'center', color: C.gray, font: '10px ' + MONO });

    // bottom: LED + readouts
    const by = wave.y + wave.h + 26;
    const cx = wave.x + 70, cy = by + 66;
    led(ctx, cx, cy, 30, p.duty);
    txt(ctx, 'LED brightness', cx, cy + 58, { align: 'center', color: C.gray, font: '10px ' + MONO });

    const Vavg = (p.duty * 5).toFixed(2);
    chip(ctx, wave.x + 150, by + 20, 'DUTY CYCLE', Math.round(p.duty * 100) + '%', C.orange);
    chip(ctx, wave.x + 150, by + 74, 'AVG VOLTAGE', Vavg + ' V', C.ink);
    chip(ctx, wave.x + 300, by + 20, 'ON TIME', (p.duty * 100).toFixed(0) + '%', C.gray);
    chip(ctx, wave.x + 300, by + 74, 'OFF TIME', ((1 - p.duty) * 100).toFixed(0) + '%', C.gray);

    // voltage bar
    const bar = { x: wave.x + wave.w - 46, y: by + 8, w: 26, h: 128 };
    ctx.strokeStyle = C.grid; rr(ctx, bar.x, bar.y, bar.w, bar.h, 5); ctx.stroke();
    const fillH = bar.h * p.duty;
    ctx.fillStyle = C.orange; rr(ctx, bar.x, bar.y + bar.h - fillH, bar.w, fillH, 5); ctx.fill();
    txt(ctx, '5V', bar.x + bar.w / 2, bar.y - 6, { align: 'center', color: C.gray, font: '9px ' + MONO });
    txt(ctx, '0V', bar.x + bar.w / 2, bar.y + bar.h + 13, { align: 'center', color: C.gray, font: '9px ' + MONO });
  };
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 2 — Frequency & period (real-time flicker)
   ════════════════════════════════════════════════════════════════════ */
function demoFreq() {
  const s = makeScene('pwm-freq');
  const p = { f: 4, duty: 0.5 };
  bind('freq-slider', (v, out) => { p.f = Math.round(v); if (out) out.textContent = p.f + ' Hz'; });
  s.draw = (t) => {
    const { ctx, w, h } = s;
    const pad = 20;
    const win = 1.0; // seconds shown
    const wave = { x: pad, y: pad, w: w - pad * 2, h: h * 0.5 };
    panel(ctx, wave, `PWM SIGNAL · ${(win * 1000).toFixed(0)} ms window`);
    midline(ctx, wave);
    const nP = p.f * win;
    square(ctx, wave, p.duty, nP, -t * 0.3, C.orange, { fill: C.orangeSoft });
    // period bracket on first cycle (if not too dense)
    if (nP <= 24) {
      const pw = wave.w / nP;
      const yb = wave.y + 14;
      ctx.strokeStyle = C.blue; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.moveTo(wave.x + 4, yb); ctx.lineTo(wave.x + 4 + pw, yb); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(wave.x + 4, yb - 4); ctx.lineTo(wave.x + 4, yb + 4);
      ctx.moveTo(wave.x + 4 + pw, yb - 4); ctx.lineTo(wave.x + 4 + pw, yb + 4); ctx.stroke();
      txt(ctx, 'T = ' + (1000 / p.f).toFixed(1) + ' ms', wave.x + 4 + pw + 8, yb + 4, { color: C.blue, font: '10px ' + MONO });
    }

    // LED — real-time flicker below ~45 Hz, fused glow above
    const by = wave.y + wave.h + 30;
    const cx = wave.x + 70, cy = by + 60;
    let bright;
    if (p.f < 45) {
      const ph = (t * p.f) % 1;
      const on = ph < p.duty;
      // small blend near threshold for smoothness
      bright = on ? 1 : 0.05;
      if (p.f > 30) { const k = (p.f - 30) / 15; bright = bright * (1 - k) + p.duty * k; }
    } else {
      bright = p.duty;
    }
    led(ctx, cx, cy, 32, bright);
    txt(ctx, p.f < 24 ? 'you can SEE the blink' : p.f < 45 ? 'blur — almost fused' : 'fused: steady glow',
      cx, cy + 60, { align: 'center', color: p.f < 24 ? C.orange : C.gray, font: '10px ' + MONO });

    chip(ctx, wave.x + 165, by + 14, 'FREQUENCY', p.f + ' Hz', C.blue);
    chip(ctx, wave.x + 165, by + 66, 'PERIOD', (1000 / p.f).toFixed(1) + ' ms', C.ink);
    chip(ctx, wave.x + 320, by + 14, 'CYCLES / SEC', p.f + '', C.gray);
    chip(ctx, wave.x + 320, by + 66, 'DUTY', '50 %', C.gray);
  };
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 3 — Digital → analog (RC smoothing)
   ════════════════════════════════════════════════════════════════════ */
function demoFilter() {
  const s = makeScene('pwm-filter');
  const p = { duty: 0.6, tau: 0.35 }; // tau: smoothing 0..1
  bind('filt-duty', (v, out) => { p.duty = v / 100; if (out) out.textContent = Math.round(v) + '%'; });
  bind('filt-tau', (v, out) => { p.tau = v / 100; if (out) out.textContent = v < 33 ? 'light' : v < 66 ? 'medium' : 'heavy'; });
  s.draw = (t) => {
    const { ctx, w, h } = s;
    const pad = 20;
    const R = { x: pad, y: pad, w: w - pad * 2, h: h * 0.62 };
    panel(ctx, R, 'PWM IN  →  RC LOW-PASS FILTER  →  SMOOTH DC OUT');
    midline(ctx, R);
    const nP = 8, phase = -t * 0.4;
    // raw pwm as sample array (v01)
    const N = Math.max(200, Math.round(R.w));
    const raw = new Array(N), filt = new Array(N);
    for (let i = 0; i < N; i++) {
      const u = i / (N - 1), ph = u * nP + phase, frac = ph - Math.floor(ph);
      raw[i] = frac < p.duty ? 1 : 0;
    }
    // RC integration; alpha from tau (heavier tau => smaller alpha)
    const alpha = 0.5 * (1 - p.tau) * (1 - p.tau) + 0.012;
    // seed near duty for stable look
    let y = p.duty;
    // a few warmup passes wrapping around for periodicity
    for (let pass = 0; pass < 3; pass++) for (let i = 0; i < N; i++) y += (raw[i] - y) * alpha;
    for (let i = 0; i < N; i++) { y += (raw[i] - y) * alpha; filt[i] = y; }

    square(ctx, R, p.duty, nP, phase, 'rgba(232,119,34,0.5)', { fill: 'rgba(232,119,34,0.07)', lw: 1.6 });
    dashY(ctx, R, p.duty, C.ink, 'target ' + (p.duty * 5).toFixed(2) + ' V');
    plotArr(ctx, R, filt, C.green, { lw: 3 });

    // ripple
    let mn = 9, mx = -9; for (const v of filt) { if (v < mn) mn = v; if (v > mx) mx = v; }
    const ripple = (mx - mn) * 5;

    const by = R.y + R.h + 30;
    chip(ctx, R.x + 4, by + 8, 'OUTPUT (DC)', (p.duty * 5).toFixed(2) + ' V', C.green);
    chip(ctx, R.x + 170, by + 8, 'RIPPLE', ripple.toFixed(2) + ' V', ripple > 0.4 ? C.orange : C.gray);
    chip(ctx, R.x + 320, by + 8, 'DUTY', Math.round(p.duty * 100) + '%', C.ink);
    txt(ctx, 'Heavier smoothing → less ripple, slower response. The average is the analog value.',
      R.x + 4, by + 66, { color: C.gray, font: '11px ' + MONO });
  };
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 4 — Counter + compare + resolution
   ════════════════════════════════════════════════════════════════════ */
function demoCounter() {
  const s = makeScene('pwm-counter');
  const p = { req: 0.5, bits: 8 };
  bind('cnt-compare', (v, out) => { p.req = v / 1000; });
  bind('cnt-bits', (v, out) => { p.bits = Math.round(v); if (out) out.textContent = p.bits + '-bit'; });
  s.draw = (t) => {
    const { ctx, w, h } = s;
    const pad = 20;
    const TOP = Math.pow(2, p.bits) - 1;
    const levels = TOP + 1;
    const compareInt = Math.round(p.req * TOP);
    const dutyQ = compareInt / TOP;

    const R = { x: pad, y: pad, w: w - pad * 2, h: h * 0.42 };
    panel(ctx, R, 'TIMER COUNTER (ramp)  vs  COMPARE VALUE');
    const nP = 4, phase = -t * 0.3;
    // ramp
    ctx.beginPath();
    const N = Math.ceil(R.w);
    for (let px = 0; px <= N; px++) {
      const u = px / R.w, ph = u * nP + phase, frac = ph - Math.floor(ph);
      const X = R.x + px, Y = vy(R, frac, R.h * 0.14);
      px === 0 ? ctx.moveTo(X, Y) : ctx.lineTo(X, Y);
    }
    ctx.strokeStyle = C.blue; ctx.lineWidth = 2; ctx.stroke();
    // compare line
    dashY(ctx, R, dutyQ, C.orange, 'compare = ' + compareInt);
    txt(ctx, 'TOP = ' + TOP, R.x + R.w - 10, vy(R, 1, R.h * 0.14) + 4, { align: 'right', color: C.blue, font: '10px ' + MONO });

    // output panel
    const O = { x: pad, y: R.y + R.h + 16, w: R.w, h: h * 0.24 };
    panel(ctx, O, 'OUTPUT · high while counter < compare');
    square(ctx, O, dutyQ, nP, phase, C.orange, { fill: C.orangeSoft });

    // readouts
    const by = O.y + O.h + 26;
    chip(ctx, R.x + 4, by + 6, 'RESOLUTION', p.bits + '-bit', C.blue);
    chip(ctx, R.x + 150, by + 6, 'STEPS', levels.toLocaleString(), C.ink);
    chip(ctx, R.x + 300, by + 6, 'DUTY', (dutyQ * 100).toFixed(2) + '%', C.orange);
    const stepPct = (100 / TOP).toFixed(3);
    txt(ctx, `smallest duty step = 1/${TOP} = ${stepPct}%  ·  requested ${(p.req * 100).toFixed(1)}% → nearest ${(dutyQ * 100).toFixed(2)}%`,
      R.x + 4, by + 60, { color: C.gray, font: '11px ' + MONO });
  };
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 5 — Servo control
   ════════════════════════════════════════════════════════════════════ */
function demoServo() {
  const s = makeScene('pwm-servo');
  const p = { pw: 1500, ang: 0 };
  bind('servo-pw', (v, out) => { p.pw = Math.round(v); if (out) out.textContent = p.pw + ' µs'; });
  s.draw = (t, dt) => {
    const { ctx, w, h } = s;
    const pad = 20;
    const target = Math.max(-135, Math.min(135, (p.pw - 1500) / 500 * 90));
    p.ang += (target - p.ang) * Math.min(1, dt * 9);

    // pulse train panel (top)
    const R = { x: pad, y: pad, w: w - pad * 2, h: h * 0.34 };
    panel(ctx, R, 'CONTROL PULSE · 50 Hz (20 ms frame)');
    midline(ctx, R);
    // one 20ms frame across width; pulse width fraction of 20ms
    const frac = p.pw / 20000; // 20ms = 20000us
    const nP = 2, phase = -t * 0.25;
    square(ctx, R, frac, nP, phase, C.orange, { fill: C.orangeSoft });
    txt(ctx, p.pw + ' µs pulse', R.x + R.w - 12, R.y + R.h - 10, { align: 'right', color: C.orange, font: '10px ' + MONO });
    // scale markers
    ['1.0', '1.5', '2.0'].forEach((lab, i) => {
      const us = [1000, 1500, 2000][i];
      txt(ctx, lab + 'ms', R.x + (us / 20000) / nP * R.w + 3, R.y + 32, { color: C.faint, font: '9px ' + MONO });
    });

    // servo drawing (bottom)
    const O = { x: pad, y: R.y + R.h + 18, w: R.w, h: h - R.h - pad - 20 };
    const cx = O.x + O.w * 0.5, cy = O.y + O.h * 0.74;
    // body
    ctx.fillStyle = '#20232a'; rr(ctx, cx - 46, cy - 8, 92, 62, 6); ctx.fill();
    ctx.fillStyle = '#2b2f38'; rr(ctx, cx - 46, cy - 8, 92, 14, 6); ctx.fill();
    // hub
    ctx.beginPath(); ctx.arc(cx, cy, 15, 0, 7); ctx.fillStyle = '#3a3f49'; ctx.fill();
    // angle arc guide
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(cx, cy, 96, Math.PI + 0.3, 2 * Math.PI - 0.3); ctx.stroke();
    // horn
    const a = (-90 + p.ang) * Math.PI / 180;
    const hx = cx + Math.cos(a) * 92, hy = cy + Math.sin(a) * 92;
    ctx.strokeStyle = C.orange; ctx.lineWidth = 8; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy); ctx.stroke();
    ctx.beginPath(); ctx.arc(hx, hy, 7, 0, 7); ctx.fillStyle = C.orange; ctx.fill();
    ctx.beginPath(); ctx.arc(cx, cy, 5, 0, 7); ctx.fillStyle = '#fff'; ctx.fill();
    // tick labels
    txt(ctx, '−90°', cx - 96, cy + 6, { align: 'right', color: C.faint, font: '10px ' + MONO });
    txt(ctx, '+90°', cx + 96, cy + 6, { align: 'left', color: C.faint, font: '10px ' + MONO });
    txt(ctx, '0°', cx, cy - 104, { align: 'center', color: C.faint, font: '10px ' + MONO });

    chip(ctx, O.x + 4, O.y + 24, 'PULSE WIDTH', p.pw + ' µs', C.orange);
    chip(ctx, O.x + 4, O.y + 78, 'ANGLE', (Math.round(p.ang)) + '°', C.ink);
  };
}

/* ════════════════════════════════════════════════════════════════════
   DEMO 6 — Sinusoidal PWM (advanced)
   ════════════════════════════════════════════════════════════════════ */
function demoSPWM() {
  const s = makeScene('pwm-spwm');
  const p = { m: 0.8, ratio: 15 };
  bind('spwm-m', (v, out) => { p.m = v / 100; if (out) out.textContent = p.m.toFixed(2); });
  bind('spwm-ratio', (v, out) => { p.ratio = Math.round(v); if (out) out.textContent = p.ratio + '×'; });
  s.draw = (t) => {
    const { ctx, w, h } = s;
    const pad = 20, gap = 12;
    const ph = t * 0.25;
    const nRef = 1.5;                 // reference periods across window
    const carrier = nRef * p.ratio;   // triangle periods across window
    const N = Math.max(240, Math.round(w));

    const panelH = (h - pad * 2 - gap * 2) / 3;
    const A = { x: pad, y: pad, w: w - pad * 2, h: panelH };
    const B = { x: pad, y: A.y + panelH + gap, w: A.w, h: panelH };
    const D = { x: pad, y: B.y + panelH + gap, w: A.w, h: panelH };

    // reference (bipolar mapped to 0..1) and triangle carrier
    const ref = new Array(N), tri = new Array(N), out = new Array(N), filt = new Array(N);
    for (let i = 0; i < N; i++) {
      const u = i / (N - 1);
      const r = p.m * Math.sin(2 * Math.PI * (u * nRef) - ph * 2 * Math.PI * 0);     // sine bipolar
      const rr2 = p.m * Math.sin(2 * Math.PI * (u * nRef) + ph);
      let cph = (u * carrier + ph * 3) % 1; if (cph < 0) cph += 1;
      const tr = (cph < 0.5 ? (cph * 4 - 1) : (3 - cph * 4)); // triangle -1..1
      ref[i] = rr2; tri[i] = tr;
      out[i] = rr2 > tr ? 1 : -1;
    }
    // filtered
    let y = 0; const alpha = 0.06;
    for (let pass = 0; pass < 2; pass++) for (let i = 0; i < N; i++) y += (out[i] - y) * alpha;
    for (let i = 0; i < N; i++) { y += (out[i] - y) * alpha; filt[i] = y; }

    // Panel A: reference + carrier
    panel(ctx, A, 'SINE REFERENCE  vs  TRIANGLE CARRIER');
    midline(ctx, A);
    plotArr(ctx, A, tri.map(v => 0.5 + 0.46 * v), C.gray, { lw: 1.4 });
    plotArr(ctx, A, ref.map(v => 0.5 + 0.46 * v), C.blue, { lw: 2.6 });

    // Panel B: comparator output
    panel(ctx, B, 'COMPARATOR OUTPUT · sine > carrier ?');
    midline(ctx, B);
    // draw as steps
    (() => {
      ctx.beginPath();
      const yH = vy(B, 0.94), yL = vy(B, 0.06);
      let prev = null;
      for (let i = 0; i < N; i++) {
        const X = B.x + (i / (N - 1)) * B.w, hi = out[i] > 0, Y = hi ? yH : yL;
        if (prev === null) ctx.moveTo(X, Y);
        else if (prev !== hi) { ctx.lineTo(X, prev ? yH : yL); ctx.lineTo(X, Y); }
        else ctx.lineTo(X, Y);
        prev = hi;
      }
      ctx.strokeStyle = C.orange; ctx.lineWidth = 1.8; ctx.lineJoin = 'round'; ctx.stroke();
    })();

    // Panel D: filtered result vs reference
    panel(ctx, D, 'FILTERED OUTPUT ≈ RECONSTRUCTED SINE');
    midline(ctx, D);
    plotArr(ctx, D, ref.map(v => 0.5 + 0.46 * v), C.blue, { lw: 1.6, dash: [4, 4] });
    plotArr(ctx, D, filt.map(v => 0.5 + 0.46 * v), C.orange, { lw: 3 });
    txt(ctx, 'M = ' + p.m.toFixed(2) + '  ·  carrier ' + p.ratio + '× ref', D.x + D.w - 12, D.y + 19, { align: 'right', color: C.gray, font: '10px ' + MONO });
  };
}

/* ── boot ─────────────────────────────────────────────────────────────── */
function boot() {
  [demoDuty, demoFreq, demoFilter, demoCounter, demoServo, demoSPWM].forEach(f => { try { f(); } catch (e) { console.warn(e); } });
  requestAnimationFrame(frame);
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
else boot();
})();
