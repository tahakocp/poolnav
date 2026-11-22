const STATES = ['driving', 'pushing wall', 'backing off', 'turning', 'unjamming', 'done'];
const $ = (id) => document.getElementById(id);

async function load() {
  const bytes = await (await fetch('poolnav.wasm')).arrayBuffer();
  const make = async () => (await WebAssembly.instantiate(bytes, {})).instance.exports;
  return Promise.all([make(), make()]);
}

class View {
  constructor(canvas, ex, label) {
    this.cv = canvas; this.ctx = canvas.getContext('2d'); this.ex = ex; this.label = label;
    this.trail = document.createElement('canvas');
    this.t90 = null; this.t95 = null;
  }

  start(pool, variant, seed, bias, err) {
    const e = this.ex;
    e.pn_new(pool, variant, seed, bias, err, 45);
    const polys = [];
    for (let i = 0; i < e.pn_poly_count(); i++) {
      polys.push(new Float64Array(e.memory.buffer, e.pn_poly_ptr(i), e.pn_poly_len(i)).slice());
    }
    this.polys = polys;
    let minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (let i = 0; i < polys[0].length; i += 2) {
      minx = Math.min(minx, polys[0][i]); maxx = Math.max(maxx, polys[0][i]);
      miny = Math.min(miny, polys[0][i + 1]); maxy = Math.max(maxy, polys[0][i + 1]);
    }
    const pad = 14, W = this.cv.width, H = this.cv.height;
    this.k = Math.min((W - 2 * pad) / (maxx - minx), (H - 2 * pad) / (maxy - miny));
    this.ox = (W - this.k * (maxx - minx)) / 2 - this.k * minx;
    this.oy = (H + this.k * (maxy - miny)) / 2 + this.k * miny;
    this.trail.width = W; this.trail.height = H;
    this.tctx = this.trail.getContext('2d');
    this.tctx.strokeStyle = 'rgba(90,150,255,.55)'; this.tctx.lineWidth = 1;
    this.last = null; this.t90 = null; this.t95 = null;
  }

  px(x, y) { return [this.ox + x * this.k, this.oy - y * this.k]; }

  step(n) {
    const e = this.ex;
    for (let i = 0; i < n; i += 5) {
      e.pn_step(Math.min(5, n - i));
      const p = this.px(e.pn_x(), e.pn_y());
      if (this.last) { this.tctx.beginPath(); this.tctx.moveTo(...this.last); this.tctx.lineTo(...p); this.tctx.stroke(); }
      this.last = p;
      const c = e.pn_coverage(), t = e.pn_time();
      if (this.t90 === null && c >= 0.9) this.t90 = t;
      if (this.t95 === null && c >= 0.95) this.t95 = t;
    }
  }

  draw() {
    const e = this.ex, ctx = this.ctx;
    ctx.clearRect(0, 0, this.cv.width, this.cv.height);
    const gw = e.pn_grid_w(), gh = e.pn_grid_h(), cell = e.pn_cell();
    const grid = new Uint8Array(e.memory.buffer, e.pn_grid_ptr(), gw * gh);
    const ox = e.pn_origin_x(), oy = e.pn_origin_y();
    const cs = cell * this.k + 0.6;
    for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) {
      const g = grid[j * gw + i];
      if (!g) continue;
      ctx.fillStyle = g === 2 ? '#2f9488' : '#26323b';
      const [x, y] = this.px(ox + i * cell, oy + (j + 1) * cell);
      ctx.fillRect(x, y, cs, cs);
    }
    ctx.drawImage(this.trail, 0, 0);
    ctx.strokeStyle = '#8fb1c4'; ctx.lineWidth = 2; ctx.fillStyle = '#0b0f12';
    this.polys.forEach((poly, n) => {
      ctx.beginPath();
      for (let i = 0; i < poly.length; i += 2) {
        const [x, y] = this.px(poly[i], poly[i + 1]);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      ctx.closePath();
      if (n > 0) ctx.fill();
      ctx.stroke();
    });
    const [rx, ry] = this.px(e.pn_x(), e.pn_y());
    const th = e.pn_heading(), L = e.pn_body_len() * this.k, Wd = e.pn_body_wid() * this.k;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-th);
    ctx.fillStyle = '#e8eef2'; ctx.fillRect(-L / 2, -Wd / 2, L, Wd);
    ctx.fillStyle = '#f0883e'; ctx.fillRect(L / 2 - 3, -Wd / 2, 3, Wd);
    ctx.restore();
  }

  stats() {
    const e = this.ex, mm = (t) => (t === null ? '-' : (t / 60).toFixed(1) + ' min');
    const t = e.pn_time();
    const m = Math.floor(t / 60), s = String(Math.floor(t % 60)).padStart(2, '0');
    let txt = `${m}:${s}  ${(e.pn_coverage() * 100).toFixed(1)}%  90%: ${mm(this.t90)}  95%: ${mm(this.t95)}  ${STATES[e.pn_state()]}`;
    if (this.label === 'lanes' && !e.pn_lanes_ok()) txt += '  (walls not square, lanes dropped)';
    return txt;
  }
}

(async () => {
  const [a, b] = await load();
  const left = new View($('c0'), a, 'random'), right = new View($('c1'), b, 'lanes');
  let paused = false;

  const fmt = () => {
    $('o-bias').textContent = $('bias').value + ' deg/s';
    $('o-err').textContent = $('err').value + '%';
    $('o-speed').textContent = $('speed').value + 'x';
  };

  function restart() {
    const pool = $('pool').selectedIndex, seed = +$('seed').value;
    const bias = +$('bias').value, err = +$('err').value / 100;
    left.start(pool, 0, seed, bias, err);
    right.start(pool, +$('variant').value, seed, bias, err);
    $('name1').textContent = $('variant').selectedOptions[0].textContent;
    fmt();
  }

  ['pool', 'variant', 'seed', 'bias', 'err'].forEach((id) => $(id).addEventListener('change', restart));
  ['bias', 'err', 'speed'].forEach((id) => $(id).addEventListener('input', fmt));
  $('restart').onclick = restart;
  $('pause').onclick = () => { paused = !paused; $('pause').textContent = paused ? 'Resume' : 'Pause'; };

  const q = new URLSearchParams(location.search);
  if (q.get('pool')) $('pool').selectedIndex = Math.max(0, ['rect', 'l-shape', 'steps', 'kidney'].indexOf(q.get('pool')));
  if (q.get('seed')) $('seed').value = q.get('seed');
  restart();

  // ?at=SECONDS jumps both robots to that moment and stops, for stills.
  if (q.get('at')) {
    left.step(+q.get('at') * 10); right.step(+q.get('at') * 10);
    paused = true;
  }
  if (q.get('clean')) document.body.classList.add('clean');

  function frame() {
    if (!paused) {
      // 10 ticks are one second of robot time.
      const n = +$('speed').value * 10;
      left.step(n); right.step(n);
    }
    left.draw(); right.draw();
    $('s0').textContent = left.stats(); $('s1').textContent = right.stats();
    requestAnimationFrame(frame);
  }
  frame();
})();
