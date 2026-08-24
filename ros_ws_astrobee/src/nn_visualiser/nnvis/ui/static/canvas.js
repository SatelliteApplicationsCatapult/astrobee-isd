/* NN visualiser canvas renderer.
 *
 * Owns the playhead.  Python ships the whole sequence once; nothing here
 * talks back to the server during playback, so frame timing is independent
 * of websocket latency and of whatever Python happens to be doing.
 */
(function () {
  'use strict';

  const S = {
    meta: null, frames: null, weights: null,
    playing: false, t: 0, rate: 1.0, last: 0,
    layout: null, gaps: null, ctx: null, dpr: 1,
  };
  window.nnvis = S;

  // ---- colour ----------------------------------------------------------
  function hex(h) {
    return [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
  }
  function mix(a, b, t) {
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
  }
  function rgba(c, a) {
    return 'rgba(' + (c[0] | 0) + ',' + (c[1] | 0) + ',' + (c[2] | 0) + ',' + a + ')';
  }
  let GREY, ELEC, AMBER, GRIP, TEXT, TEXTB;

  // grey -> electric blue.  For ReLU activations, which are never negative.
  function uni(v) {
    const t = Math.min(1, Math.max(0, v));
    return mix(GREY, ELEC, Math.pow(t, 0.65));
  }
  // amber <- grey -> electric blue.  For signed channels: inputs and F/T.
  function div(v) {
    const t = Math.min(1, Math.abs(v));
    const e = Math.pow(t, 0.65);
    return v < 0 ? mix(GREY, AMBER, e) : mix(GREY, ELEC, e);
  }

  // ---- layout ----------------------------------------------------------
  function buildLayout(m) {
    const top = m.col_top, bot = m.col_bottom;
    const cols = m.sizes.map(function (n, k) {
      const ys = new Float32Array(n);
      // inputs and outputs get breathing room; hidden layers fill the column
      const pad = n <= 16 ? (bot - top) * 0.06 : 0;
      const a = top + pad, b = bot - pad;
      for (let i = 0; i < n; i++) ys[i] = a + (b - a) * (n === 1 ? 0.5 : i / (n - 1));
      return { n: n, x: m.col_x[k], ys: ys, r: n <= 16 ? 7.5 : (n <= 80 ? 3.4 : 2.4) };
    });
    // the "actual" strip mirrors the output column
    const actual = { n: m.sizes[3], x: m.col_x[4], ys: cols[3].ys, r: 7.5 };

    // ribbon blocks + precomputed weight block sums
    const gaps = [];
    let woff = 0;
    for (let k = 0; k < m.sizes.length - 1; k++) {
      const nA = m.sizes[k], nB = m.sizes[k + 1];
      const gA = m.ribbon_groups[k], gB = m.ribbon_groups[k + 1];
      const W = new Float32Array(S.weights.buffer, woff * 4, nA * nB);
      woff += nA * nB;

      const bandsA = bands(nA, gA), bandsB = bands(nB, gB);
      // Wsum[i * gB + B] = sum of W[i][j] for j in block B  (and abs version)
      const Wsum = new Float32Array(nA * gB), Wabs = new Float32Array(nA * gB);
      for (let i = 0; i < nA; i++) {
        for (let B = 0; B < gB; B++) {
          let s = 0, sa = 0;
          for (let j = bandsB[B][0]; j < bandsB[B][1]; j++) {
            const w = W[i * nB + j];
            s += w; sa += Math.abs(w);
          }
          Wsum[i * gB + B] = s; Wabs[i * gB + B] = sa;
        }
      }
      gaps.push({
        k: k, nA: nA, nB: nB, W: W, gA: gA, gB: gB,
        bandsA: bandsA, bandsB: bandsB, Wsum: Wsum, Wabs: Wabs,
        // Ribbons are composited additively, so gA*gB of them overlapping in
        // the same space saturate to a slab.  Normalising by sqrt(gA*gB)
        // against the 16x8 gap as reference keeps ink density roughly equal
        // across gaps instead of making it a side effect of how many groups a
        // layer happens to have.
        alphaK: Math.sqrt(16 * 8) / Math.sqrt(gA * gB),
        contrib: new Float32Array(nA * nB),
        blkAbs: new Float32Array(gA * gB), blkSgn: new Float32Array(gA * gB),
      });
    }
    return { cols: cols, actual: actual, gaps: gaps };
  }

  function bands(n, g) {
    const out = [];
    for (let b = 0; b < g; b++) {
      out.push([Math.floor(b * n / g), Math.floor((b + 1) * n / g)]);
    }
    return out;
  }

  function bandY(col, band) {
    const ys = col.ys;
    const a = ys[band[0]], b = ys[Math.max(band[0], band[1] - 1)];
    const half = Math.max(3, (b - a) / 2 + col.r * 1.6);
    const c = (a + b) / 2;
    return [c - half, c + half];
  }

  // ---- per-frame maths -------------------------------------------------
  function computeGap(g, actA) {
    // ribbons: |a_i * W_ij| summed over a block factorises as
    // |a_i| * sum_j |W_ij|, so the block sums are 128x8 not 128x64.
    g.blkAbs.fill(0); g.blkSgn.fill(0);
    for (let A = 0; A < g.gA; A++) {
      const band = g.bandsA[A];
      for (let i = band[0]; i < band[1]; i++) {
        const a = actA[i];
        if (a === 0) continue;
        const aa = Math.abs(a);
        for (let B = 0; B < g.gB; B++) {
          g.blkSgn[A * g.gB + B] += a * g.Wsum[i * g.gB + B];
          g.blkAbs[A * g.gB + B] += aa * g.Wabs[i * g.gB + B];
        }
      }
    }
    // individual contributions, for the bright overlay
    let mx = 0;
    for (let i = 0; i < g.nA; i++) {
      const a = actA[i];
      const base = i * g.nB;
      if (a === 0) { g.contrib.fill(0, base, base + g.nB); continue; }
      for (let j = 0; j < g.nB; j++) {
        const c = a * g.W[base + j];
        g.contrib[base + j] = c;
        const ac = c < 0 ? -c : c;
        if (ac > mx) mx = ac;
      }
    }
    g.max = mx;
  }

  // ---- draw ------------------------------------------------------------
  function draw() {
    const m = S.meta, L = S.layout, ctx = S.ctx;
    const idx = Math.min(m.n_frames - 1, Math.max(0, Math.round(S.t * m.fps)));
    const off = idx * m.stride;
    const F = S.frames;

    const acts = [
      F.subarray(off + m.offsets.in, off + m.offsets.in + m.sizes[0]),
      F.subarray(off + m.offsets.h1, off + m.offsets.h1 + m.sizes[1]),
      F.subarray(off + m.offsets.h2, off + m.offsets.h2 + m.sizes[2]),
      F.subarray(off + m.offsets.pred, off + m.offsets.pred + m.sizes[3]),
    ];
    const actual = F.subarray(off + m.offsets.act, off + m.offsets.act + m.sizes[3]);
    const raw = F.subarray(off + m.offsets.raw, off + m.offsets.raw + m.sizes[0]);

    ctx.fillStyle = m.palette.bg;
    ctx.fillRect(0, 0, m.canvas[0], m.canvas[1]);

    ctx.globalCompositeOperation = 'lighter';

    for (let gi = 0; gi < L.gaps.length; gi++) {
      const g = L.gaps[gi];
      const colA = L.cols[gi], colB = L.cols[gi + 1];
      // for the input gap the activation is signed; use |a| for the ribbons
      computeGap(g, acts[gi]);

      // --- ribbon substrate ---
      let bmax = 0;
      for (let i = 0; i < g.blkAbs.length; i++) if (g.blkAbs[i] > bmax) bmax = g.blkAbs[i];
      if (bmax > 0) {
        for (let A = 0; A < g.gA; A++) {
          const ya = bandY(colA, g.bandsA[A]);
          for (let B = 0; B < g.gB; B++) {
            const v = g.blkAbs[A * g.gB + B] / bmax;
            if (v < 0.02) continue;
            const yb = bandY(colB, g.bandsB[B]);
            const sgn = g.blkSgn[A * g.gB + B];
            const c = sgn < 0 ? AMBER : ELEC;
            ctx.fillStyle = rgba(mix(GREY, c, Math.pow(v, 0.8)),
                                 (0.026 + 0.055 * v * v) * g.alphaK);
            ctx.beginPath();
            ctx.moveTo(colA.x, ya[0]);
            ctx.bezierCurveTo((colA.x + colB.x) / 2, ya[0], (colA.x + colB.x) / 2, yb[0], colB.x, yb[0]);
            ctx.lineTo(colB.x, yb[1]);
            ctx.bezierCurveTo((colA.x + colB.x) / 2, yb[1], (colA.x + colB.x) / 2, ya[1], colA.x, ya[1]);
            ctx.closePath();
            ctx.fill();
          }
        }
      }

      // --- bright individual edges ---
      // True top-K within the gap: a fixed threshold made the wide gap and
      // the narrow one look like different drawings.  A cheap pre-filter
      // keeps the candidate list small so the sort stays trivial.
      if (g.max > 0) {
        const want = m.edge_counts[gi];
        const pre = g.max * 0.05;
        const picks = [];
        for (let i = 0; i < g.nA; i++) {
          const base = i * g.nB;
          for (let j = 0; j < g.nB; j++) {
            const c = g.contrib[base + j];
            const ac = c < 0 ? -c : c;
            if (ac >= pre) picks.push([ac, c, i, j]);
          }
        }
        picks.sort(function (p, q) { return q[0] - p[0]; });
        if (picks.length > want) picks.length = want;
        const mid = (colA.x + colB.x) / 2;
        for (let p = 0; p < picks.length; p++) {
          const ac = picks[p][0], c = picks[p][1], i = picks[p][2], j = picks[p][3];
          const v = Math.pow(ac / g.max, 0.7);
          const col = c < 0 ? AMBER : ELEC;
          ctx.strokeStyle = rgba(mix(GREY, col, 0.30 + 0.70 * v), 0.12 + 0.55 * v);
          ctx.lineWidth = 0.45 + 1.55 * v;
          ctx.beginPath();
          const y0 = colA.ys[i], y1 = colB.ys[j];
          ctx.moveTo(colA.x, y0);
          ctx.bezierCurveTo(mid, y0, mid, y1, colB.x, y1);
          ctx.stroke();
        }
      }
    }

    ctx.globalCompositeOperation = 'source-over';

    // --- neurons ---
    for (let k = 0; k < L.cols.length; k++) {
      const col = L.cols[k], a = acts[k], sc = m.scales[k];
      for (let i = 0; i < col.n; i++) {
        let c, v;
        if (k === 0) { v = a[i] / sc; c = div(v); }
        else if (k === 3) {
          if (i === col.n - 1) { v = a[i]; c = mix(GREY, GRIP, Math.min(1, v)); }
          else { v = a[i] / m.out_scales[i]; c = div(v); }
        } else { v = a[i] / sc; c = uni(v); }
        neuron(ctx, col.x, col.ys[i], col.r, c, Math.min(1, Math.abs(v)));
      }
    }
    // actual strip
    for (let i = 0; i < L.actual.n; i++) {
      let c, v;
      if (i === L.actual.n - 1) { v = actual[i]; c = mix(GREY, GRIP, Math.min(1, v)); }
      else { v = actual[i] / m.out_scales[i]; c = div(v); }
      neuron(ctx, L.actual.x, L.actual.ys[i], L.actual.r, c, Math.min(1, Math.abs(v)));
    }

    labels(ctx, m, L, raw, acts, actual, idx);
  }

  function neuron(ctx, x, y, r, c, v) {
    if (v > 0.08) {
      const gr = ctx.createRadialGradient(x, y, 0, x, y, r * 4.2);
      gr.addColorStop(0, rgba(c, 0.42 * v));
      gr.addColorStop(1, rgba(c, 0));
      ctx.fillStyle = gr;
      ctx.beginPath(); ctx.arc(x, y, r * 4.2, 0, 6.2832); ctx.fill();
    }
    ctx.fillStyle = rgba(c, 0.95);
    ctx.beginPath(); ctx.arc(x, y, r, 0, 6.2832); ctx.fill();
  }

  function labels(ctx, m, L, raw, acts, actual, idx) {
    ctx.textBaseline = 'middle';
    const MONO = 'ui-monospace, Menlo, monospace';

    // Observation column.  The scale caption is not decoration: the colour is
    // (value - mean) / 3 sigma, so it answers "how unusual is this for this
    // channel", while the number answers "what is it in metres".  Without the
    // sigma on screen those two read as the same quantity and they are not.
    for (let i = 0; i < m.sizes[0]; i++) {
      const y = L.cols[0].ys[i];
      const x = L.cols[0].x - 16;
      ctx.textAlign = 'right';
      ctx.font = '400 9px ' + MONO;
      ctx.fillStyle = rgba(TEXT, 0.5);
      const cap = m.in_caption[i];
      ctx.fillText(cap, x, y - 8);
      ctx.font = '600 11px ' + MONO;
      ctx.fillStyle = rgba(TEXTB, 0.92);
      ctx.fillText(m.in_labels[i], x - ctx.measureText(cap).width - 26, y - 8);
      ctx.font = '400 10px ' + MONO;
      ctx.fillStyle = rgba(TEXT, 0.85);
      ctx.fillText(raw[i].toFixed(m.in_dp[i]), x, y + 7);
    }

    // Action columns.  Force full-scale is 0.8 N and torque 0.05 Nm, so the
    // same printed number is ~6x brighter on a torque channel than a force
    // one.  The +/- caption is the denominator that explains it.
    const mx = (L.cols[3].x + L.actual.x) / 2;
    ctx.textAlign = 'center';
    for (let i = 0; i < m.sizes[3]; i++) {
      const y = L.cols[3].ys[i];
      ctx.font = '600 11px ' + MONO;
      ctx.fillStyle = rgba(TEXTB, 0.92);
      ctx.fillText(m.out_labels[i], mx, y - 26);
      ctx.font = '400 9px ' + MONO;
      ctx.fillStyle = rgba(TEXT, 0.5);
      ctx.fillText(m.out_caption[i], mx, y - 13);
      ctx.font = '400 10px ' + MONO;
      ctx.fillStyle = rgba(TEXT, 0.85);
      ctx.fillText(acts[3][i].toFixed(m.out_dp[i]), L.cols[3].x, y + 20);
      ctx.fillText(actual[i].toFixed(m.out_dp[i]), L.actual.x, y + 20);
    }

    // column headers
    const hd = ['OBSERVATION', 'HIDDEN ' + m.sizes[1], 'HIDDEN ' + m.sizes[2],
                'PREDICTED', 'ACTUAL'];
    for (let k = 0; k < 5; k++) {
      ctx.textAlign = 'center';
      ctx.font = '600 11px ' + MONO;
      ctx.fillStyle = rgba(TEXT, 0.7);
      ctx.fillText(hd[k], m.col_x[k], m.col_top - 56);
      if (k === 1 || k === 2) {
        ctx.font = '400 9px ' + MONO;
        ctx.fillStyle = rgba(TEXT, 0.45);
        ctx.fillText(m.hidden_caption[k - 1], m.col_x[k], m.col_top - 40);
      }
    }
    ctx.textAlign = 'center';
    ctx.font = '400 9px ' + MONO;
    ctx.fillStyle = rgba(TEXT, 0.45);
    ctx.fillText('\u00b13\u03c3 (standardised)', m.col_x[0], m.col_top - 40);

    // clock
    ctx.textAlign = 'left';
    ctx.font = '600 13px ' + MONO;
    ctx.fillStyle = rgba(TEXTB, 0.9);
    ctx.fillText((idx / m.fps).toFixed(2) + ' s  /  ' + m.duration.toFixed(2) + ' s', 24, 28);
    ctx.font = '400 11px ' + MONO;
    ctx.fillStyle = rgba(TEXT, 0.65);
    ctx.fillText('frame ' + idx + ' / ' + (m.n_frames - 1) + '   x' + S.rate.toFixed(2)
                 + '   ' + (S.fps || 0).toFixed(0) + ' fps', 24, 48);

    // gripper state banner
    const g = actual[m.sizes[3] - 1] > 0.5;
    ctx.textAlign = 'right';
    ctx.font = '600 12px ' + MONO;
    ctx.fillStyle = g ? rgba(GRIP, 0.95) : rgba(TEXT, 0.5);
    ctx.fillText(g ? 'GRIPPER CLOSED' : 'GRIPPER OPEN', m.canvas[0] - 24, 28);
  }

  function tick(now) {
    if (!S.meta) { requestAnimationFrame(tick); return; }
    // No clamp on dt.  The old Math.min(0.1, ...) silently discarded any time
    // beyond 100 ms, so below 10 fps the playhead advanced at 0.1 * render_fps
    // -- 0.5x at 5 fps -- while the clock readout claimed 1x.  The clamp only
    // existed to stop a leap after a backgrounded tab, which the
    // visibilitychange reset below handles properly.
    const dt = S.last ? (now - S.last) / 1000 : 0;
    S.last = now;
    S.fps = S.fps ? S.fps * 0.9 + (dt > 0 ? 0.1 / dt : 0) : (dt > 0 ? 1 / dt : 0);
    if (S.playing) {
      S.t += dt * S.rate;
      if (S.t >= S.meta.duration) S.t = 0;
      const sl = document.getElementById('nnvis-scrub');
      if (sl) sl.value = String(S.t / S.meta.duration * 1000);
    }
    draw();
    requestAnimationFrame(tick);
  }

  // ---- wiring ----------------------------------------------------------
  S.load = async function () {
    const meta = await (await fetch('/nnvis/meta?v=' + Date.now())).json();
    const fb = await (await fetch('/nnvis/frames?v=' + Date.now())).arrayBuffer();
    const wb = await (await fetch('/nnvis/weights?v=' + Date.now())).arrayBuffer();
    S.meta = meta;
    S.frames = new Float32Array(fb);
    S.weights = new Float32Array(wb);
    GREY = hex(meta.palette.grey); ELEC = hex(meta.palette.electric);
    AMBER = hex(meta.palette.amber); GRIP = hex(meta.palette.gripper_on);
    TEXT = hex(meta.palette.text); TEXTB = hex(meta.palette.text_bright);
    S.layout = buildLayout(meta);
    S.t = 0; S.playing = true;
    const c = document.getElementById('nnvis-canvas');
    S.dpr = window.devicePixelRatio || 1;
    c.width = meta.canvas[0] * S.dpr; c.height = meta.canvas[1] * S.dpr;
    c.style.width = meta.canvas[0] + 'px'; c.style.height = meta.canvas[1] + 'px';
    S.ctx = c.getContext('2d');
    S.ctx.setTransform(S.dpr, 0, 0, S.dpr, 0, 0);
    const btn = document.getElementById('nnvis-play');
    if (btn) btn.textContent = '❚❚';
  };

  S.draw = draw;          // exposed for headless testing

  S.toggle = function () {
    S.playing = !S.playing;
    const b = document.getElementById('nnvis-play');
    if (b) b.textContent = S.playing ? '❚❚' : '▶';
  };
  S.seek = function (frac) {
    if (!S.meta) return;
    S.t = frac * S.meta.duration;
  };
  S.setRate = function (r) { S.rate = r; };
  S.step = function (n) {
    if (!S.meta) return;
    S.playing = false;
    const b = document.getElementById('nnvis-play');
    if (b) b.textContent = '▶';
    S.t = Math.min(S.meta.duration, Math.max(0, S.t + n / S.meta.fps));
    const sl = document.getElementById('nnvis-scrub');
    if (sl) sl.value = String(S.t / S.meta.duration * 1000);
  };

  // Backgrounded tabs stop firing requestAnimationFrame.  Zeroing S.last means
  // the first frame back has dt = 0, so the playhead does not leap forward --
  // this is what the removed dt clamp was actually for.
  document.addEventListener('visibilitychange', function () { S.last = 0; });

  requestAnimationFrame(tick);
})();
