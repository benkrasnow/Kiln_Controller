// Dependency-free canvas chart: actual temperature and setpoint on the left
// axis, controller power (0-100 %) as a shaded area on the right axis.
// Colours come from CSS custom properties so the chart follows the theme.

const PAD = { left: 52, right: 44, top: 14, bottom: 28 };
export const TEMP_STEPS = [1, 2, 5, 10, 20, 25, 50, 100, 200, 250, 500];
export const TIME_STEPS_MIN = [1, 2, 5, 10, 15, 30, 60, 120, 180, 360, 720, 1440];

export class Chart {
  #canvas;
  #ctx;
  #tip;
  #samples = [];
  #rangeMs = Infinity;
  #hoverX = null;
  #geom = null;

  constructor(canvas, tooltip) {
    this.#canvas = canvas;
    this.#ctx = canvas.getContext("2d");
    this.#tip = tooltip;
    new ResizeObserver(() => this.draw()).observe(canvas);
    canvas.addEventListener("pointermove", (e) => {
      this.#hoverX = e.offsetX;
      this.draw();
    });
    canvas.addEventListener("pointerleave", () => {
      this.#hoverX = null;
      this.draw();
    });
    matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => this.draw());
  }

  // samples: [{t, pv, sp, duty}] in time order. rangeMs: visible window.
  setData(samples, rangeMs) {
    this.#samples = samples;
    this.#rangeMs = rangeMs;
    this.draw();
  }

  draw() {
    const c = this.#canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = c.clientWidth;
    const h = c.clientHeight;
    if (!w || !h) return;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
    }
    const ctx = this.#ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const css = getComputedStyle(c);
    const color = (name) => css.getPropertyValue(name).trim();
    const font = `12px ${css.fontFamily}`;
    ctx.font = font;

    const all = this.#samples;
    if (all.length < 2) {
      ctx.fillStyle = color("--muted");
      ctx.textAlign = "center";
      ctx.fillText("Waiting for data…", w / 2, h / 2);
      this.#tip.hidden = true;
      return;
    }

    const tEnd = all[all.length - 1].t;
    // Show at most rangeMs, but don't pad a short history with empty space.
    const tStart = Number.isFinite(this.#rangeMs) ? Math.max(all[0].t, tEnd - this.#rangeMs) : all[0].t;
    const vis = all.filter((s) => s.t >= tStart);
    const data = downsample(vis, Math.max(200, Math.floor(w)));

    let lo = Infinity;
    let hi = -Infinity;
    for (const s of data) {
      for (const v of [s.pv, s.sp]) {
        if (v == null) continue;
        lo = Math.min(lo, v);
        hi = Math.max(hi, v);
      }
    }
    if (!Number.isFinite(lo)) [lo, hi] = [0, 100];
    const span = Math.max(hi - lo, 20);
    const step = pickStep(span / 5, TEMP_STEPS);
    const yMin = Math.max(0, Math.floor((lo - span * 0.05) / step) * step);
    const yMax = Math.ceil((hi + span * 0.05) / step) * step;

    const x0 = PAD.left;
    const x1 = w - PAD.right;
    const y0 = h - PAD.bottom;
    const y1 = PAD.top;
    const tSpan = Math.max(tEnd - tStart, 1000);
    const X = (t) => x0 + ((t - tStart) / tSpan) * (x1 - x0);
    const Y = (v) => y0 - ((v - yMin) / (yMax - yMin)) * (y0 - y1);
    const YD = (d) => y0 - d * (y0 - y1);
    this.#geom = { X, data };

    // Grid and axis labels.
    ctx.strokeStyle = color("--grid");
    ctx.fillStyle = color("--muted");
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let v = yMin; v <= yMax + 1e-9; v += step) {
      const y = Math.round(Y(v)) + 0.5;
      line(ctx, x0, y, x1, y);
      ctx.fillText(`${v}°`, x0 - 8, y);
    }
    ctx.textAlign = "left";
    for (const d of [0, 0.5, 1]) ctx.fillText(`${d * 100}%`, x1 + 8, YD(d));

    const maxTimeTicks = Math.max(2, Math.floor((x1 - x0) / 90));  // ~90 px per label
    const tStep = pickStep(tSpan / 60000 / maxTimeTicks, TIME_STEPS_MIN) * 60000;
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    const offset = new Date(tStart).getTimezoneOffset() * 60000;
    for (let t = Math.ceil((tStart - offset) / tStep) * tStep + offset; t <= tEnd; t += tStep) {
      const x = Math.round(X(t)) + 0.5;
      line(ctx, x, y1, x, y0);
      ctx.fillText(clock(t), x, y0 + 8);
    }

    // Power area.
    ctx.beginPath();
    ctx.moveTo(X(data[0].t), y0);
    for (const s of data) ctx.lineTo(X(s.t), YD(s.duty ?? 0));
    ctx.lineTo(X(data[data.length - 1].t), y0);
    ctx.closePath();
    ctx.fillStyle = color("--power-fill");
    ctx.fill();

    // Setpoint (stepped, dashed) and actual temperature.
    ctx.lineJoin = "round";
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = color("--sp");
    ctx.lineWidth = 1.5;
    path(ctx, data, X, Y, "sp", true);
    ctx.setLineDash([]);
    ctx.strokeStyle = color("--pv");
    ctx.lineWidth = 2.25;
    path(ctx, data, X, Y, "pv", false);

    this.#drawHover(ctx, color, X, Y, y0, y1, data, w);
  }

  #drawHover(ctx, color, X, Y, y0, y1, data, w) {
    if (this.#hoverX == null || this.#hoverX < PAD.left || this.#hoverX > w - PAD.right) {
      this.#tip.hidden = true;
      return;
    }
    let best = data[0];
    for (const s of data) if (Math.abs(X(s.t) - this.#hoverX) < Math.abs(X(best.t) - this.#hoverX)) best = s;
    const x = Math.round(X(best.t)) + 0.5;
    ctx.strokeStyle = color("--muted");
    ctx.lineWidth = 1;
    line(ctx, x, y1, x, y0);
    for (const [key, name] of [["sp", "--sp"], ["pv", "--pv"]]) {
      if (best[key] == null) continue;
      ctx.fillStyle = color(name);
      ctx.beginPath();
      ctx.arc(x, Y(best[key]), 4, 0, Math.PI * 2);
      ctx.fill();
    }
    const fmt = (v) => (v == null ? "–" : `${v.toFixed(0)}°C`);
    this.#tip.innerHTML =
      `<b>${clock(best.t, true)}</b>` +
      `<span><i style="background:var(--pv)"></i>Actual ${fmt(best.pv)}</span>` +
      `<span><i style="background:var(--sp)"></i>Set ${fmt(best.sp)}</span>` +
      `<span><i style="background:var(--power)"></i>Power ${Math.round((best.duty ?? 0) * 100)}%</span>`;
    this.#tip.hidden = false;
    const tipW = this.#tip.offsetWidth;
    this.#tip.style.left = `${x + 12 + tipW > w ? x - 12 - tipW : x + 12}px`;
  }
}

export function line(ctx, ax, ay, bx, by) {
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.stroke();
}

function path(ctx, data, X, Y, key, stepped) {
  ctx.beginPath();
  let started = false;
  let prev = null;
  for (const s of data) {
    const v = s[key];
    if (v == null) {
      started = false;
      continue;
    }
    if (!started) {
      ctx.moveTo(X(s.t), Y(v));
      started = true;
    } else {
      if (stepped) ctx.lineTo(X(s.t), Y(prev));
      ctx.lineTo(X(s.t), Y(v));
    }
    prev = v;
  }
  ctx.stroke();
}

export function pickStep(raw, steps) {
  return steps.find((s) => s >= raw) ?? steps[steps.length - 1];
}

function downsample(samples, maxPoints) {
  if (samples.length <= maxPoints) return samples;
  const stride = Math.ceil(samples.length / maxPoints);
  const out = [];
  for (let i = 0; i < samples.length; i += stride) out.push(samples[i]);
  if (out[out.length - 1] !== samples[samples.length - 1]) out.push(samples[samples.length - 1]);
  return out;
}

function clock(t, seconds = false) {
  return new Date(t).toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    ...(seconds ? { second: "2-digit" } : {}),
  });
}
