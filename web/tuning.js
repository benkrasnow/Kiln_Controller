// PID tuning card: the controller's tuning table (baseline gains, plus gains
// and holding power at a few temperatures, interpolated by setpoint) and step
// tests that measure new points. Gains are shown in kiln-friendly units:
//   P  % power per °C of error          (kp * 100)
//   I  % power per °C·min of error      (ki * 100 * 60)
//   D  % power per °C/min of change     (kd * 100 / 60)

import { TEMP_STEPS, line, pickStep } from "./chart.js";

const STORAGE_KEY = "kiln.tuning.v2";  // {points, base}
const OLD_STORAGE_KEY = "kiln.tuning.v1";  // points only
const MAX_POINTS = 4;
const MAX_C = 1400;
export const DEFAULT_GAINS = { kp: 0.033, ki: 0.00055, kd: 0.27 };  // lib/kiln_core/src/pid.h

// Display <-> stored conversions.
const UNITS = {
  kp: { toUi: (v) => v * 100, fromUi: (v) => v / 100, digits: 2 },
  ki: { toUi: (v) => v * 6000, fromUi: (v) => v / 6000, digits: 3 },
  kd: { toUi: (v) => (v * 100) / 60, fromUi: (v) => (v * 60) / 100, digits: 2 },
  hold: { toUi: (v) => v * 100, fromUi: (v) => v / 100, digits: 1 },
};
const fmt = (key, v) => UNITS[key].toUi(v).toFixed(UNITS[key].digits);

// StepTestConfig in lib/kiln_core/src/tuning.h; the status line sends the
// live values once the test reaches each phase.
const TEST = { band: 5, need: 300, maxDrift: 5, minStep: 300, maxStep: 3600, maxRise: 50 };
const PHASE_ORDER = ["heating", "settling", "stepping", "done"];

function sorted(points) {
  return [...points].sort((a, b) => a.temp - b.temp);
}

const close = (x, y) => Math.abs(x - y) <= 1e-6 + 1e-4 * Math.abs(y);

function samePoints(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  return a.every((p, i) => ["temp", "kp", "ki", "kd", "hold"].every((k) => close(p[k], b[i][k])));
}

function sameGains(a, b) {
  return !!a && !!b && ["kp", "ki", "kd"].every((k) => close(a[k], b[k]));
}

const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
const minutes = (s) => (s < 90 ? "about a minute" : `about ${Math.round(s / 60)} minutes`);

// Same interpolation as lib/kiln_core/src/tuning.cpp, for the preview chart.
// `base`: the baseline gains, used with no points.
export function gainsAt(points, t, base = DEFAULT_GAINS) {
  if (!points.length) return base;
  if (t <= points[0].temp) return points[0];
  const last = points[points.length - 1];
  if (t >= last.temp) return last;
  let i = 1;
  while (points[i].temp < t) i++;
  const a = points[i - 1], b = points[i], f = (t - a.temp) / (b.temp - a.temp);
  return { kp: a.kp + (b.kp - a.kp) * f, ki: a.ki + (b.ki - a.ki) * f, kd: a.kd + (b.kd - a.kd) * f };
}

export function feedforwardAt(points, t, ambient = 20) {
  if (!points.length || t <= ambient) return 0;
  const first = points[0];
  if (t <= first.temp) return first.temp > ambient ? (first.hold * (t - ambient)) / (first.temp - ambient) : 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    if (t <= b.temp) return a.hold + ((b.hold - a.hold) * (t - a.temp)) / (b.temp - a.temp);
  }
  const b = points[points.length - 1];
  const a = points.length > 1 ? points[points.length - 2] : { temp: ambient, hold: 0 };
  if (b.temp - a.temp < 1) return Math.min(1, Math.max(0, b.hold));
  return Math.min(1, Math.max(0, b.hold + ((b.hold - a.hold) * (t - b.temp)) / (b.temp - a.temp)));
}

function validGains(g) {
  return !!g && ["kp", "ki", "kd"].every((k) => typeof g[k] === "number" && g[k] >= 0);
}

function load() {
  const empty = { points: [], base: { ...DEFAULT_GAINS } };
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (saved && Array.isArray(saved.points)) {
      return { points: sorted(saved.points), base: validGains(saved.base) ? saved.base : empty.base };
    }
    const old = JSON.parse(localStorage.getItem(OLD_STORAGE_KEY));
    if (Array.isArray(old)) return { ...empty, points: sorted(old) };
  } catch {}
  return empty;
}

function save(points, base) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({ points, base }));
  } catch {}
}

// actions: {request(cmd, params) -> Promise, toast(text, isError)}
export function initTuning(actions) {
  const $ = (id) => document.getElementById(id);
  const rows = $("tun-rows");
  const canvas = $("tun-chart");
  let { points, base } = load();
  let device = {
    connected: false, status: null, points: null, base: null, defaultBase: null, result: null, busy: false,
  };
  let heatStart = null;  // temperature when the heat-up was first seen, for its progress bar
  let history = [];  // [{t (ms), pv}] over the last minute of this test phase, for rate estimates

  function changed({ rebuild = false } = {}) {
    save(points, base);
    if (rebuild) {
      buildRows();
      buildBase();
    }
    render();
  }

  // Baseline gains: used when there are no tuning points.
  const baseInputs = ["kp", "ki", "kd"].map((k) => $(`tun-base-${k}`));
  function buildBase() {
    for (const input of baseInputs) {
      input.value = fmt(input.dataset.key, base[input.dataset.key]);
      input.classList.remove("bad");
    }
  }
  for (const input of baseInputs) {
    input.addEventListener("input", () => {
      const v = Number(input.value);
      const ok = input.value !== "" && v >= 0;
      input.classList.toggle("bad", !ok);
      if (!ok) return;
      base = { ...base, [input.dataset.key]: UNITS[input.dataset.key].fromUi(v) };
      changed();
    });
    input.addEventListener("change", () => changed({ rebuild: true }));
  }
  $("tun-base-reset").addEventListener("click", () => {
    base = { ...(device.defaultBase ?? DEFAULT_GAINS) };
    changed({ rebuild: true });
  });

  function buildRows() {
    rows.replaceChildren();
    points.forEach((p, i) => {
      const tr = document.createElement("tr");
      const cell = (key, value, step) =>
        `<td><input class="num" data-key="${key}" type="number" step="${step}" min="0" value="${value}"
                    aria-label="${key} of point ${i + 1}"></td>`;
      tr.innerHTML =
        cell("temp", Math.round(p.temp), 1) +
        cell("kp", fmt("kp", p.kp), "0.01") +
        cell("ki", fmt("ki", p.ki), "0.001") +
        cell("kd", fmt("kd", p.kd), "0.01") +
        cell("hold", fmt("hold", p.hold), "0.1") +
        `<td><button class="icon-btn" aria-label="Delete point ${i + 1}" title="Delete">×</button></td>`;
      for (const input of tr.querySelectorAll("input")) {
        input.addEventListener("input", () => {
          const key = input.dataset.key;
          const v = Number(input.value);
          const ok = input.value !== "" && v >= 0 && (key !== "temp" || v <= MAX_C) && (key !== "hold" || v <= 100);
          input.classList.toggle("bad", !ok);
          if (!ok) return;
          p[key] = key === "temp" ? v : UNITS[key].fromUi(v);
          changed();
        });
        input.addEventListener("change", () => {
          points = sorted(points);
          changed({ rebuild: true });
        });
      }
      tr.querySelector(".icon-btn").addEventListener("click", () => {
        points.splice(i, 1);
        changed({ rebuild: true });
      });
      rows.append(tr);
    });
    $("tun-add").disabled = points.length >= MAX_POINTS;
  }

  function validationError() {
    for (let i = 1; i < points.length; i++) {
      if (points[i].temp - points[i - 1].temp < 1) return "Two points have the same temperature.";
    }
    return null;
  }

  function render() {
    const st = device.status;
    const state = st?.state;
    const testing = state === "tune";
    const err = validationError();
    // Firmware before 0.7 has no baseline gains: compare points only.
    const match = samePoints(points, device.points) && (!device.base || sameGains(base, device.base));
    const defaults = device.defaultBase ?? DEFAULT_GAINS;

    $("tun-base-note").textContent =
      (points.length
        ? "Not used while there are tuning points: the points' gains apply at every temperature."
        : "In use: with no tuning points, the controller uses these gains at every temperature, including for a step test's heat-up and hold.") +
      (sameGains(base, defaults) ? " These are the built-in defaults." : "");
    $("tun-base-reset").disabled = sameGains(base, defaults);

    $("tun-summary").textContent = err ??
      (points.length === 0
        ? "No points: the controller uses the baseline gains and no holding-power feed-forward."
        : points.length === 1
          ? "One point: its gains apply at every temperature, and holding power scales from ambient."
          : "Gains are interpolated between points by setpoint; holding power is interpolated, and extended above the last point.");

    const info = $("tun-device");
    if (!device.connected) {
      info.textContent = "Connect to the controller to load or send tuning.";
    } else if (device.points) {
      info.innerHTML = "";
      const baseText = !device.base ? "" : sameGains(device.base, defaults) ? ", default baseline gains" : ", custom baseline gains";
      info.append(`On controller: ${device.points.length ? device.points.length + " point(s)" : "no points"}${baseText} `);
      const tag = document.createElement("span");
      tag.className = `tag ${match ? "ok" : "diff"}`;
      tag.textContent = match ? "same as editor" : "differs from editor";
      info.append(tag);
    } else {
      info.textContent = "";
    }
    $("tun-send").disabled = !device.connected || device.busy || !!err || match;
    $("tun-load").disabled = !device.connected || device.busy || !device.points || match;

    // Step test controls and live status.
    const busyElsewhere = state === "program";
    $("tun-start").disabled = !device.connected || device.busy || testing || busyElsewhere || !!st?.fault;
    $("tun-start").textContent = testing ? "Step test running…" : "Run step test";
    const live = $("tun-live");
    live.hidden = !testing;
    $("tun-plan").hidden = testing;
    $("tun-plan").textContent = planText();
    if (testing && st.tune) renderLive(st);
    $("tun-hint-busy").hidden = !busyElsewhere;

    const r = device.result;
    $("tun-result").hidden = !r;
    if (r) {
      $("tun-r-temp").textContent = Math.round(r.temp);
      $("tun-r-hold").textContent = `${fmt("hold", r.hold)} %`;
      $("tun-r-rate").textContent = `${(r.rate * 60).toFixed(1)} °C/min`;
      $("tun-r-dead").textContent = `${Math.round(r.dead)} s`;
      $("tun-r-gains").textContent = `P ${fmt("kp", r.kp)} · I ${fmt("ki", r.ki)} · D ${fmt("kd", r.kd)}`;
    }
    draw();
  }

  function planText() {
    const temp = Number($("tun-temp").value) || 150;
    return `How it works: the controller heats to ${temp} °C and holds it until the kiln has stayed within ` +
      `±${TEST.band} °C, moving less than ${TEST.maxDrift} °C, for ${TEST.need / 60} minutes. That window's ` +
      "average power is the holding power. Then it switches the control loop off, adds the power step and " +
      `records how fast the kiln responds (${TEST.minStep / 60}–${TEST.maxStep / 60} minutes). Heating stops at ` +
      "the end and the suggested gains appear here; nothing changes on the controller until you send them.";
  }

  // Temperature change in °C/s over the last minute of status lines, or null.
  function rate() {
    if (history.length < 2) return null;
    const a = history[0], b = history[history.length - 1];
    const span = (b.t - a.t) / 1000;
    return span >= 20 ? (b.pv - a.pv) / span : null;
  }

  function renderLive(st) {
    const t = st.tune;
    const live = $("tun-live");
    const pv = typeof st.pv === "number" ? st.pv : null;
    const r = rate();
    const perMin = r === null ? "" : `, ${r >= 0 ? "rising" : "falling"} ${Math.abs(r * 60).toFixed(1)} °C/min`;
    const band = t.band ?? TEST.band, need = t.need ?? TEST.need, maxDrift = t.max_drift ?? TEST.maxDrift;
    const stepRange = `${TEST.minStep / 60}–${(t.max_s ?? TEST.maxStep) / 60} min`;
    let title, detail, next, progress, left;

    if (t.phase === "heating") {
      const goal = t.temp - band;
      const toGo = pv === null ? null : Math.max(0, goal - pv);
      const eta = toGo !== null && r > 0.01 ? toGo / r : null;
      const start = heatStart ?? pv ?? 20;
      title = `Step 1 of 3 · Heating to ${t.temp} °C`;
      detail = pv === null ? "No temperature reading." :
        `Kiln ${pv.toFixed(0)} °C, ${toGo.toFixed(0)} °C to go${perMin}` + (eta !== null ? ` · ${minutes(eta)} to reach ${goal} °C` : "");
      next = `Next: holding ${t.temp} °C until it has been steady for ${need / 60} minutes.`;
      progress = pv === null || goal <= start ? 0 : (pv - start) / (goal - start);
      left = eta !== null ? `${minutes(eta + need)}, plus the power step (${stepRange})` : null;
    } else if (t.phase === "settling") {
      const steady = t.steady ?? 0;
      title = `Step 2 of 3 · Holding steady at ${t.temp} °C`;
      detail = steady > 0
        ? `Steady for ${mmss(steady)} of ${mmss(need)} · kiln ${pv?.toFixed(1) ?? "–"} °C, ` +
          `moved ${Math.abs(t.drift ?? 0).toFixed(1)} of ${maxDrift} °C allowed`
        : `Waiting for the kiln to come within ±${band} °C of ${t.temp} °C (now ${pv?.toFixed(1) ?? "–"} °C${perMin})`;
      next = `Leaving ±${band} °C, or moving more than ${maxDrift} °C, restarts the ${need / 60} minutes. ` +
        "Next: the control loop switches off and a fixed power step is added.";
      progress = steady / need;
      left = `${minutes(need - steady)}, plus the power step (${stepRange})`;
    } else {
      const rise = t.rise ?? 0, maxRise = t.max_rise ?? TEST.maxRise, maxS = t.max_s ?? TEST.maxStep;
      const down = rise < -0.5;  // stepped down: the kiln was near full power
      const dr = r === null ? null : down ? -r : r;
      const toMaxRise = dr > 0.005 ? (maxRise - Math.abs(rise)) / dr : Infinity;
      const most = Math.max(0, Math.min(maxS - t.phase_el, toMaxRise));
      const least = Math.min(most, Math.max(0, TEST.minStep - t.phase_el));
      title = "Step 3 of 3 · Power step: recording the response";
      detail = `Kiln ${rise < 0 ? "−" : "+"}${Math.abs(rise).toFixed(1)} °C since the step began ${mmss(t.phase_el)} ago${perMin}` +
        (down ? " (power stepped down, since the kiln was near full power)" : "");
      next = `It stops once the ${down ? "cooling" : "heating"} rate has peaked (no sooner than ${TEST.minStep / 60} minutes in), ` +
        `at ${down ? "−" : "+"}${maxRise} °C, or after ${maxS / 60} minutes. Then heating turns off and the suggested gains appear below.`;
      progress = Math.max(Math.abs(rise) / maxRise, t.phase_el / maxS);
      left = least > 0 ? `at least ${minutes(least)}, at most ${minutes(most)}` : `at most ${minutes(most)}`;
    }

    const now = PHASE_ORDER.indexOf(t.phase);
    for (const li of live.querySelectorAll(".tun-steps li")) {
      const i = PHASE_ORDER.indexOf(li.dataset.phase);
      li.classList.toggle("now", i === now);
      li.classList.toggle("past", i < now);
    }
    const pct = Math.round(100 * Math.min(1, Math.max(0, progress)));
    $("tun-bar").style.width = `${pct}%`;
    $("tun-bar").parentElement.setAttribute("aria-valuenow", String(pct));
    $("tun-phase").textContent = title;
    $("tun-detail").textContent = detail;
    $("tun-times").textContent = `${mmss(t.el)} elapsed` + (left ? ` · time left: ${left}` : "");
    $("tun-next").textContent = next;
  }

  async function busy(fn) {
    device.busy = true;
    render();
    try {
      await fn();
    } catch (e) {
      actions.toast(e.message, true);
    } finally {
      device.busy = false;
      render();
    }
  }

  async function refreshDevice() {
    const t = await actions.request("tuning.get");
    device.points = sorted(t.points);
    device.base = t.base ?? null;
    device.defaultBase = t.default_base ?? null;
    try {
      device.result = (await actions.request("tune.result")).result;
    } catch {}
    render();
  }

  $("tun-add").addEventListener("click", () => {
    const last = points[points.length - 1];
    points.push(last ? { ...last, temp: Math.min(MAX_C, last.temp + 300) } : { temp: 150, ...DEFAULT_GAINS, hold: 0 });
    points = sorted(points);
    changed({ rebuild: true });
  });

  $("tun-send").addEventListener("click", () =>
    busy(async () => {
      await actions.request("tuning.set", device.base ? { points, base } : { points });
      actions.toast("Tuning sent to the controller");
      await refreshDevice();
    }));

  $("tun-load").addEventListener("click", () => {
    if (!device.points) return;
    if (!confirm("Replace the editor's tuning with the controller's?")) return;
    points = structuredClone(device.points);
    if (device.base) base = { ...device.base };
    changed({ rebuild: true });
  });

  for (const b of document.querySelectorAll("[data-tun-temp]")) {
    b.addEventListener("click", () => {
      $("tun-temp").value = b.dataset.tunTemp;
      render();
    });
  }
  $("tun-temp").addEventListener("input", render);

  $("tun-start").addEventListener("click", () => {
    const temp = Number($("tun-temp").value);
    const step = Number($("tun-step").value) / 100;
    if (!(temp >= 50 && temp <= 1300)) return actions.toast("Test temperature must be 50–1300 °C", true);
    if (!(step >= 0.05 && step <= 0.5)) return actions.toast("Power step must be 5–50 %", true);
    const msg =
      `Run a step test at ${temp} °C?\n\nThe controller heats the kiln to ${temp} °C and holds it until it has been ` +
      `steady (within ±${TEST.band} °C) for ${TEST.need / 60} minutes, then adds ${Math.round(step * 100)} % power ` +
      `for ${TEST.minStep / 60}–${TEST.maxStep / 60} minutes (stopping by +${TEST.maxRise} °C). On a small kiln the whole ` +
      "test usually takes 15–30 minutes. Heating stops at the end; Stop cancels it at any time.";
    if (!confirm(msg)) return;
    busy(async () => {
      await actions.request("tune.start", { temp, step });
      actions.toast("Step test started");
    });
  });

  $("tun-use").addEventListener("click", () => {
    const r = device.result;
    if (!r) return;
    const point = { temp: Math.round(r.temp), kp: r.kp, ki: r.ki, kd: r.kd, hold: r.hold };
    const near = points.findIndex((p) => Math.abs(p.temp - point.temp) < 25);
    if (near >= 0) points[near] = point;
    else if (points.length < MAX_POINTS) points.push(point);
    else return actions.toast(`The table holds ${MAX_POINTS} points; delete one first`, true);
    points = sorted(points);
    changed({ rebuild: true });
    actions.toast(`Added the ${point.temp} °C point; send it to the controller to use it`);
  });

  $("tun-use-base").addEventListener("click", () => {
    const r = device.result;
    if (!r) return;
    base = { kp: r.kp, ki: r.ki, kd: r.kd };
    changed({ rebuild: true });
    actions.toast("Baseline gains set; send them to the controller to use them");
  });

  // Preview: P and D (left axis) and holding power (right axis) vs temperature.
  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(canvas);
    const color = (n) => css.getPropertyValue(n).trim();
    ctx.font = `12px ${css.fontFamily}`;

    const pts = validationError() ? [] : points;
    const x0 = 48, x1 = w - 48, y0 = h - 26, y1 = 14;
    const X = (t) => x0 + (t / MAX_C) * (x1 - x0);
    const samples = [];
    for (let t = 0; t <= MAX_C; t += 10) {
      const g = gainsAt(pts, t, base);
      samples.push({ t, p: UNITS.kp.toUi(g.kp), d: UNITS.kd.toUi(g.kd), hold: feedforwardAt(pts, t) * 100 });
    }
    const gMax = Math.max(1, ...samples.map((s) => Math.max(s.p, s.d))) * 1.15;
    const gStep = pickStep(gMax / 4, [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50]);
    const gTop = Math.ceil(gMax / gStep) * gStep;
    const Y = (v) => y0 - (v / gTop) * (y0 - y1);
    const YH = (v) => y0 - (v / 100) * (y0 - y1);

    ctx.strokeStyle = color("--grid");
    ctx.fillStyle = color("--muted");
    ctx.lineWidth = 1;
    ctx.textBaseline = "middle";
    for (let v = 0; v <= gTop + 1e-9; v += gStep) {
      const y = Math.round(Y(v)) + 0.5;
      line(ctx, x0, y, x1, y);
      ctx.textAlign = "right";
      ctx.fillText(v.toFixed(gStep < 1 ? 1 : 0), x0 - 8, y);
    }
    ctx.textAlign = "left";
    for (const v of [0, 50, 100]) ctx.fillText(`${v}%`, x1 + 8, YH(v));
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let t = 0; t <= MAX_C; t += pickStep(MAX_C / Math.max(2, (x1 - x0) / 70), TEMP_STEPS)) {
      const x = Math.round(X(t)) + 0.5;
      line(ctx, x, y1, x, y0);
      ctx.fillText(`${t}°`, x, y0 + 8);
    }

    const series = (key, scale, col, width, dash = []) => {
      ctx.strokeStyle = color(col);
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.beginPath();
      samples.forEach((s, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, X(s.t), scale(s[key])));
      ctx.stroke();
      ctx.setLineDash([]);
    };
    series("hold", YH, "--power", 2, [5, 4]);
    series("d", Y, "--sp", 1.75);
    series("p", Y, "--pv", 2.5);
    ctx.fillStyle = color("--pv");
    for (const p of pts) {
      ctx.beginPath();
      ctx.arc(X(p.temp), Y(UNITS.kp.toUi(p.kp)), 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  new ResizeObserver(draw).observe(canvas);
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
  buildRows();
  buildBase();
  render();

  return {
    async setConnected(connected) {
      device.connected = connected;
      if (!connected) Object.assign(device, { status: null, points: null, base: null, defaultBase: null, result: null });
      render();
      if (connected) {
        try {
          await refreshDevice();
        } catch {
          device.points = null;  // older firmware without tuning
          render();
        }
      }
    },
    setStatus(status) {
      if (status?.state !== "tune" || status.tune?.phase !== device.status?.tune?.phase) {
        history = [];
        heatStart = status?.tune?.phase === "heating" && typeof status.pv === "number" ? status.pv : null;
      }
      if (typeof status?.pv === "number") {
        history.push({ t: performance.now(), pv: status.pv });
        while (history.length > 2 && history[history.length - 1].t - history[0].t > 60000) history.shift();
      }
      device.status = status;
      render();
    },
    onEvent(evt) {
      if (evt.evt === "tune_done") {
        device.result = evt.result;
        actions.toast(`Step test at ${Math.round(evt.result.temp)} °C finished`);
      } else if (evt.evt === "tune_failed") {
        actions.toast(`Step test stopped: ${evt.reason}`, true);
      }
      render();
    },
  };
}
