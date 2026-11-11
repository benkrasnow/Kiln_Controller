// Firing-program editor. A program is a list of control points
// {min, c}: minutes from program start and the temperature to reach then.
// The first point is always at 0 min. Between points the setpoint ramps
// linearly; equal temperatures make a hold.
//
// Points can be edited in the table or on the chart: drag a point to move
// it, click empty space to add one, double-click a point to delete it.

import { TEMP_STEPS, TIME_STEPS_MIN, line, pickStep } from "./chart.js";

const STORAGE_KEY = "kiln.program.v1";
const MAX_C = 1400;
const MAX_POINTS = 32;
const PAD = { left: 52, right: 20, top: 26, bottom: 30 };
const HIT_PX = 11;            // how close a pointer must be to grab a point
const CLICK_SLOP_PX = 4;      // movement allowed for a click
const SNAP_MIN = 5;           // drag/click snapping (hold Shift for 1)
const SNAP_C = 5;
const HEADROOM = 1.12;        // empty space past the last point / peak

const EXAMPLE = {
  name: "Example bisque",
  wait: true,
  points: [
    { min: 0, c: 20 },
    { min: 180, c: 200 },
    { min: 240, c: 200 },
    { min: 480, c: 1000 },
    { min: 495, c: 1000 },
  ],
};

// ---- Time parsing / formatting -------------------------------------------

// Accepts "90" (minutes), "1:30" (h:mm), "1.5h", "2h", "1h 30m", "45 min".
export function parseDuration(text) {
  const s = String(text).trim().toLowerCase();
  if (!s) return NaN;
  if (/^\d+(\.\d+)?$/.test(s)) return Number(s);
  let m = s.match(/^(\d+):([0-5]?\d)$/);
  if (m) return Number(m[1]) * 60 + Number(m[2]);
  m = s.match(/^(?:(\d+(?:\.\d+)?)\s*h(?:ours?|rs?)?)?\s*(?:(\d+(?:\.\d+)?)\s*m(?:in(?:utes?)?)?)?$/);
  if (m && (m[1] || m[2])) return Number(m[1] || 0) * 60 + Number(m[2] || 0);
  return NaN;
}

// Times are stored to the second (fractional minutes) but shown rounded
// down to the minute. The tolerance (0.06 s) absorbs float error, so a
// point sent as 180 min and read back as 179.9999 still shows as 3 h.
const wholeMinutes = (min) => Math.floor(min + 1e-3);

export function formatDuration(min) {
  const total = wholeMinutes(min);
  const h = Math.floor(total / 60);
  const m = total % 60;
  if (h === 0) return `${m} min`;
  return m === 0 ? `${h} h` : `${h} h ${m} min`;
}

// Segment lengths are always shown in minutes.
function formatMinutes(min) {
  return `${wholeMinutes(min)} min`;
}

export function clockLabel(min) {
  const total = wholeMinutes(min);
  return `${Math.floor(total / 60)}:${String(total % 60).padStart(2, "0")}`;
}

function formatRate(perMin) {
  if (Math.abs(perMin) < 0.005) return "hold";
  const digits = Math.abs(perMin) >= 10 ? 0 : Math.abs(perMin) >= 1 ? 1 : 2;
  return `${perMin > 0 ? "+" : "−"}${Math.abs(perMin).toFixed(digits)} °C/min`;
}

// ---- Model ---------------------------------------------------------------

// Segments between consecutive points, with problems flagged.
export function segments(points) {
  const out = [];
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];
    const minutes = b.min - a.min;
    out.push({
      from: a,
      to: b,
      minutes,
      rate: minutes > 0 ? (b.c - a.c) / minutes : NaN,
      error: minutes > 0 ? null : "must be later than the point before",
    });
  }
  return out;
}

// Profile temperature at `min` minutes (linear between points).
export function profileAt(points, min) {
  if (min <= points[0].min) return points[0].c;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    if (min <= b.min) return b.min > a.min ? a.c + ((b.c - a.c) * (min - a.min)) / (b.min - a.min) : b.c;
  }
  return points[points.length - 1].c;
}

export function isValidProgram(program) {
  return program.points.length >= 2 && segments(program.points).every((s) => !s.error);
}

// "Wait for the kiln" defaults to on when a program doesn't say.
export const waitsForKiln = (program) => program.wait !== false;

export function samePrograms(a, b) {
  if (!a || !b || a.points.length !== b.points.length || (a.name || "") !== (b.name || "")) return false;
  if (waitsForKiln(a) !== waitsForKiln(b)) return false;
  return a.points.every((p, i) => Math.abs(p.min - b.points[i].min) < 0.01 && Math.abs(p.c - b.points[i].c) < 0.01);
}

function validPoint(p) {
  return Number.isFinite(p.min) && p.min >= 0 && Number.isFinite(p.c) && p.c >= 0 && p.c <= MAX_C;
}

function load() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (saved && Array.isArray(saved.points) && saved.points.length && saved.points.every(validPoint)) {
      return { ...saved, wait: waitsForKiln(saved) };
    }
  } catch {}
  return structuredClone(EXAMPLE);
}

function save(program) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(program));
  } catch {}
}

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// ---- Editor --------------------------------------------------------------

// actions.run(program, atSeconds|null) -> Promise of the program the
// controller is now running (or null on failure), wired by app.js.
export function initProgramEditor(actions) {
  const $ = (id) => document.getElementById(id);
  const rows = $("prog-rows");
  const canvas = $("prog-chart");
  let program = load();
  let device = { connected: false, program: null, status: null, busy: false };

  // Chart interaction state.
  let geom = null;      // scale/transform from the last draw
  let frozen = null;    // scale held fixed while dragging
  let drag = null;      // {index, x, y, moved}
  let hover = -1;
  let rowsQueued = false;
  // While a program runs, the user can drag the "now" line to choose a new
  // program time to jump to; it applies when the program is sent.
  let jumpMin = null;

  // Program time of the running program in minutes, or null if none runs.
  const runningNow = () =>
    device.status?.state === "program" && device.status.prog ? device.status.prog.el / 60 : null;

  const changed = ({ rebuild = false } = {}) => {
    save(program);
    if (rebuild) buildRows();
    updateSegmentCells();
    updateDeviceRow();
    draw();
  };

  // Rebuilding the table on every pointer move is wasteful; once per frame.
  const changedDuringDrag = () => {
    save(program);
    draw();
    if (!rowsQueued) {
      rowsQueued = true;
      requestAnimationFrame(() => {
        rowsQueued = false;
        buildRows();
        updateSegmentCells();
        updateDeviceRow();
      });
    }
  };

  // Sort by time once an edit is finished (not while typing).
  const settle = () => {
    const [first, ...rest] = program.points;
    rest.sort((a, b) => a.min - b.min);
    program.points = [first, ...rest];
    changed({ rebuild: true });
  };

  function buildRows() {
    rows.replaceChildren();
    program.points.forEach((p, i) => {
      const tr = document.createElement("tr");
      if (i === drag?.index || (drag === null && i === hover)) tr.classList.add("active");
      tr.innerHTML = `
        <td class="idx">${i}</td>
        <td>${i === 0
          ? '<span class="start">0:00 · start</span>'
          : `<input class="time" value="${formatDuration(p.min)}" aria-label="Time of point ${i}" spellcheck="false">`}</td>
        <td><div class="cell-unit"><input class="temp" type="number" min="0" max="${MAX_C}" step="1"
             value="${p.c}" aria-label="Temperature of point ${i}"><span>°C</span></div></td>
        <td class="seg-info"></td>
        <td>${i === 0 ? "" : `<button class="icon-btn" aria-label="Delete point ${i}" title="Delete">×</button>`}</td>`;

      const time = tr.querySelector(".time");
      if (time) {
        time.addEventListener("input", () => {
          const v = parseDuration(time.value);
          const ok = v > 0;
          time.classList.toggle("bad", !ok);
          if (ok) {
            p.min = v;
            changed();
          }
        });
        time.addEventListener("change", () => {
          if (!time.classList.contains("bad")) settle();
          else time.value = formatDuration(p.min);  // revert nonsense or a time in the past
          time.classList.remove("bad");
        });
      }
      const temp = tr.querySelector(".temp");
      temp.addEventListener("input", () => {
        const v = Number(temp.value);
        const ok = temp.value !== "" && v >= 0 && v <= MAX_C;
        temp.classList.toggle("bad", !ok);
        if (ok) {
          p.c = v;
          changed();
        }
      });
      temp.addEventListener("change", () => {
        temp.value = p.c;
        temp.classList.remove("bad");
      });
      tr.querySelector(".icon-btn")?.addEventListener("click", () => deletePoint(i));
      rows.append(tr);
    });
    $("prog-add").disabled = program.points.length >= MAX_POINTS;
    if (document.activeElement !== $("prog-name")) $("prog-name").value = program.name || "";
    $("prog-wait").checked = waitsForKiln(program);
  }

  function updateSegmentCells() {
    const segs = segments(program.points);
    rows.querySelectorAll(".seg-info").forEach((cell, i) => {
      const s = segs[i - 1];
      cell.className = "seg-info";
      if (!s) {
        cell.textContent = "";
      } else if (s.error) {
        cell.textContent = `Time ${s.error}`;
        cell.classList.add("error");
      } else {
        cell.innerHTML = `<b>${formatRate(s.rate)}</b><span>${formatMinutes(s.minutes)}</span>`;
        if (Math.abs(s.rate) < 0.005) cell.classList.add("hold");
      }
    });
    const last = program.points[program.points.length - 1];
    const peak = Math.max(...program.points.map((p) => p.c));
    const bad = segs.some((s) => s.error);
    $("prog-summary").textContent = bad
      ? "Fix the highlighted times: each point must come after the one before."
      : `${program.points.length} points · total ${formatDuration(last.min)} · peak ${peak} °C`;
  }

  function deletePoint(i) {
    if (i <= 0 || i >= program.points.length) return;
    program.points.splice(i, 1);
    hover = -1;
    changed({ rebuild: true });
  }

  function addPoint(min, c) {
    if (program.points.length >= MAX_POINTS || min < 1) return false;
    if (program.points.some((p) => Math.abs(p.min - min) < 1)) return false;
    program.points.push({ min, c });
    settle();
    return true;
  }

  // ---- Controller section (send-and-start / load) ----

  function updateDeviceRow() {
    const dev = device.program;
    const now = runningNow();
    const running = now !== null;
    if (!running) jumpMin = null;
    const valid = isValidProgram(program);
    const match = samePrograms(program, dev);
    const fault = !!device.status?.fault;
    const info = $("prog-device");
    if (!device.connected) {
      info.textContent = "Connect to the controller to send this program.";
    } else if (!dev) {
      info.textContent = "The controller has no program yet.";
    } else {
      const total = formatDuration(dev.points[dev.points.length - 1].min);
      info.innerHTML = "";
      const mode = waitsForKiln(dev) ? "waits for kiln" : "time only";
      info.append(`On controller: “${dev.name || "Untitled"}” · ${dev.points.length} points · ${total} · ${mode} `);
      const tag = document.createElement("span");
      tag.className = `tag ${match ? "ok" : "diff"}`;
      tag.textContent = match ? "same as editor" : running ? "edited, not sent" : "differs from editor";
      info.append(tag);
    }

    // Not running: send and start. Running: send edits and/or a jump.
    $("prog-run").disabled = !device.connected || device.busy || !valid || fault ||
      (running && match && jumpMin === null);
    $("prog-load").disabled = !device.connected || device.busy || !dev || match;

    const note = $("prog-live-note");
    note.hidden = !running;
    if (running) {
      note.textContent = jumpMin !== null
        ? `Will jump to ${clockLabel(jumpMin)} when sent (now ${clockLabel(now)}). Press Esc or drag the marker ` +
          "back to the dotted line to cancel."
        : `Program running at ${clockLabel(now)}. Edit any point, or drag the dotted “now” line to choose ` +
          "where to continue from, then send.";
    }
  }

  async function runAction(fn) {
    device.busy = true;
    updateDeviceRow();
    try {
      await fn();
    } finally {
      device.busy = false;
      updateDeviceRow();
    }
  }

  $("prog-run").addEventListener("click", () => {
    const name = program.name || "Untitled";
    const total = formatDuration(program.points[program.points.length - 1].min);
    const now = runningNow();
    const at = jumpMin;
    const replaces = now !== null && (program.name || "") !== (device.program?.name || "");
    let question;
    if (now === null) {
      question = `Start “${name}” (${total}) on the controller now?` +
        (device.status?.state === "run" ? "\n\nThis takes over from manual setpoint control." : "");
    } else {
      question = (replaces ? `Replace the running program with “${name}”` : "Update the running program") +
        (at !== null ? ` and jump to ${clockLabel(at)}?` : "? It carries on from the controller's current time.");
    }
    if (!confirm(question)) return;
    runAction(async () => {
      // `at` is seconds of program time; when omitted the controller keeps its
      // own current time (or starts at 0 if nothing is running).
      const running = await actions.run(structuredClone(program), at === null ? null : at * 60);
      if (running) {
        jumpMin = null;
        program = structuredClone(running);
        changed({ rebuild: true });
      }
    });
  });
  $("prog-load").addEventListener("click", () => {
    if (!device.program) return;
    if (!confirm("Replace the editor's program with the one on the controller?")) return;
    program = structuredClone(device.program);
    changed({ rebuild: true });
  });

  // ---- Editor buttons ----

  $("prog-add").addEventListener("click", () => {
    const last = program.points[program.points.length - 1];
    program.points.push({ min: last.min + 60, c: last.c });
    changed({ rebuild: true });
    rows.querySelector("tr:last-child .temp")?.focus();
  });

  $("prog-name").addEventListener("input", (e) => {
    program.name = e.target.value.slice(0, 32);
    save(program);
    updateDeviceRow();
  });

  $("prog-wait").addEventListener("change", (e) => {
    program.wait = e.target.checked;
    save(program);
    updateDeviceRow();
  });

  $("prog-clear").addEventListener("click", () => {
    if (!confirm("Clear this program and start a new one?")) return;
    program = { name: "", wait: true, points: [{ min: 0, c: 20 }, { min: 60, c: 100 }] };
    changed({ rebuild: true });
  });

  $("prog-export").addEventListener("click", () => {
    const file = {
      format: "kiln-program", version: 1, name: program.name, wait: waitsForKiln(program), points: program.points,
    };
    const blob = new Blob([JSON.stringify(file, null, 2) + "\n"], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `${(program.name || "kiln-program").replace(/[^\w.-]+/g, "_")}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });

  $("prog-import").addEventListener("click", () => $("prog-file").click());
  $("prog-file").addEventListener("change", async (e) => {
    const file = e.target.files[0];
    e.target.value = "";
    if (!file) return;
    try {
      const data = JSON.parse(await file.text());
      const pts = data.points;
      if (data.format !== "kiln-program" || !Array.isArray(pts) || pts.length < 1 || pts.length > MAX_POINTS ||
          !pts.every(validPoint) || pts[0].min !== 0) {
        throw new Error("not a kiln program file");
      }
      program = {
        name: String(data.name || ""),
        wait: data.wait !== false,
        points: pts.map((p) => ({ min: p.min, c: p.c })),
      };
      settle();
    } catch (err) {
      alert(`Couldn't import ${file.name}: ${err.message}`);
    }
  });

  // ---- Chart: scale, drawing, interaction ----

  function computeScale(w, h) {
    const pts = program.points;
    const tMax = Math.max(60, ...pts.map((p) => p.min)) * HEADROOM;
    const cMax = Math.max(100, ...pts.map((p) => p.c));
    const yStep = pickStep(cMax / 5, TEMP_STEPS);
    const yMax = Math.ceil((cMax * HEADROOM) / yStep) * yStep;
    return { w, h, tMax, yMax, yStep };
  }

  function makeGeom(s) {
    const x0 = PAD.left, x1 = s.w - PAD.right, y0 = s.h - PAD.bottom, y1 = PAD.top;
    return {
      ...s, x0, x1, y0, y1,
      X: (min) => x0 + (min / s.tMax) * (x1 - x0),
      Y: (c) => y0 - (c / s.yMax) * (y0 - y1),
      T: (x) => ((x - x0) / (x1 - x0)) * s.tMax,
      C: (y) => ((y0 - y) / (y0 - y1)) * s.yMax,
    };
  }

  function hitTest(x, y) {
    if (!geom) return -1;
    let best = -1, bestD = HIT_PX;
    program.points.forEach((p, i) => {
      const d = Math.hypot(geom.X(p.min) - x, geom.Y(p.c) - y);
      if (d <= bestD) [best, bestD] = [i, d];
    });
    return best;
  }

  const inPlot = (x, y) => geom && x >= geom.x0 && x <= geom.x1 && y >= geom.y1 && y <= geom.y0;
  const snap = (v, step) => Math.round(v / step) * step;

  // The time marker: the chosen jump time if any, else the live "now" line.
  const MARKER = -2;  // drag.index value while dragging the marker
  const markerMin = () => jumpMin ?? runningNow();
  const onMarker = (x, y) => {
    const t = markerMin();
    return t !== null && inPlot(x, y) && Math.abs(geom.X(t) - x) <= 6;
  };
  const programEnd = () => program.points[program.points.length - 1].min;

  function setJump(min) {
    jumpMin = min;
    updateDeviceRow();
    draw();
  }

  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && jumpMin !== null) setJump(null);
  });

  canvas.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return;
    let index = hitTest(e.offsetX, e.offsetY);
    if (index < 0 && onMarker(e.offsetX, e.offsetY)) index = MARKER;
    drag = { index, x: e.offsetX, y: e.offsetY, moved: false };
    if (index >= 0 || index === MARKER) {
      frozen = computeScale(canvas.clientWidth, canvas.clientHeight);
      canvas.setPointerCapture(e.pointerId);
      e.preventDefault();
    }
  });

  canvas.addEventListener("pointermove", (e) => {
    const x = e.offsetX, y = e.offsetY;
    if (drag && drag.index === MARKER) {
      drag.moved = true;
      // Whole minutes, or exact (to the second) with Shift.
      const t = e.shiftKey ? Math.round(geom.T(x) * 60) / 60 : snap(geom.T(x), 1);
      jumpMin = clamp(t, 0, Math.max(0, programEnd() - 1 / 60));
      updateDeviceRow();
      draw();
      return;
    }
    if (drag && drag.index >= 0) {
      if (!drag.moved && Math.hypot(x - drag.x, y - drag.y) < CLICK_SLOP_PX) return;
      drag.moved = true;
      const i = drag.index;
      const p = program.points[i];
      const stepT = e.shiftKey ? 1 : SNAP_MIN;
      const stepC = e.shiftKey ? 1 : SNAP_C;
      p.c = clamp(snap(geom.C(y), stepC), 0, MAX_C);
      if (i > 0) {
        // Keep the point between its neighbours so the order never changes.
        const prev = program.points[i - 1].min + 1;
        const next = program.points[i + 1]?.min ?? Infinity;
        p.min = clamp(snap(geom.T(x), stepT), prev, Math.min(next - 1, geom.tMax));
      }
      changedDuringDrag();
      return;
    }
    const h = hitTest(x, y);
    canvas.style.cursor = h >= 0 ? "grab" : onMarker(x, y) ? "ew-resize" : inPlot(x, y) ? "crosshair" : "default";
    if (h !== hover) {
      hover = h;
      draw();
      rows.querySelectorAll("tr").forEach((tr, i) => tr.classList.toggle("active", i === hover));
    }
  });

  const endDrag = (e) => {
    if (!drag) return;
    const d = drag;
    drag = null;
    frozen = null;
    if (d.index === MARKER) {
      canvas.releasePointerCapture?.(e.pointerId);
      // Dropped back on the live line: no jump after all.
      const now = runningNow();
      if (jumpMin !== null && (now === null || Math.abs(jumpMin - now) < 1)) setJump(null);
    } else if (d.index >= 0) {
      canvas.releasePointerCapture?.(e.pointerId);
      changed({ rebuild: true });
    } else if (e.type === "pointerup" && Math.hypot(e.offsetX - d.x, e.offsetY - d.y) < CLICK_SLOP_PX &&
               inPlot(e.offsetX, e.offsetY)) {
      const stepT = e.shiftKey ? 1 : SNAP_MIN;
      const stepC = e.shiftKey ? 1 : SNAP_C;
      addPoint(snap(geom.T(e.offsetX), stepT), clamp(snap(geom.C(e.offsetY), stepC), 0, MAX_C));
    }
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("pointerleave", () => {
    if (!drag && hover !== -1) {
      hover = -1;
      draw();
    }
  });
  canvas.addEventListener("dblclick", (e) => deletePoint(hitTest(e.offsetX, e.offsetY)));

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(canvas);
    const color = (n) => css.getPropertyValue(n).trim();
    const family = css.fontFamily;

    geom = makeGeom(frozen ?? computeScale(w, h));
    const { X, Y, x0, x1, y0, y1, tMax, yMax, yStep } = geom;
    const pts = program.points;

    // Grid and axes.
    ctx.font = `12px ${family}`;
    ctx.strokeStyle = color("--grid");
    ctx.fillStyle = color("--muted");
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let c = 0; c <= yMax; c += yStep) {
      const y = Math.round(Y(c)) + 0.5;
      line(ctx, x0, y, x1, y);
      ctx.fillText(`${c}°`, x0 - 8, y);
    }
    const tStep = pickStep(tMax / Math.max(2, Math.floor((x1 - x0) / 80)), TIME_STEPS_MIN);
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let t = 0; t <= tMax + 1e-9; t += tStep) {
      const x = Math.round(X(t)) + 0.5;
      line(ctx, x, y1, x, y0);
      ctx.fillText(clockLabel(t), x, y0 + 8);
    }

    const segs = segments(pts);

    // Area under the profile, then the profile itself.
    const grad = ctx.createLinearGradient(0, y1, 0, y0);
    grad.addColorStop(0, color("--prog-fill-top"));
    grad.addColorStop(1, color("--prog-fill-bottom"));
    ctx.beginPath();
    ctx.moveTo(X(pts[0].min), y0);
    for (const p of pts) ctx.lineTo(X(p.min), Y(p.c));
    ctx.lineTo(X(pts[pts.length - 1].min), y0);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    ctx.lineJoin = "round";
    ctx.lineWidth = 2.5;
    for (const s of segs) {
      ctx.strokeStyle = s.error ? color("--danger") : color("--pv");
      ctx.setLineDash(s.error ? [5, 4] : []);
      line(ctx, X(s.from.min), Y(s.from.c), X(s.to.min), Y(s.to.c));
    }
    ctx.setLineDash([]);

    // The running program's position (dotted), the kiln's actual temperature
    // on it, and a chosen jump time (solid) if the user dragged the line.
    const st = device.status;
    const now = runningNow();
    const label = (text, x) => {
      ctx.font = `600 11px ${family}`;
      ctx.textAlign = x > x1 - 60 ? "right" : "left";
      ctx.textBaseline = "top";
      ctx.fillText(text, x + (x > x1 - 60 ? -4 : 4), y1 - 16);
    };
    if (now !== null) {
      const x = Math.round(X(now)) + 0.5;
      ctx.strokeStyle = color("--sp");
      ctx.fillStyle = color("--sp");
      ctx.globalAlpha = jumpMin !== null ? 0.45 : 1;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 3]);
      line(ctx, x, y1, x, y0);
      ctx.setLineDash([]);
      if (typeof st.pv === "number") {
        ctx.beginPath();
        ctx.arc(x, Y(st.pv), 5, 0, Math.PI * 2);
        ctx.fill();
      }
      if (jumpMin === null) label("now", x);
      ctx.globalAlpha = 1;
    }
    if (now !== null && jumpMin !== null) {
      const x = Math.round(X(jumpMin)) + 0.5;
      ctx.strokeStyle = color("--sp");
      ctx.fillStyle = color("--sp");
      ctx.lineWidth = 2.5;
      line(ctx, x, y1, x, y0);
      ctx.beginPath();  // grab handle
      ctx.arc(x, y1 + 6, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();  // where the setpoint will be after the jump
      ctx.arc(x, Y(profileAt(pts, jumpMin)), 5, 0, Math.PI * 2);
      ctx.fill();
      label(`start at ${clockLabel(jumpMin)}`, x);
    }

    // Segment annotations, written along each segment.
    for (const s of segs) {
      if (s.error) continue;
      const ax = X(s.from.min), ay = Y(s.from.c);
      const bx = X(s.to.min), by = Y(s.to.c);
      const len = Math.hypot(bx - ax, by - ay);
      const angle = Math.atan2(by - ay, bx - ax);
      const rate = formatRate(s.rate);
      const dur = formatMinutes(s.minutes);
      ctx.save();
      ctx.translate((ax + bx) / 2, (ay + by) / 2);
      ctx.rotate(angle);
      ctx.textAlign = "center";
      ctx.font = `600 12px ${family}`;
      const fits = ctx.measureText(rate).width + 10 < len;
      ctx.font = `600 ${fits ? 12 : 10}px ${family}`;
      if (ctx.measureText(rate).width + 6 < len) {
        ctx.fillStyle = color("--text");
        ctx.textBaseline = "bottom";
        ctx.fillText(rate, 0, -6);
        ctx.font = `${fits ? 12 : 10}px ${family}`;
        ctx.fillStyle = color("--muted");
        ctx.textBaseline = "top";
        if (ctx.measureText(dur).width + 6 < len) ctx.fillText(dur, 0, 6);
      }
      ctx.restore();
    }

    // Control points with their temperatures.
    const active = drag?.index >= 0 ? drag.index : hover;
    ctx.font = `600 11px ${family}`;
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    pts.forEach((p, i) => {
      const x = X(p.min), y = Y(p.c);
      const isActive = i === active;
      ctx.fillStyle = isActive ? color("--pv") : color("--surface");
      ctx.strokeStyle = color("--pv");
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x, y, isActive ? 6.5 : 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      // Label a point where the profile changes direction or at the ends.
      const prev = pts[i - 1], next = pts[i + 1];
      const corner = !prev || !next || Math.sign(p.c - prev.c) !== Math.sign(next.c - p.c);
      if (corner && i > 0 && !isActive) {
        ctx.fillStyle = color("--muted");
        ctx.fillText(`${p.c}°`, clamp(x, x0 + 14, x1 - 14), y - 8);
      }
    });

    // Readout for the point being dragged or hovered.
    if (active >= 0 && pts[active]) {
      const p = pts[active];
      const label = `${clockLabel(p.min)} · ${Math.round(p.c)} °C`;
      ctx.font = `600 12px ${family}`;
      const tw = ctx.measureText(label).width + 14;
      const bx = clamp(X(p.min) - tw / 2, x0, x1 - tw);
      const by = Y(p.c) - 36 < y1 - 20 ? Y(p.c) + 14 : Y(p.c) - 36;
      ctx.fillStyle = color("--text");
      roundRect(ctx, bx, by, tw, 22, 6);
      ctx.fill();
      ctx.fillStyle = color("--bg");
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillText(label, bx + 7, by + 11);
    }
  }

  new ResizeObserver(draw).observe(canvas);
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
  buildRows();
  changed();

  return {
    getProgram: () => program,
    // state: {connected, program, status}; any subset.
    setDevice(state) {
      device = { ...device, ...state };
      updateDeviceRow();
      draw();
    },
  };
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}
