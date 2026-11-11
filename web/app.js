import { Chart } from "./chart.js";
import { DemoLink } from "./demo.js";
import { SerialLink } from "./device.js";
import { formatDuration, initProgramEditor } from "./program.js";
import { initTuning } from "./tuning.js";

const STALE_MS = 3500;          // no status for this long -> warn
const MAX_SAMPLES = 100_000;    // ~28 h at 1 Hz
const MAX_LOG_LINES = 400;

const $ = (id) => document.getElementById(id);
const demo = new URLSearchParams(location.search).has("demo");

const state = {
  link: null,
  info: null,          // reply to "hello"
  status: null,        // last status event
  lastStatusAt: 0,     // performance.now() of last status
  samples: [],
  rangeMs: 3_600_000,
  userDisconnected: false,
  connecting: false,
  stop: null,          // {reason, detail, alarm, at}: why heating last stopped
  stopDismissed: false,
  events: [],          // [{t, text}] noted in the CSV export
};

const chart = new Chart($("chart"), $("tooltip"));

// Controller's [[min, c], ...] format <-> the editor's {min, c} points.
const fromWire = (p) =>
  p ? { name: p.name, wait: p.wait !== false, points: p.points.map(([min, c]) => ({ min, c })) } : null;

const editor = initProgramEditor({
  // Starts the program, or updates a running one. atS: program time to run
  // from, in seconds; null keeps the running program's time (or starts at 0).
  async run(program, atS) {
    const params = {
      name: program.name || "",
      wait: program.wait !== false,
      points: program.points.map((p) => [p.min, p.c]),
      ...(atS === null ? {} : { at: atS }),
    };
    try {
      const r = await state.link.request("program.run", params);
      toast(r.updated ? (atS === null ? "Program updated; it keeps running" : "Program updated and moved") : "Program started");
      state.deviceProgram = fromWire(r.program);
      editor.setDevice({ program: state.deviceProgram });
      return state.deviceProgram;
    } catch (err) {
      toast(`Couldn't send the program: ${err.message}`, true);
      return null;
    }
  },
});

const tuning = initTuning({
  request: (cmd, params) => state.link.request(cmd, params),
  toast: (text, error) => toast(text, error),
});

// Ask the controller which program it has stored.
async function refreshDeviceProgram() {
  try {
    state.deviceProgram = fromWire((await state.link.request("program.get")).program);
  } catch {
    state.deviceProgram = null;  // older firmware without programs
  }
  editor.setDevice({ program: state.deviceProgram });
}

// ---- Connection ------------------------------------------------------------

async function connect(port) {
  if (state.connecting || state.link?.connected) return;
  state.connecting = true;
  setPill("busy", "Connecting…");
  const link = demo ? new DemoLink() : new SerialLink();
  try {
    await link.open(port);
    attach(link);
    state.info = await link.request("hello");
    if (state.info.proto !== 1) toast(`Controller speaks protocol ${state.info.proto}; this page expects 1`, true);
    state.userDisconnected = false;
    editor.setDevice({ connected: true });
    tuning.setConnected(true);
    await refreshDeviceProgram();
    render();
  } catch (err) {
    await link.close().catch(() => {});
    setPill("off", "Not connected");
    toast(friendlyError(err), true);
  } finally {
    state.connecting = false;
    render();
  }
}

// Why heating last stopped, from the controller's own record (`stopped` in
// the status line) or the `stopped` event. Normal ends aren't shown.
const QUIET_STOPS = new Set(["STOPPED BY PC", "STOPPED ON DIAL", "PROGRAM DONE", "STEP TEST DONE"]);
const STOP_TEXT = {
  "NO TEMP READING": "the controller had no usable thermocouple reading",
  "OVER TEMP": "the kiln went over the controller's maximum temperature",
  "STEP TEST FAILED": "the step test failed",
};
function stopMessage(x) {
  const what = STOP_TEXT[x.reason] ?? x.reason.toLowerCase();
  return `Heating stopped: ${what}${x.detail ? ` (${x.detail})` : ""}.`;
}

function attach(link) {
  state.link = link;
  link.addEventListener("status", (e) => onStatus(e.detail));
  link.addEventListener("line", (e) => logLine(e.detail.dir, e.detail.text));
  link.addEventListener("event", (e) => {
    tuning.onEvent(e.detail);
    if (e.detail.evt === "program_done") toast("Program finished");
    if (e.detail.evt === "program_aborted") toast(`Program aborted: ${e.detail.reason}`, true);
    if (e.detail.evt === "stopped" && !QUIET_STOPS.has(e.detail.reason)) {
      state.stop = { ...e.detail, at: link.now() };
      state.stopDismissed = false;
      state.events.push({ t: link.now(), text: stopMessage(e.detail) });
      render();
    }
  });
  link.addEventListener("disconnect", () => {
    if (state.link !== link) return;
    state.link = null;
    state.info = null;
    state.deviceProgram = null;
    editor.setDevice({ connected: false, program: null, status: null });
    tuning.setConnected(false);
    if (!state.userDisconnected) toast("Controller disconnected");
    render();
  });
}

async function onConnectClick() {
  if (state.link?.connected) {
    state.userDisconnected = true;
    await state.link.close();
    return;
  }
  if (demo) return connect();
  try {
    const port = await SerialLink.pickPort();
    await connect(port);
  } catch (err) {
    if (err.name !== "NotFoundError") toast(friendlyError(err), true);  // NotFound = picker cancelled
  }
}

// Reconnect without a picker to a controller this site was allowed before.
async function autoConnect() {
  if (state.userDisconnected || state.link?.connected) return;
  const [port] = await SerialLink.knownPorts();
  if (port) await connect(port);
}

function friendlyError(err) {
  if (err?.name === "NetworkError" || /already open/i.test(err?.message)) {
    return "The port is busy. Close any other program using it (serial monitor, PlatformIO), then try again.";
  }
  return err?.message || String(err);
}

// ---- Data ------------------------------------------------------------------

function onStatus(s) {
  state.status = s;
  state.lastStatusAt = performance.now();
  state.samples.push({
    t: state.link.now(),
    pv: typeof s.pv === "number" ? s.pv : null,
    sp: s.sp,
    duty: s.state !== "off" ? s.duty : 0,
    ssr: s.ssr,
    state: s.state,
    fault: s.fault || "",
    warn: s.warn ? `${s.warn.what} x${s.warn.n}` : "",
  });
  // The controller remembers the last stop until heating starts again, so
  // this also shows it after reconnecting.
  if (s.stopped && !QUIET_STOPS.has(s.stopped.reason)) {
    const at = state.link.now() - s.stopped.ago * 1000;
    if (!state.stop || Math.abs(state.stop.at - at) > 5000) {
      state.stop = { ...s.stopped, at };
      state.stopDismissed = false;
    }
  } else if (s.state !== "off") {
    state.stop = null;
  }
  if (state.samples.length > MAX_SAMPLES) state.samples.splice(0, state.samples.length - MAX_SAMPLES);
  render();
  chart.setData(state.samples, state.rangeMs);
  editor.setDevice({ status: s });
  tuning.setStatus(s);
}

function exportCsv() {
  const rows = [["time", "actual_c", "setpoint_c", "power_pct", "ssr", "state", "fault", "tc_warning", "event"]];
  let ev = 0;
  for (const s of state.samples) {
    const notes = [];
    while (ev < state.events.length && state.events[ev].t <= s.t) notes.push(state.events[ev++].text);
    rows.push([
      new Date(s.t).toISOString(),
      s.pv ?? "",
      s.sp,
      Math.round(s.duty * 1000) / 10,
      s.ssr,
      s.state,
      s.fault,
      s.warn ?? "",
      notes.length ? `"${notes.join(" ").replaceAll('"', "'")}"` : "",
    ]);
  }
  const blob = new Blob([rows.map((r) => r.join(",")).join("\n") + "\n"], { type: "text/csv" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `kiln-${new Date().toISOString().slice(0, 16).replace(/[:T]/g, "-")}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ---- Commands --------------------------------------------------------------

// Returns true on success; failures are shown as a toast.
async function command(cmd, params, okMessage) {
  try {
    await state.link.request(cmd, params);
    if (okMessage) toast(okMessage);
    return true;
  } catch (err) {
    toast(`${cmd} failed: ${err.message}`, true);
    return false;
  }
}

$("sp-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const input = $("sp-input");
  const value = Number(input.value);
  const max = state.info?.sp_max ?? 1400;
  if (!Number.isFinite(value) || value < 0 || value > max) {
    toast(`Setpoint must be 0–${max} °C`, true);
    return;
  }
  input.dataset.dirty = "";
  await command("setpoint", { value }, `Setpoint ${Math.round(value)} °C`);
  input.blur();
});
$("sp-input").addEventListener("input", (e) => (e.target.dataset.dirty = "1"));
$("start").addEventListener("click", () => command("start", {}, "Heating started"));
$("stop").addEventListener("click", () => {
  if (state.status?.state === "program" && !confirm("Stop the firing program? It can't be resumed from here.")) return;
  if (state.status?.state === "tune" && !confirm("Cancel the step test?")) return;
  command("stop", {}, "Heating stopped");
});
$("connect").addEventListener("click", onConnectClick);
$("export").addEventListener("click", exportCsv);
$("stop-dismiss").addEventListener("click", () => {
  state.stopDismissed = true;
  render();
});

for (const b of document.querySelectorAll(".seg button")) {
  b.addEventListener("click", () => {
    state.rangeMs = Number(b.dataset.range);
    for (const o of document.querySelectorAll(".seg button")) o.setAttribute("aria-pressed", String(o === b));
    chart.setData(state.samples, state.rangeMs);
  });
}

$("raw-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const input = $("raw-input");
  let msg;
  try {
    msg = JSON.parse(input.value);
  } catch {
    logLine("err", "not valid JSON");
    return;
  }
  const { cmd, id, ...params } = msg;
  if (typeof cmd !== "string") return logLine("err", 'needs a "cmd" field');
  state.link.request(cmd, params).catch((err) => logLine("err", err.message));
  input.value = "";
});

// ---- Rendering -------------------------------------------------------------

function render() {
  const link = state.link;
  const connected = !!link?.connected && !!state.info;
  const s = connected ? state.status : null;
  const stale = connected && performance.now() - state.lastStatusAt > STALE_MS;
  const fault = s?.fault || "";
  const programRunning = s?.state === "program";
  const tuneRunning = s?.state === "tune";
  const running = s?.state === "run" || programRunning || tuneRunning;

  if (!state.connecting) {
    if (connected) setPill("on", demo ? "Demo" : "Connected");
    else setPill("off", "Not connected");
  }
  $("connect").textContent = connected ? "Disconnect" : "Connect";
  $("connect").classList.toggle("primary", !connected);
  $("connect").disabled = state.connecting || (!demo && !SerialLink.supported());

  $("offline-note").hidden = connected || state.samples.length === 0;
  $("fault-banner").hidden = !fault;
  $("fault-banner").textContent = !fault ? "" : running
    ? `Sensor fault: ${fault}. Holding the last power level; heating stops if there's still no usable reading after 30 s.`
    : `Sensor fault: ${fault}. Heating can't start until the controller gets a usable reading.`;
  const stop = state.stop && !state.stopDismissed && connected ? state.stop : null;
  $("stop-banner").hidden = !stop;
  $("stop-banner").classList.toggle("danger", !!stop?.alarm);
  if (stop) {
    const when = new Date(stop.at).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    $("stop-text").textContent = `${when} · ${stopMessage(stop)}`;
  }
  const warn = s?.warn;
  $("warn-banner").hidden = !warn;
  $("warn-banner").textContent = warn
    ? `Thermocouple warning: ${warn.n} reading(s) flagged ${warn.what} since heating started. These usually come ` +
      "from current leaking from hot elements into the thermocouple; the readings are still used. Heating continues."
    : "";
  $("stale").hidden = !stale;

  const chip = $("state-chip");
  chip.dataset.state = !s ? "unknown" : fault ? "fault" : s.state;
  chip.textContent = !s ? "—" : fault ? "Fault" : programRunning ? "Program" : tuneRunning ? "Step test" : running ? "Heating" : "Off";
  $("sim-chip").hidden = !(state.info?.sim);

  const p = programRunning ? s.prog : null;
  $("prog-status").hidden = !p;
  if (p) {
    $("ps-name").textContent = p.name || "Untitled program";
    $("ps-wait").hidden = !p.wait;
    $("ps-bar").style.width = `${p.total ? (100 * p.el) / p.total : 0}%`;
    $("ps-seg").textContent = `Segment ${p.seg} of ${p.segs}`;
    $("ps-time").textContent = `${formatDuration(p.el / 60)} elapsed · ${formatDuration((p.total - p.el) / 60)} left`;
  }

  const hasPv = !!s && typeof s.pv === "number";
  $("pv").textContent = hasPv ? s.pv.toFixed(0) : "—";
  $("pv").parentElement.classList.toggle("empty", !hasPv);
  $("sp").textContent = s && s.sp != null ? s.sp.toFixed(0) : "—";
  $("duty").textContent = s ? Math.round((running ? s.duty : 0) * 100) : "—";
  const on = !!s && s.ssr === 1;
  $("ssr").classList.toggle("on", on);
  $("ssr").setAttribute("aria-label", on ? "on" : "off");
  $("ssr-text").textContent = s ? (on ? "On" : "Off") : "—";

  const input = $("sp-input");
  // A running program owns the setpoint; manual control is locked meanwhile.
  input.disabled = !connected || programRunning || tuneRunning;
  input.max = state.info?.sp_max ?? 1400;
  if (s && !programRunning && s.sp != null && document.activeElement !== input && !input.dataset.dirty) {
    input.value = s.sp.toFixed(0);
  }
  $("sp-apply").disabled = !connected || programRunning || tuneRunning;
  $("start").disabled = !connected || running || !!fault;
  $("stop").disabled = !connected || !running;
  $("export").disabled = state.samples.length === 0;
  for (const el of $("raw-form").elements) el.disabled = !connected;

  $("fw-info").textContent = state.info
    ? `Firmware ${state.info.fw} · protocol ${state.info.proto}${state.info.sim ? " · simulated kiln" : ""}`
    : "";
  document.title = s && typeof s.pv === "number" ? `${s.pv.toFixed(0)}°C · Kiln Controller` : "Kiln Controller";
}

function setPill(kind, text) {
  $("link-pill").dataset.state = kind;
  $("link-pill").textContent = text;
}

let toastTimer = 0;
function toast(text, error = false) {
  const el = $("toast");
  el.textContent = text;
  el.classList.toggle("error", error);
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (el.hidden = true), error ? 6000 : 2500);
}

function logLine(dir, text) {
  const log = $("log");
  const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
  const row = document.createElement("div");
  row.className = dir;
  row.textContent = `${dir === "tx" ? "→" : dir === "err" ? "!" : "←"} ${text}`;
  log.append(row);
  while (log.childElementCount > MAX_LOG_LINES) log.firstElementChild.remove();
  if (atBottom) log.scrollTop = log.scrollHeight;
}

// ---- Start -----------------------------------------------------------------

setInterval(render, 1000);  // keeps the "no data" warning current
render();
chart.setData(state.samples, state.rangeMs);

if (demo) {
  connect();
} else if (!SerialLink.supported()) {
  $("unsupported").hidden = false;
} else {
  navigator.serial.addEventListener("connect", autoConnect);  // re-plugged
  autoConnect();
}
