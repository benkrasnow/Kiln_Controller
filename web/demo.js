// In-browser stand-in for the controller (open the app with ?demo).
// Speaks the same line protocol as the firmware and runs the same kiln model,
// PID, gain schedule and step test as lib/kiln_core, sped up so a firing's
// worth of chart fills in minutes. For UI work only; it shares no code with
// the real firmware.

import { Link } from "./device.js";
import { DEFAULT_GAINS, feedforwardAt, gainsAt } from "./tuning.js";

// Simulated seconds per real second: ?demo (30) or e.g. ?demo=120.
const SPEEDUP = Math.max(1, Number(new URLSearchParams(location.search).get("demo")) || 30);
const TICK_MS = 1000;      // one status line per real second, like the device
const SUBSTEPS = SPEEDUP;  // 1 s model steps
const WAIT_BAND_C = 15;    // ProgramRunner::kWaitBandC

// lib/kiln_core/src/kiln_model.h
const MODEL = { ambient: 22, heatPerS: 0.35, kCond: 8.2e-5, kRad: 4.0e-14, sensorTau: 40 };
const D_TAU_S = 10;        // lib/kiln_core/src/pid.h
const APPROACH_C_PER_MIN = 0;   // StepTestConfig::approach_c_per_min (0: no ramp)
const SETTLE_BAND_C = 5;   // StepTestConfig::settle_band_c
const SETTLE_DRIFT_C = 5;  // StepTestConfig::settle_drift_c
const SETTLE_S = 300;      // StepTestConfig::settle_s

function lossCPerS(c) {
  const t = c + 273.15, ta = MODEL.ambient + 273.15;
  return MODEL.kCond * (c - MODEL.ambient) + MODEL.kRad * (t ** 4 - ta ** 4);
}

const newPid = () => ({ integral: 0, lastPv: null, dRate: 0 });

// Same algorithm as lib/kiln_core/src/pid.cpp (with feed-forward).
function pidUpdate(s, g, sp, pv, dt, ff = 0) {
  if (s.lastPv === null) s.lastPv = pv;
  const rate = (pv - s.lastPv) / dt;
  s.lastPv = pv;
  s.dRate += (dt / (dt + D_TAU_S)) * (rate - s.dRate);
  const e = sp - pv;
  const p = g.kp * e;
  const d = -g.kd * s.dRate;
  const iNext = s.integral + g.ki * e * dt;
  const u = ff + p + iNext + d;
  if (!(u > 1 && e > 0) && !(u < 0 && e < 0)) s.integral = Math.min(1, Math.max(-1, iNext));
  return Math.min(1, Math.max(0, ff + p + s.integral + d));
}

// Same rule as lib/kiln_core suggestGains (SIMC, tau_c = dead time / 2).
function suggestGains(rate, dead) {
  const theta = Math.max(10, dead);
  const kc = 1 / (rate * 1.5 * theta);
  return { kp: kc, ki: kc / (6 * theta), kd: kc * 0.5 * theta };
}

// Port of lib/kiln_core StepTest.
class StepTest {
  constructor(target, step, holdGains) {
    Object.assign(this, { target, stepCfg: step, phase: "heating", total: 0, phaseS: 0, inBand: 0, dutySum: 0 });
    this.pid = newPid();
    this.gains = holdGains;
  }

  get active() {
    return ["heating", "settling", "stepping"].includes(this.phase);
  }

  fail(why) {
    this.phase = "failed";
    this.reason = why;
  }

  update(pv, dt) {
    this.lastPv = pv;
    this.total += dt;
    this.phaseS += dt;
    if (this.total > 6 * 3600) return this.fail("took too long"), 0;
    if (this.phase === "heating" || this.phase === "settling") {
      let sp = this.target;
      if (this.phase === "heating" && APPROACH_C_PER_MIN > 0) {
        this.rampSp = Math.min(this.target, (this.rampSp ?? pv) + APPROACH_C_PER_MIN * dt / 60);
        sp = this.rampSp;
      }
      const duty = pidUpdate(this.pid, this.gains, sp, pv, dt);
      if (this.phase === "heating") {
        if (sp >= this.target && pv >= this.target - SETTLE_BAND_C) Object.assign(this, { phase: "settling", phaseS: 0, inBand: 0, dutySum: 0 });
        return duty;
      }
      if (Math.abs(pv - this.target) <= SETTLE_BAND_C) {
        if (this.inBand === 0) this.bandPv0 = pv;
        this.inBand += dt;
        this.dutySum += duty * dt;
      } else {
        this.inBand = this.dutySum = 0;
      }
      if (this.inBand >= SETTLE_S && Math.abs(pv - this.bandPv0) > SETTLE_DRIFT_C) this.inBand = this.dutySum = 0;
      if (this.inBand >= SETTLE_S) {
        this.hold = this.dutySum / this.inBand;
        this.holdDrift = (pv - this.bandPv0) / this.inBand;
        this.step = Math.min(1, this.hold + this.stepCfg) - this.hold;
        if (this.step < 0.05) this.step = -Math.min(this.stepCfg, this.hold);  // no room: step down
        if (Math.abs(this.step) < 0.05) return this.fail("holding power leaves no room for a step"), 0;
        Object.assign(this, { phase: "stepping", phaseS: 0, pv0: pv, samples: [pv], since: 0, maxSlope: 0 });
      }
      return duty;
    }
    // Stepping: fixed power; slope over the last minute, sampled every 10 s.
    this.since += dt;
    if (this.since >= 10) {
      this.since = 0;
      this.samples.push(pv);
      if (this.samples.length > 7) this.samples.shift();
      if (this.samples.length === 7) {
        const dir = Math.sign(this.step);
        const slope = (dir * (this.samples[6] - this.samples[0])) / 60;
        if (slope > this.maxSlope) {
          Object.assign(this, { maxSlope: slope, tAtMax: this.phaseS - 30, pvAtMax: (this.samples[0] + this.samples[6]) / 2 });
        }
        const pastPeak = this.maxSlope > 0 && slope < 0.7 * this.maxSlope && this.phaseS > 300;
        if (pastPeak || dir * (pv - this.pv0) >= 50 || this.phaseS >= 3600) this.finish();
      }
    }
    return this.phase === "stepping" ? this.hold + this.step : 0;
  }

  finish() {
    const dir = Math.sign(this.step);
    if (this.maxSlope <= 0) return this.fail(dir > 0 ? "kiln did not heat during the step" : "kiln did not cool during the step");
    const dead = Math.max(0, this.tAtMax - (dir * (this.pvAtMax - this.pv0)) / this.maxSlope);
    const rate = this.maxSlope / Math.abs(this.step);
    const hold = Math.min(1, Math.max(0, this.hold - this.holdDrift / rate));  // drift-corrected
    this.result = { temp: this.target, hold, step: this.step, rate, dead, ...suggestGains(rate, dead) };
    this.phase = "done";
  }
}

export class DemoLink extends Link {
  #timer = null;
  #t0 = Date.now();
  #simS = 0;
  #kiln = MODEL.ambient;
  #tc = MODEL.ambient;
  #sp = 850;
  #mode = "off";           // "off" | "run" | "program" | "tune"
  #program = null;         // {name, wait, points: [[min, c], ...]}
  #el = 0;                 // program clock, seconds
  #waiting = false;
  #duty = 0;
  #pid = newPid();
  #tuning = [];            // [{temp, kp, ki, kd, hold}], sorted
  #base = { ...DEFAULT_GAINS };  // gains with no points
  #test = null;            // StepTest while in "tune" mode
  #stopped = null;         // why heating last stopped (alarms only here)
  #lastResult = null;

  get connected() {
    return this.#timer !== null;
  }

  now() {
    return this.#t0 + this.#simS * 1000;
  }

  async open() {
    this.#timer = setInterval(() => this.#tick(), TICK_MS);
  }

  async close() {
    clearInterval(this.#timer);
    this.#timer = null;
    this.linkLost();
  }

  #event(obj) {
    this.receive(JSON.stringify(obj) + "\n");
  }

  async writeLine(text) {
    const req = JSON.parse(text);
    const reply = (extra = {}, error) =>
      setTimeout(() => this.receive(JSON.stringify({ id: req.id, ok: !error, ...(error ? { error } : extra) }) + "\n"), 30);
    const busy = this.#mode === "program" ? "program running; stop it first"
      : this.#mode === "tune" ? "step test running; stop it first" : null;

    switch (req.cmd) {
      case "hello":
        return reply({ fw: "demo", proto: 1, sim: true, sp_min: 0, sp_max: 1400, max_points: 32, max_tune_points: 4 });
      case "setpoint":
        if (this.#mode === "program") return reply({}, busy);
        if (typeof req.value !== "number") return reply({}, "value must be a number");
        if (req.value < 0 || req.value > 1400) return reply({}, "value out of range");
        this.#sp = Math.round(req.value);
        return reply();
      case "start":
        if (busy) return reply({}, busy);
        if (this.#mode === "off") this.#pid = newPid();
        this.#mode = "run";
        return reply();
      case "stop":
        this.#mode = "off";
        this.#test = null;
        this.#duty = 0;
        return reply();
      case "program.get":
        return reply({ program: this.#program });
      case "program.run": {
        const pts = req.points;
        const ok = Array.isArray(pts) && pts.length >= 2 && pts.length <= 32 && pts[0][0] === 0 &&
          pts.every((p, i) => p[1] >= 0 && p[1] <= 1400 && (i === 0 || p[0] > pts[i - 1][0]));
        if (!ok) return reply({}, "bad points");
        const next = { name: String(req.name || "").slice(0, 32), wait: req.wait !== false, points: pts.map((p) => [...p]) };
        // Same rules as the firmware's runProgram: run from "at" seconds; if
        // omitted, keep a running program's time, else start at 0.
        if (req.at != null && typeof req.at !== "number") return reply({}, "at must be a number");
        const updated = this.#mode === "program";
        const start = req.at ?? (updated ? this.#el : 0);
        if (!(start >= 0 && start < pts[pts.length - 1][0] * 60)) return reply({}, "start time outside the program");
        if (!updated || req.at != null) this.#pid = newPid();
        this.#test = null;
        this.#program = next;
        this.#mode = "program";
        this.#el = start;
        return reply({ updated, program: this.#program });
      }
      case "tune.start": {
        if (this.#mode === "program") return reply({}, busy);
        const temp = req.temp, step = req.step ?? 0.2;
        if (typeof temp !== "number" || temp < 50 || temp > 1300) return reply({}, "temp out of range");
        if (!(step >= 0.05 && step <= 0.5)) return reply({}, "step must be 0.05..0.5");
        this.#test = new StepTest(temp, step, gainsAt(this.#tuning, temp, this.#base));
        this.#mode = "tune";
        this.#sp = temp;
        return reply();
      }
      case "tune.result":
        return reply({ result: this.#lastResult });
      case "tuning.get":
        return reply({ points: this.#tuning, base: this.#base, default_base: DEFAULT_GAINS });
      case "tuning.set": {
        const pts = req.points;
        if (!Array.isArray(pts) || pts.length > 4) return reply({}, "bad points");
        const sorted = [...pts].sort((a, b) => a.temp - b.temp);
        for (let i = 0; i < sorted.length; i++) {
          const p = sorted[i];
          if (["temp", "kp", "ki", "kd", "hold"].some((k) => typeof p[k] !== "number")) return reply({}, "bad number");
          if (p.kp < 0 || p.ki < 0 || p.kd < 0) return reply({}, "gains must not be negative");
          if (p.hold < 0 || p.hold > 1) return reply({}, "hold power must be 0..1");
          if (i > 0 && p.temp - sorted[i - 1].temp < 1) return reply({}, "temperatures must differ");
        }
        const b = req.base ?? this.#base;
        if (["kp", "ki", "kd"].some((k) => typeof b[k] !== "number")) return reply({}, "bad number");
        if (b.kp < 0 || b.ki < 0 || b.kd < 0) return reply({}, "gains must not be negative");
        this.#tuning = sorted.map((p) => ({ ...p }));
        this.#base = { kp: b.kp, ki: b.ki, kd: b.kd };
        return reply();
      }
      default:
        return reply({}, "unknown cmd");
    }
  }

  #tick() {
    const dt = SPEEDUP / SUBSTEPS;
    for (let i = 0; i < SUBSTEPS; i++) {
      if (this.#mode === "program") this.#programStep(dt);
      if (this.#mode === "tune") {
        this.#duty = this.#test.update(this.#tc, dt);
        if (!this.#test.active) this.#finishTest();
      } else if (this.#mode !== "off") {
        const ff = feedforwardAt(this.#tuning, this.#sp);
        this.#duty = pidUpdate(this.#pid, gainsAt(this.#tuning, this.#sp, this.#base), this.#sp, this.#tc, dt, ff);
      }
      const power = this.#mode !== "off" ? this.#duty : 0;
      this.#kiln += (MODEL.heatPerS * power - lossCPerS(this.#kiln)) * dt;
      this.#tc += ((this.#kiln - this.#tc) * dt) / (MODEL.sensorTau + dt);
      this.#simS += dt;
    }
    const pv = this.#tc + (Math.random() - 0.5) * 0.3;
    const status = {
      evt: "status",
      ms: Math.round(this.#simS * 1000),
      state: this.#mode,
      sim: true,
      pv: +pv.toFixed(2),
      sp: +this.#sp.toFixed(1),
      duty: +(this.#mode !== "off" ? this.#duty : 0).toFixed(3),
      ssr: this.#mode !== "off" && Math.random() < this.#duty ? 1 : 0,
      fault: "",
    };
    if (this.#mode === "program") {
      const pts = this.#program.points;
      const t = this.#el / 60;
      let seg = pts.findIndex((p) => t < p[0]);
      if (seg < 1) seg = pts.length - 1;
      status.prog = {
        name: this.#program.name, el: Math.round(this.#el), total: pts[pts.length - 1][0] * 60,
        seg, segs: pts.length - 1, wait: this.#waiting,
      };
    }
    if (this.#mode === "off" && this.#stopped) {
      const { at, ...x } = this.#stopped;
      status.stopped = { ...x, ago: Math.round(this.#simS - at) };
    } else if (this.#mode !== "off") {
      this.#stopped = null;
    }
    if (this.#mode === "tune") {
      const t = this.#test;
      status.tune = { phase: t.phase, temp: t.target, el: Math.round(t.total), phase_el: Math.round(t.phaseS) };
      if (t.phase === "settling") {
        Object.assign(status.tune, {
          steady: Math.round(t.inBand), need: SETTLE_S, band: SETTLE_BAND_C,
          drift: t.inBand > 0 ? +(t.lastPv - t.bandPv0).toFixed(2) : 0, max_drift: SETTLE_DRIFT_C,
        });
      } else if (t.phase === "stepping") {
        Object.assign(status.tune, { rise: +(t.lastPv - t.pv0).toFixed(1), max_rise: 50, max_s: 3600 });
      }
    }
    this.receive(JSON.stringify(status) + "\n");
  }

  #finishTest() {
    const t = this.#test;
    this.#mode = "off";
    this.#duty = 0;
    this.#test = null;
    if (t.phase === "done") {
      this.#lastResult = t.result;
      this.#event({ evt: "tune_done", result: t.result });
    } else {
      this.#event({ evt: "tune_failed", reason: t.reason || "failed" });
      this.#stopped = { reason: "STEP TEST FAILED", detail: t.reason || "failed", alarm: true, at: this.#simS };
      const { at, ...ev } = this.#stopped;
      this.#event({ evt: "stopped", ...ev });
    }
  }

  // Same rules as lib/kiln_core ProgramRunner: with "wait" on, the clock
  // pauses while the kiln is behind the profile.
  #programStep(dt) {
    const pts = this.#program.points;
    const total = pts[pts.length - 1][0] * 60;
    const spAt = (s) => {
      const t = s / 60;
      for (let i = 1; i < pts.length; i++) {
        const [am, ac] = pts[i - 1], [bm, bc] = pts[i];
        if (t <= bm) return ac + ((bc - ac) * (t - am)) / (bm - am);
      }
      return pts[pts.length - 1][1];
    };
    // Wait only when the kiln is behind: too cold on a rise or hold, too hot
    // on a cooling segment (direction judged a minute ahead).
    const sp = spAt(this.#el);
    const cooling = spAt(this.#el + 60) < sp;
    const behind = cooling ? this.#tc > sp + WAIT_BAND_C : this.#tc < sp - WAIT_BAND_C;
    this.#waiting = this.#program.wait && behind;
    if (!this.#waiting) this.#el += dt;
    if (this.#el >= total) {
      this.#mode = "off";
      this.#duty = 0;
      this.#event({ evt: "program_done" });
      return;
    }
    this.#sp = spAt(this.#el);
  }
}
