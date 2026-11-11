// Kiln controller: single setpoint or a stored ramp/hold program.
//
// - Reads the MAX31855 at 5 Hz, linearizes (NIST type K), median-filters.
// - Setpoint mode: turning the dial chooses 0..1400 C in 5 C clicks (bigger
//   steps when spun fast), applied once the dial is still for a moment.
//   A click starts/stops heating, a long press always stops.
// - Program mode: runs the program stored in flash (sent from the web app)
//   on its own, optionally waiting for the kiln if it falls behind (a
//   per-program setting). The dial is locked;
//   a long press stops. Progress is checkpointed so a reboot can resume.
// - A PID loop sets the SSR duty once per second. Its gains are interpolated
//   by setpoint from a tuning table measured by step tests, and the table's
//   holding power is added as a feed-forward (see lib/kiln_core/tuning.h).
// - Tune mode runs a step test (heat, hold steady, apply a power step) and
//   reports suggested gains; the web app stores the table.
// - Any sensor fault or over-temperature stops heating and sounds the buzzer.
// - One JSON status line per second on USB serial, and newline-delimited
//   JSON commands (see handleCommand and web/README.md).
//
// Build env m5dial-sim (-DKILN_SIMULATE) replaces the thermocouple with a
// simulated kiln and never drives the SSR pin.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "encoder.h"
#include "kiln_model.h"
#include "max31855.h"
#include "median_filter.h"
#include "pid.h"
#include "pins.h"
#include "program.h"
#include "serial_out.h"
#include "thermocouple.h"
#include "tp_output.h"
#include "tuning.h"
#include "ui.h"

// See platformio.ini: the USB-Serial/JTAG stack can be reset by the host in
// hardware; TinyUSB CDC lets firmware decide (Serial.enableReboot).
#if ARDUINO_USB_MODE != 0 || ARDUINO_USB_CDC_ON_BOOT != 1
#error "Build with TinyUSB CDC (ARDUINO_USB_MODE=0, ARDUINO_USB_CDC_ON_BOOT=1)"
#endif

#ifdef KILN_SIMULATE
constexpr bool kSimulated = true;
#else
constexpr bool kSimulated = false;
#endif

namespace {

constexpr const char* kFirmwareVersion = "0.8.0";
constexpr int kProtocolVersion = 1;
constexpr uint32_t kToastMs = 2500;

constexpr float kSetpointMinC = 0.0f;
constexpr float kSetpointMaxC = 1400.0f;
constexpr float kSetpointStepC = 5.0f;              // one dial click
constexpr float kMaxTempC = kSetpointMaxC + 25.0f;  // hard over-temperature cutoff
constexpr uint32_t kSetpointCommitMs = 1200;        // dial still this long -> apply
constexpr uint32_t kWindowMs = 4000;     // SSR time-proportioning window
constexpr uint32_t kMinPulseMs = 500;
constexpr uint32_t kControlPeriodMs = 1000;
constexpr uint32_t kReadPeriodMs = 200;  // MAX31855 converts in ~100 ms
constexpr int kFaultDebounce = 3;        // consecutive unusable frames -> fault shown
// While heating, a sensor fault only stops the kiln once there has been no
// usable reading for this long. Until then the output holds its last duty
// (hot elements leaking into the thermocouple, or SSR switching noise, can
// spoil readings for a while; see thermocouple.h).
constexpr uint32_t kSensorLostMs = 30000;
constexpr float kFlaggedJumpC = 50.0f;  // a flagged reading further off is discarded
constexpr uint32_t kUiPeriodMs = 50;
constexpr uint32_t kStatusPeriodMs = 1000;
constexpr uint32_t kHoldMs = 1500;
constexpr uint32_t kCheckpointMs = 60000;
// After a reboot mid-program, resume only if the kiln is still this close to
// where the program was; otherwise it was off too long and the firing is
// abandoned rather than restarted from a wrong point.
constexpr float kResumeBandC = 50.0f;
constexpr uint32_t kResumeDecideMs = 5000;  // wait this long for a reading
// If loop() stops for this long (any hang), the chip resets: the SSR pin
// drops low at reset and a running program resumes from its checkpoint.
constexpr int kWatchdogS = 5;
constexpr float kAmbientC = 20.0f;  // feed-forward is zero at ambient

enum class Mode { Off, Setpoint, Program, Tune };

Max31855 s_tc(kPinTcSck, kPinTcCs, kPinTcSo);
kiln::TimeProportioningOutput s_out(kWindowMs, kMinPulseMs);
kiln::MedianFilter<5> s_filter;
kiln::Pid s_pid;
kiln::KilnModel s_sim;
kiln::Program s_program{};  // stored program (layout 0 = none)
kiln::ProgramRunner s_runner;
kiln::TuningTable s_tuning = kiln::kEmptyTuning;  // no points, built-in baseline gains
kiln::StepTest s_test;
bool s_have_test_result = false;
Preferences s_prefs;

kiln::TcReading s_last{kiln::kTcFaultBus, NAN, NAN, NAN};
uint8_t s_fault = kiln::kTcFaultBus;  // debounced; start faulted until read
int s_bad_reads = 0;
uint32_t s_last_frame = 0;

float s_setpoint = 0.0f;  // setpoint-mode target
float s_pending = 0.0f;   // what the dial shows while it's being turned
bool s_editing = false;
uint32_t s_last_turn_ms = 0;

Mode s_mode = Mode::Off;
float s_control_sp = NAN;  // setpoint the PID is using right now
float s_duty = 0.0f;
bool s_ssr = false;
bool s_overtemp = false;
uint32_t s_last_good_ms = 0;  // last usable reading
uint8_t s_warn = 0;           // flags on the last reading that was used anyway
uint32_t s_warn_count = 0;    // such readings since heating started
// Why heating last stopped (shown until the next start). `s_alarm` also
// keeps it on the Dial in red until a click.
const char* s_stop_reason = nullptr;
char s_stop_detail[48] = "";
uint32_t s_stop_ms = 0;
bool s_alarm = false;
bool s_resume_pending = false;
float s_resume_elapsed_s = 0.0f;

const char* s_toast = nullptr;
uint32_t s_toast_until = 0;

void toast(const char* msg) {
  s_toast = msg;
  s_toast_until = millis() + kToastMs;
}

bool hasProgram() { return kiln::validateProgram(s_program, kSetpointMaxC) == nullptr; }

void setSsr(bool on) {
  s_ssr = on;
  if (!kSimulated) digitalWrite(kPinSsr, on ? HIGH : LOW);
}

void beep(uint16_t hz, uint32_t ms) { M5.Speaker.tone(hz, ms); }

void saveCheckpoint() {
  const bool running = s_mode == Mode::Program;
  s_prefs.putBool("ck_run", running);
  if (running) s_prefs.putFloat("ck_el", s_runner.elapsedS());
}

void send(JsonDocument& doc);

// `reason`: short, upper case (fits the Dial); `detail`: optional extra.
// An alarm stop (a fault) beeps and stays on the Dial until a click.
void stopHeating(bool alarm, const char* reason, const char* detail = nullptr) {
  if (s_mode == Mode::Off) return;
  const bool was_program = s_mode == Mode::Program;
  if (s_mode == Mode::Tune && s_test.active()) {
    // Stopped from outside the test: report it like any other failure.
    s_test.abort(reason);
    JsonDocument ev;
    ev["evt"] = "tune_failed";
    ev["reason"] = detail ? detail : reason;
    send(ev);
  }
  s_mode = Mode::Off;
  s_runner.stop();
  s_duty = 0.0f;
  s_control_sp = NAN;
  s_out.forceOff();
  setSsr(false);
  if (was_program) saveCheckpoint();
  s_stop_reason = reason;
  snprintf(s_stop_detail, sizeof(s_stop_detail), "%s", detail ? detail : "");
  s_stop_ms = millis();
  s_alarm = alarm;
  JsonDocument ev;
  ev["evt"] = "stopped";
  ev["reason"] = reason;
  if (detail) ev["detail"] = detail;
  ev["alarm"] = alarm;
  send(ev);
  if (alarm) beep(2000, 600);
}

// Clears the last stop reason; call when heating starts.
void startedHeating() {
  s_stop_reason = nullptr;
  s_alarm = false;
  s_warn_count = 0;
  s_warn = 0;
}

float temperature() { return s_filter.value(); }

const char* faultText() {
  if (s_fault != kiln::kTcFaultNone) return kiln::tcFaultName(s_fault);
  if (s_overtemp) return "OVER TEMP";
  return nullptr;
}

void readSensor() {
  if (kSimulated) {
    s_sim.step(s_ssr ? 1.0f : 0.0f, kReadPeriodMs / 1000.0f);
    s_last = {kiln::kTcFaultNone, s_sim.thermocouple(), s_sim.ambient(), s_sim.thermocouple()};
    s_last_frame = 0;
  } else {
    s_last_frame = s_tc.readFrame();
    s_last = kiln::decodeMax31855(s_last_frame);
  }
  // A reading flagged short-to-GND/VCC still has a temperature (see
  // thermocouple.h). Use it if it's plausible next to the current value;
  // the median filter rejects any remaining spikes.
  const float now_c = temperature();
  const bool usable = !std::isnan(s_last.temp_c) &&
                      (s_last.faults == kiln::kTcFaultNone || std::isnan(now_c) ||
                       std::fabs(s_last.temp_c - now_c) <= kFlaggedJumpC);
  if (usable) {
    s_bad_reads = 0;
    s_fault = kiln::kTcFaultNone;
    s_last_good_ms = millis();
    s_filter.push(s_last.temp_c);
    if (s_last.faults != kiln::kTcFaultNone) {
      s_warn = s_last.faults;
      ++s_warn_count;
    }
  } else if (++s_bad_reads >= kFaultDebounce) {
    s_fault = s_last.faults;
    // While heating, keep the last temperature through a dropout (see
    // kSensorLostMs); when off, show that there's no reading.
    if (s_mode == Mode::Off) s_filter.reset();
  }
  const float t = temperature();
  s_overtemp = !std::isnan(t) && t >= kMaxTempC;
}

// ---- Modes -------------------------------------------------------------------

// Each returns an error message, or nullptr on success.
const char* startSetpoint() {
  if (s_mode == Mode::Program) return "program running; stop it first";
  if (s_mode == Mode::Tune) return "step test running; stop it first";
  if (const char* fault = faultText()) {
    beep(300, 300);
    return fault;
  }
  if (s_mode != Mode::Setpoint) {
    s_mode = Mode::Setpoint;
    s_pid.reset();
    startedHeating();
    beep(1800, 80);
  }
  return nullptr;
}

const char* startProgram() {
  if (!hasProgram()) return "no program stored";
  if (const char* fault = faultText()) {
    beep(300, 300);
    return fault;
  }
  s_runner.start(s_program);
  s_control_sp = kiln::programSetpoint(s_program, s_runner.elapsedS());
  s_mode = Mode::Program;
  s_pid.reset();
  startedHeating();
  saveCheckpoint();
  beep(1800, 80);
  return nullptr;
}

void finishProgram() {
  stopHeating(false, "PROGRAM DONE");
  toast("PROGRAM DONE");
  for (int i = 0; i < 3; ++i) {
    beep(1500 + 300 * i, 120);
    delay(160);
  }
  constexpr char kDone[] = "{\"evt\":\"program_done\"}";
  serial_out::sendLine(kDone, sizeof(kDone) - 1);
}

// Step test: heat to `target_c`, hold it steady, then apply a power step.
const char* startTune(float target_c, float step_duty) {
  if (s_mode == Mode::Program) return "program running; stop it first";
  if (const char* fault = faultText()) return fault;
  kiln::StepTestConfig cfg;
  cfg.target_c = target_c;
  cfg.step_duty = step_duty;
  s_test.start(cfg, kiln::gainsAt(s_tuning, target_c));
  s_mode = Mode::Tune;
  s_control_sp = target_c;
  startedHeating();
  beep(1800, 80);
  return nullptr;
}

void gainsToJson(JsonObject o, const kiln::PidGains& g) {
  o["kp"] = g.kp;
  o["ki"] = g.ki;
  o["kd"] = g.kd;
}

void tuneResultToJson(JsonObject r) {
  const kiln::StepTestResult& res = s_test.result();
  r["temp"] = res.target_c;
  r["hold"] = res.hold_duty;
  r["step"] = res.step_duty;
  r["rate"] = res.rate_c_per_s;
  r["dead"] = res.dead_s;
  r["kp"] = res.gains.kp;
  r["ki"] = res.gains.ki;
  r["kd"] = res.gains.kd;
}

void finishTune() {
  const bool ok = s_test.phase() == kiln::StepTest::Phase::Done;
  const char* why = s_test.failure() ? s_test.failure() : "failed";
  // The kiln stays hot; heating stops.
  if (ok) stopHeating(false, "STEP TEST DONE"); else stopHeating(true, "STEP TEST FAILED", why);
  s_have_test_result = ok;
  JsonDocument ev;
  ev["evt"] = ok ? "tune_done" : "tune_failed";
  if (ok) {
    tuneResultToJson(ev["result"].to<JsonObject>());
    toast("STEP TEST DONE");
    for (int i = 0; i < 3; ++i) {
      beep(1500 + 300 * i, 120);
      delay(160);
    }
  } else {
    ev["reason"] = why;
  }
  send(ev);
}

// Called once a temperature is available after boot.
void decideResume() {
  s_resume_pending = false;
  const float pv = temperature();
  const float sp = kiln::programSetpoint(s_program, s_resume_elapsed_s);
  if (hasProgram() && !faultText() && fabsf(pv - sp) <= kResumeBandC) {
    s_runner.resume(s_program, s_resume_elapsed_s);
    s_control_sp = sp;
    s_mode = Mode::Program;
    s_pid.reset();
    startedHeating();
    toast("PROGRAM RESUMED");
    beep(1800, 80);
  } else {
    s_prefs.putBool("ck_run", false);
    toast("PROGRAM ABORTED");
    beep(2000, 600);
    constexpr char kAborted[] = "{\"evt\":\"program_aborted\",\"reason\":\"restarted too far from profile\"}";
    serial_out::sendLine(kAborted, sizeof(kAborted) - 1);
  }
}

void control() {
  const float pv = temperature();
  if (s_mode == Mode::Off) return;
  // No usable reading right now: hold the last duty (and pause the program
  // clock and step test) until readings return or kSensorLostMs passes.
  if (s_fault != kiln::kTcFaultNone) return;
  switch (s_mode) {
    case Mode::Off:
      return;
    case Mode::Setpoint:
      s_control_sp = s_setpoint;
      break;
    case Mode::Program:
      s_control_sp = s_runner.update(pv, kControlPeriodMs / 1000.0f);
      if (s_runner.finished()) {
        finishProgram();
        return;
      }
      break;
    case Mode::Tune:
      // The step test drives the output itself (its own hold PID, then the
      // fixed power step).
      s_duty = s_test.update(pv, kControlPeriodMs / 1000.0f);
      if (!s_test.active()) {
        finishTune();
        return;
      }
      s_out.setDuty(s_duty);
      return;
  }
  // Gain scheduling plus holding-power feed-forward from the tuning table.
  s_pid.setGains(kiln::gainsAt(s_tuning, s_control_sp));
  const float ff = kiln::feedforwardAt(s_tuning, s_control_sp, kAmbientC);
  s_duty = s_pid.update(s_control_sp, pv, kControlPeriodMs / 1000.0f, ff);
  s_out.setDuty(s_duty);
}

// ---- Dial and button -----------------------------------------------------------

// One click moves kSetpointStepC; spinning faster multiplies the step.
float setpointStep(int32_t detents, uint32_t since_last_ms) {
  const int32_t n = detents < 0 ? -detents : detents;
  if (n >= 4) return kSetpointStepC * 20;                     // 100 C
  if (n >= 2 || since_last_ms < 60) return kSetpointStepC * 5;  // 25 C
  return kSetpointStepC;
}

void commitSetpoint() {
  s_editing = false;
  if (s_pending == s_setpoint) return;
  s_setpoint = s_pending;
  s_prefs.putFloat("sp", s_setpoint);
  beep(2400, 30);
}

void handleInput(uint32_t now) {
  const int32_t detents = encoder::takeDetents();
  if (detents != 0 && (s_mode == Mode::Program || s_mode == Mode::Tune)) {
    toast(s_mode == Mode::Tune ? "STEP TEST RUNNING" : "PROGRAM RUNNING");
  } else if (detents != 0) {
    const float step = setpointStep(detents, now - s_last_turn_ms);
    float sp = s_pending + detents * step;
    sp = roundf(sp / step) * step;  // land on multiples of the step size
    s_pending = constrain(sp, kSetpointMinC, kSetpointMaxC);
    s_editing = true;
    s_last_turn_ms = now;
  } else if (s_editing && now - s_last_turn_ms >= kSetpointCommitMs) {
    commitSetpoint();
  }

  if (M5.BtnA.wasHold()) {
    if (s_mode != Mode::Off) beep(800, 150);
    stopHeating(false, "STOPPED ON DIAL");
    s_alarm = false;
  } else if (M5.BtnA.wasClicked()) {
    if (s_alarm) {
      s_alarm = false;  // first click only dismisses the alarm
    } else if (s_mode == Mode::Program || s_mode == Mode::Tune) {
      toast("HOLD TO STOP");  // a stray click must not end a firing or test
    } else if (s_editing) {
      commitSetpoint();  // a click while turning only confirms the setpoint
    } else if (s_mode == Mode::Setpoint) {
      stopHeating(false, "STOPPED ON DIAL");
      beep(800, 80);
    } else {
      startSetpoint();  // refuses (with a low beep) while faulted
    }
  }
}

// ---- Serial protocol ------------------------------------------------------------
// One JSON object per line in each direction. Requests carry an optional
// "id" that is echoed in the reply. See web/README.md for the full list.

// All output goes through serial_out, which never blocks the loop.
void send(JsonDocument& doc) {
  static char buf[2048];  // a full 32-point program is ~700 bytes
  const size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n > 0 && n < sizeof(buf) - 1) serial_out::sendLine(buf, n);
}

void reply(JsonVariantConst id, const char* error = nullptr, JsonDocument* extra = nullptr) {
  JsonDocument out;
  if (!id.isNull()) out["id"] = id;
  out["ok"] = error == nullptr;
  if (error) out["error"] = error;
  if (extra) {
    for (JsonPairConst kv : extra->as<JsonObjectConst>()) out[kv.key()] = kv.value();
  }
  send(out);
}

void programToJson(JsonDocument& doc) {
  if (!hasProgram()) {
    doc["program"] = nullptr;
    return;
  }
  JsonObject p = doc["program"].to<JsonObject>();
  p["name"] = s_program.name;
  p["wait"] = s_program.wait != 0;
  JsonArray pts = p["points"].to<JsonArray>();
  for (int i = 0; i < s_program.count; ++i) {
    JsonArray q = pts.add<JsonArray>();
    q.add(s_program.pts[i].min);
    q.add(s_program.pts[i].c);
  }
}

// {"name","wait","points":[[min,c],...]} -> Program. nullptr or a reason.
const char* parseProgram(JsonVariantConst req, kiln::Program& p) {
  JsonArrayConst pts = req["points"];
  if (pts.isNull() || pts.size() > static_cast<size_t>(kiln::kMaxProgramPoints)) return "bad points";
  p = {};
  p.layout = kiln::kProgramLayout;
  p.wait = (req["wait"] | true) ? 1 : 0;
  strlcpy(p.name, req["name"] | "", sizeof(p.name));
  for (JsonArrayConst q : pts) {
    if (q.size() != 2 || !q[0].is<float>() || !q[1].is<float>()) return "bad points";
    p.pts[p.count++] = {q[0].as<float>(), q[1].as<float>()};
  }
  return kiln::validateProgram(p, kSetpointMaxC);
}

void storeProgram(const kiln::Program& p) {
  s_program = p;
  s_prefs.putBytes("prog", &s_program, sizeof(s_program));
}

const char* putProgram(JsonVariantConst req) {
  if (s_mode == Mode::Program) return "program running; stop it first";
  kiln::Program p;
  if (const char* err = parseProgram(req, p)) return err;
  storeProgram(p);
  return nullptr;
}

// Store a program and run it from req["at"] (seconds of program time). If
// "at" is omitted: a running program keeps its current time with the new
// points swapped in; otherwise it starts at 0. Setpoint steps are allowed.
// Sets *updated when a program that was already running was changed.
const char* runProgram(JsonVariantConst req, bool* updated) {
  kiln::Program next;
  if (const char* err = parseProgram(req, next)) return err;
  JsonVariantConst at = req["at"];
  if (!at.isNull() && !at.is<float>()) return "at must be a number";
  *updated = s_mode == Mode::Program;
  const float start_s = !at.isNull() ? at.as<float>() : (*updated ? s_runner.elapsedS() : 0.0f);
  if (!(start_s >= 0.0f && start_s < kiln::programDurationS(next))) return "start time outside the program";
  if (const char* fault = faultText()) return fault;

  storeProgram(next);
  s_runner.resume(s_program, start_s);
  s_control_sp = kiln::programSetpoint(s_program, start_s);
  if (!*updated || !at.isNull()) s_pid.reset();  // fresh start or a jump
  if (!*updated) startedHeating();
  s_mode = Mode::Program;
  saveCheckpoint();  // so a reboot resumes from the new position
  beep(*updated ? 2400 : 1800, *updated ? 30 : 80);
  return nullptr;
}

void handleCommand(const char* line) {
  JsonDocument req;
  if (deserializeJson(req, line) != DeserializationError::Ok || !req.is<JsonObject>()) {
    reply(JsonVariantConst(), "bad json");
    return;
  }
  JsonVariantConst id = req["id"];
  const char* cmd = req["cmd"] | "";

  if (strcmp(cmd, "hello") == 0) {
    JsonDocument info;
    info["fw"] = kFirmwareVersion;
    info["proto"] = kProtocolVersion;
    info["sim"] = kSimulated;
    info["sp_min"] = kSetpointMinC;
    info["sp_max"] = kSetpointMaxC;
    info["max_points"] = kiln::kMaxProgramPoints;
    info["max_tune_points"] = kiln::kMaxTunePoints;
    reply(id, nullptr, &info);
  } else if (strcmp(cmd, "setpoint") == 0) {
    if (s_mode == Mode::Program) return reply(id, "program running; stop it first");
    JsonVariantConst v = req["value"];
    if (!v.is<float>()) return reply(id, "value must be a number");
    const float sp = v.as<float>();
    if (!(sp >= kSetpointMinC && sp <= kSetpointMaxC)) return reply(id, "value out of range");
    s_pending = roundf(sp);
    commitSetpoint();
    toast("SET BY PC");
    reply(id);
  } else if (strcmp(cmd, "start") == 0) {
    const char* err = startSetpoint();
    if (!err) toast("STARTED BY PC");
    reply(id, err);
  } else if (strcmp(cmd, "stop") == 0) {
    if (s_mode != Mode::Off) beep(800, 80);
    stopHeating(false, "STOPPED BY PC");
    s_alarm = false;
    toast("STOPPED BY PC");
    reply(id);
  } else if (strcmp(cmd, "program.put") == 0) {
    const char* err = putProgram(req.as<JsonVariantConst>());
    if (!err) toast("PROGRAM SAVED");
    reply(id, err);
  } else if (strcmp(cmd, "program.get") == 0) {
    JsonDocument out;
    programToJson(out);
    reply(id, nullptr, &out);
  } else if (strcmp(cmd, "program.start") == 0) {
    if (s_mode == Mode::Program) return reply(id, "program already running");
    const char* err = startProgram();
    if (!err) toast("PROGRAM STARTED");
    reply(id, err);
  } else if (strcmp(cmd, "tune.start") == 0) {
    JsonVariantConst t = req["temp"];
    if (!t.is<float>()) return reply(id, "temp must be a number");
    const float temp = t.as<float>();
    if (!(temp >= 50.0f && temp <= kSetpointMaxC - 100.0f)) return reply(id, "temp out of range");
    const float step = req["step"] | 0.2f;
    if (!(step >= 0.05f && step <= 0.5f)) return reply(id, "step must be 0.05..0.5");
    const char* err = startTune(temp, step);
    if (!err) toast("STEP TEST STARTED");
    reply(id, err);
  } else if (strcmp(cmd, "tune.result") == 0) {
    JsonDocument out;
    if (s_have_test_result) tuneResultToJson(out["result"].to<JsonObject>()); else out["result"] = nullptr;
    reply(id, nullptr, &out);
  } else if (strcmp(cmd, "tuning.get") == 0) {
    JsonDocument out;
    gainsToJson(out["base"].to<JsonObject>(), s_tuning.base);
    gainsToJson(out["default_base"].to<JsonObject>(), kiln::kDefaultGains);
    JsonArray pts = out["points"].to<JsonArray>();
    for (int i = 0; i < s_tuning.count; ++i) {
      const kiln::TunePoint& q = s_tuning.pts[i];
      JsonObject o = pts.add<JsonObject>();
      o["temp"] = q.temp_c;
      o["kp"] = q.kp;
      o["ki"] = q.ki;
      o["kd"] = q.kd;
      o["hold"] = q.hold_duty;
    }
    reply(id, nullptr, &out);
  } else if (strcmp(cmd, "tuning.set") == 0) {
    JsonArrayConst pts = req["points"];
    if (pts.isNull() || pts.size() > static_cast<size_t>(kiln::kMaxTunePoints)) return reply(id, "bad points");
    kiln::TuningTable t = kiln::kEmptyTuning;
    t.base = s_tuning.base;  // kept unless the request has "base"
    JsonObjectConst base = req["base"];
    if (!base.isNull()) t.base = {base["kp"] | NAN, base["ki"] | NAN, base["kd"] | NAN, 0.0f};
    for (JsonObjectConst o : pts) {
      t.pts[t.count++] = {o["temp"] | NAN, o["kp"] | NAN, o["ki"] | NAN, o["kd"] | NAN, o["hold"] | NAN};
    }
    if (const char* err = kiln::validateTuning(t)) return reply(id, err);
    s_tuning = t;
    s_prefs.putBytes("tune", &s_tuning, sizeof(s_tuning));
    toast("TUNING SAVED");
    reply(id);
  } else if (strcmp(cmd, "program.run") == 0) {
    bool updated = false;
    const char* err = runProgram(req.as<JsonVariantConst>(), &updated);
    if (err) return reply(id, err);
    toast(updated ? "PROGRAM UPDATED" : "PROGRAM STARTED");
    JsonDocument out;
    out["updated"] = updated;
    programToJson(out);  // the program now stored and running
    reply(id, nullptr, &out);
  } else {
    reply(id, "unknown cmd");
  }
}

void pollSerial() {
  static char line[1536];  // a full 32-point program fits comfortably
  static size_t len = 0;
  static bool overflow = false;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (len > 0 && !overflow) {
        line[len] = '\0';
        handleCommand(line);
      } else if (overflow) {
        reply(JsonVariantConst(), "line too long");
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}

const char* modeName() {
  switch (s_mode) {
    case Mode::Setpoint: return "run";
    case Mode::Program: return "program";
    case Mode::Tune: return "tune";
    default: return "off";
  }
}

// Setpoint to show: the program's while one runs, else the dial's.
float displaySetpoint() {
  return s_mode == Mode::Program || s_mode == Mode::Tune ? s_control_sp : s_setpoint;
}

void printStatus(uint32_t now) {
  JsonDocument st;
  st["evt"] = "status";
  st["ms"] = now;
  st["state"] = modeName();
  st["sim"] = kSimulated;
  const float pv = temperature();
  if (std::isnan(pv)) st["pv"] = nullptr; else st["pv"] = serialized(String(pv, 2));
  const float sp = displaySetpoint();
  if (std::isnan(sp)) st["sp"] = nullptr; else st["sp"] = serialized(String(sp, 1));
  st["duty"] = serialized(String(s_mode == Mode::Off ? 0.0f : s_duty, 3));
  st["ssr"] = s_ssr ? 1 : 0;
  const char* fault = faultText();
  st["fault"] = fault ? fault : "";
  if (s_warn_count > 0) {
    JsonObject w = st["warn"].to<JsonObject>();
    w["what"] = kiln::tcFaultName(s_warn);
    w["n"] = s_warn_count;
  }
  if (s_mode == Mode::Off && s_stop_reason) {
    JsonObject x = st["stopped"].to<JsonObject>();
    x["reason"] = s_stop_reason;
    if (s_stop_detail[0]) x["detail"] = s_stop_detail;
    x["alarm"] = s_alarm;
    x["ago"] = (now - s_stop_ms) / 1000;
  }
  if (s_mode == Mode::Tune) {
    JsonObject t = st["tune"].to<JsonObject>();
    t["phase"] = kiln::StepTest::phaseName(s_test.phase());
    t["temp"] = s_test.config().target_c;
    t["el"] = static_cast<uint32_t>(s_test.elapsedS());
    t["phase_el"] = static_cast<uint32_t>(s_test.phaseElapsedS());
    const kiln::StepTestConfig& c = s_test.config();
    if (s_test.phase() == kiln::StepTest::Phase::Settling) {
      t["steady"] = static_cast<uint32_t>(s_test.steadyS());
      t["need"] = static_cast<uint32_t>(c.settle_s);
      t["band"] = c.settle_band_c;
      t["drift"] = serialized(String(s_test.driftC(), 2));
      t["max_drift"] = c.settle_drift_c;
    } else if (s_test.phase() == kiln::StepTest::Phase::Stepping) {
      t["rise"] = serialized(String(s_test.riseC(), 1));
      t["max_rise"] = c.max_rise_c;
      t["max_s"] = static_cast<uint32_t>(c.max_step_s);
    }
  }
  if (s_mode == Mode::Program) {
    JsonObject p = st["prog"].to<JsonObject>();
    p["name"] = s_program.name;
    p["el"] = static_cast<uint32_t>(s_runner.elapsedS());
    p["total"] = static_cast<uint32_t>(s_runner.durationS());
    p["seg"] = s_runner.segment();
    p["segs"] = s_program.count - 1;
    p["wait"] = s_runner.waiting();
  }
  if (std::isnan(s_last.raw_c)) st["raw"] = nullptr; else st["raw"] = serialized(String(s_last.raw_c, 2));
  if (std::isnan(s_last.cj_c)) st["cj"] = nullptr; else st["cj"] = serialized(String(s_last.cj_c, 3));
  char frame[9];
  snprintf(frame, sizeof(frame), "%08lX", static_cast<unsigned long>(s_last_frame));
  st["frame"] = frame;
  send(st);
}

}  // namespace

void setup() {
  // Heater off before anything else touches the hardware.
  pinMode(kPinSsr, OUTPUT);
  digitalWrite(kPinSsr, LOW);

  auto cfg = M5.config();
  M5.begin(cfg);  // also drives the power-hold pin (G46)
  setSsr(false);
  M5.BtnA.setHoldThresh(kHoldMs);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // (doesn't stop write() spinning; see serial_out.h)
  Serial.enableReboot(true);  // disabled while heating, see loop()

  s_prefs.begin("kiln");
  s_setpoint = constrain(s_prefs.getFloat("sp", 0.0f), kSetpointMinC, kSetpointMaxC);
  s_pending = s_setpoint;
  if (s_prefs.getBytes("tune", &s_tuning, sizeof(s_tuning)) != sizeof(s_tuning) ||
      kiln::validateTuning(s_tuning) != nullptr) {
    s_tuning = kiln::kEmptyTuning;
  }
  if (s_prefs.getBytes("prog", &s_program, sizeof(s_program)) != sizeof(s_program) || !hasProgram()) {
    s_program = {};
  }
  if (s_prefs.getBool("ck_run", false)) {
    s_resume_pending = true;
    s_resume_elapsed_s = s_prefs.getFloat("ck_el", 0.0f);
  }

  esp_task_wdt_init(kWatchdogS, true);  // true: reset the chip, don't just log
  esp_task_wdt_add(nullptr);            // watch this (the loop) task
  const esp_reset_reason_t why = esp_reset_reason();
  if (why == ESP_RST_TASK_WDT || why == ESP_RST_INT_WDT || why == ESP_RST_PANIC) {
    toast("RECOVERED FROM FAULT");  // a resumed program will override this
  }

  if (!kSimulated) s_tc.begin();
  encoder::begin(kPinEncA, kPinEncB);
  ui::begin();
  beep(1500, 60);
}

void loop() {
  static uint32_t next_read = 0, next_control = 0, next_ui = 0, next_status = 0;
  static uint32_t next_checkpoint = 0;
  const uint32_t now = millis();
  esp_task_wdt_reset();

  M5.update();
  handleInput(now);
  pollSerial();

  // The CDC stack reboots into the bootloader on a 1200-baud touch or the
  // esptool DTR/RTS sequence. Allow that only while not heating, so uploads
  // still auto-reset but a host can never reboot us mid-firing.
  Serial.enableReboot(s_mode == Mode::Off && !s_resume_pending);

  if (static_cast<int32_t>(now - next_read) >= 0) {
    next_read = now + kReadPeriodMs;
    readSensor();
    if (s_mode != Mode::Off) {
      if (s_overtemp) {
        stopHeating(true, "OVER TEMP");
      } else if (s_fault != kiln::kTcFaultNone && millis() - s_last_good_ms >= kSensorLostMs) {
        char why[48];
        snprintf(why, sizeof(why), "no usable reading for %u s (%s)", static_cast<unsigned>(kSensorLostMs / 1000),
                 kiln::tcFaultName(s_fault));
        stopHeating(true, "NO TEMP READING", why);
        s_filter.reset();
      }
    }
  }

  if (s_resume_pending && (!std::isnan(temperature()) || now > kResumeDecideMs)) decideResume();

  if (static_cast<int32_t>(now - next_control) >= 0) {
    next_control = now + kControlPeriodMs;
    control();
  }

  if (s_mode == Mode::Program && static_cast<int32_t>(now - next_checkpoint) >= 0) {
    next_checkpoint = now + kCheckpointMs;
    saveCheckpoint();
  }

  const bool window_on = s_out.update(now);  // keep windows running while stopped
  setSsr(s_mode != Mode::Off && window_on);

  if (static_cast<int32_t>(now - next_ui) >= 0) {
    next_ui = now + kUiPeriodMs;
    UiState st{};
    st.temp_c = temperature();
    st.setpoint_c = s_editing ? s_pending : displaySetpoint();
    st.setpoint_max_c = kSetpointMaxC;
    st.editing = s_editing;
    st.running = s_mode != Mode::Off;
    st.program = s_mode == Mode::Program;
    st.waiting = s_runner.waiting();
    st.prog_seg = s_runner.segment();
    st.prog_segs = s_program.count - 1;
    st.prog_left_s = s_runner.durationS() - s_runner.elapsedS();
    st.duty = s_mode == Mode::Off ? 0.0f : s_duty;
    st.ssr_on = s_ssr;
    st.fault = faultText() ? faultText() : (s_alarm ? s_stop_reason : nullptr);
    st.simulated = kSimulated;
    st.toast = static_cast<int32_t>(now - s_toast_until) < 0 ? s_toast : nullptr;
    static char detail[40];
    st.tuning = s_mode == Mode::Tune;
    if (st.tuning) {
      const int t = static_cast<int>(s_test.phaseElapsedS());
      if (s_test.phase() == kiln::StepTest::Phase::Settling) {
        const int st = static_cast<int>(s_test.steadyS());
        const int need = static_cast<int>(s_test.config().settle_s);
        snprintf(detail, sizeof(detail), "steady %d:%02d/%d:%02d", st / 60, st % 60, need / 60, need % 60);
      } else if (s_test.phase() == kiln::StepTest::Phase::Stepping) {
        snprintf(detail, sizeof(detail), "step +%.0fC %d:%02d", s_test.riseC(), t / 60, t % 60);
      } else {
        snprintf(detail, sizeof(detail), "%s %d:%02d", kiln::StepTest::phaseName(s_test.phase()), t / 60, t % 60);
      }
      st.detail = detail;
    }
    ui::draw(st);
  }

  if (static_cast<int32_t>(now - next_status) >= 0) {
    next_status = now + kStatusPeriodMs;
    printStatus(now);
  }

  serial_out::pump();
}
