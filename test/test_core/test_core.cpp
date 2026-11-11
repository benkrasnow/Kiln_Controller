// Unit tests for lib/kiln_core. Run on the host with `pio test -e native`
// (needs g++), or on the Dial with `pio test -e m5dial`.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "kiln_model.h"
#include "median_filter.h"
#include "pid.h"
#include "program.h"
#include "thermocouple.h"
#include "tp_output.h"
#include "tuning.h"

using namespace kiln;

// KilnModel's default plant is a mid-size kiln, slower than the QuickFire the
// firmware defaults are sized for; these are the gains that suit it.
constexpr PidGains kMidSizeGains{0.03f, 0.00005f, 2.0f, 20.0f};

// A small, fast kiln like the Paragon QuickFire: ~1.5 C/s at full power,
// ~4% power to hold 150 C, tops out near 1150 C.
KilnModel::Params quickFire() {
  KilnModel::Params p;
  p.heat_c_per_s = 1.5f;
  p.k_cond = 2.6e-4f;
  p.k_rad = 2.9e-13f;
  p.sensor_tau_s = 15.0f;
  return p;
}

void setUp() {}
void tearDown() {}

// ---- NIST type K reference function -------------------------------------

void test_typeK_forward_matches_nist_table() {
  // ITS-90 type K table values, mV.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.554f, typeKMillivolts(-100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.000f, typeKMillivolts(0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.000f, typeKMillivolts(25));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.096f, typeKMillivolts(100));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.644f, typeKMillivolts(500));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 41.276f, typeKMillivolts(1000));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 48.838f, typeKMillivolts(1200));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 52.410f, typeKMillivolts(1300));
}

void test_typeK_inverse_round_trips() {
  // NIST inverse polynomials are good to about +/-0.06 C.
  for (int t = -200; t <= 1370; t += 10) {
    TEST_ASSERT_FLOAT_WITHIN(0.07f, t, typeKCelsius(typeKMillivolts(t)));
  }
}

void test_typeK_inverse_rejects_out_of_range() {
  TEST_ASSERT_TRUE(std::isnan(typeKCelsius(-6.0)));
  TEST_ASSERT_TRUE(std::isnan(typeKCelsius(55.0)));
}

// ---- MAX31855 frame decoding --------------------------------------------

// Build a frame the way the chip would for a given true temperature.
uint32_t frameFor(double true_c, double cj_c) {
  const double v = typeKMillivolts(true_c) - typeKMillivolts(cj_c);
  const double raw = cj_c + v / 0.041276;
  const int32_t tc = static_cast<int32_t>(std::lround(raw / 0.25));
  const int32_t cj = static_cast<int32_t>(std::lround(cj_c / 0.0625));
  return (static_cast<uint32_t>(tc & 0x3FFF) << 18) |
         (static_cast<uint32_t>(cj & 0x0FFF) << 4);
}

void test_decode_known_frames() {
  // +100.75 C TC (0x193 << 18), +25 C internal (0x190 << 4).
  TcReading r = decodeMax31855(0x064C1900);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultNone, r.faults);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.75f, r.raw_c);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, r.cj_c);

  // Negative values from the datasheet tables: -250 C TC (0x3C18),
  // -55 C internal (0xC90).
  r = decodeMax31855((0x3C18u << 18) | (0xC90u << 4));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -250.0f, r.raw_c);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -55.0f, r.cj_c);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultRange, r.faults);  // below NIST table
}

void test_decode_linearizes_high_temperatures() {
  // The chip's linear approximation is ~16 C low at 1200 C; we correct it.
  const double cases[] = {20, 100, 300, 600, 900, 1000, 1200, 1300};
  for (double t : cases) {
    const TcReading r = decodeMax31855(frameFor(t, 25.0));
    TEST_ASSERT_EQUAL_UINT8(kTcFaultNone, r.faults);
    TEST_ASSERT_FLOAT_WITHIN(0.35f, static_cast<float>(t), r.temp_c);
  }
  const TcReading r = decodeMax31855(frameFor(1200, 25.0));
  TEST_ASSERT_TRUE(r.temp_c - r.raw_c > 10.0f);
}

void test_decode_faults() {
  const uint32_t base = frameFor(25, 25) | (1u << 16);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultOpen, decodeMax31855(base | 0x1).faults);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultShortGnd, decodeMax31855(base | 0x2).faults);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultShortVcc, decodeMax31855(base | 0x4).faults);
  TEST_ASSERT_TRUE(std::isnan(decodeMax31855(base | 0x1).temp_c));
  // Leakage flags keep the measurement.
  const TcReading leak = decodeMax31855(frameFor(1150, 25.0) | (1u << 16) | 0x2);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultShortGnd, leak.faults);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1150.0f, leak.temp_c);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, decodeMax31855(base | 0x1).cj_c);
}

void test_decode_bus_faults() {
  TEST_ASSERT_EQUAL_UINT8(kTcFaultBus, decodeMax31855(0x00000000).faults);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultBus, decodeMax31855(0xFFFFFFFF).faults);
  // Reserved bits D17 / D3 set.
  TEST_ASSERT_EQUAL_UINT8(kTcFaultBus, decodeMax31855(frameFor(25, 25) | (1u << 17)).faults);
  TEST_ASSERT_EQUAL_UINT8(kTcFaultBus, decodeMax31855(frameFor(25, 25) | (1u << 3)).faults);
}

// ---- Time-proportioning output ------------------------------------------

// Run one window in 10 ms steps; return ON time and count state changes.
uint32_t runWindow(TimeProportioningOutput& out, uint32_t& t, int& edges, bool& last) {
  uint32_t on_ms = 0;
  for (uint32_t i = 0; i < out.windowMs(); i += 10, t += 10) {
    const bool s = out.update(t);
    if (s != last) ++edges;
    last = s;
    if (s) on_ms += 10;
  }
  return on_ms;
}

void test_tp_half_duty() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(0.5f);
  uint32_t t = 0;
  int edges = 0;
  bool last = false;
  for (int w = 0; w < 5; ++w) {
    TEST_ASSERT_EQUAL_UINT32(2000, runWindow(out, t, edges, last));
  }
  TEST_ASSERT_EQUAL_INT(10, edges);  // one on + one off per window
}

void test_tp_small_duty_carries_forward() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(0.05f);  // 200 ms, below the 500 ms minimum pulse
  uint32_t t = 0, total = 0;
  int edges = 0;
  bool last = false;
  for (int w = 0; w < 30; ++w) {
    const uint32_t on = runWindow(out, t, edges, last);
    TEST_ASSERT_TRUE(on == 0 || on >= 500);
    total += on;
  }
  TEST_ASSERT_UINT32_WITHIN(500, 30 * 200, total);
}

void test_tp_high_duty_carries_forward() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(0.95f);  // 200 ms off, below the minimum
  uint32_t t = 0, total = 0;
  int edges = 0;
  bool last = false;
  for (int w = 0; w < 30; ++w) {
    const uint32_t on = runWindow(out, t, edges, last);
    TEST_ASSERT_TRUE(on == 4000 || on <= 3500);
    total += on;
  }
  TEST_ASSERT_UINT32_WITHIN(500, 30 * 3800, total);
}

void test_tp_full_and_zero_never_switch() {
  TimeProportioningOutput out(4000, 500);
  uint32_t t = 0;
  int edges = 0;
  bool last = false;
  out.setDuty(1.0f);
  out.update(t);
  last = true;
  for (int w = 0; w < 5; ++w) TEST_ASSERT_EQUAL_UINT32(4000, runWindow(out, t, edges, last));
  TEST_ASSERT_EQUAL_INT(0, edges);

  out.setDuty(0.0f);
  runWindow(out, t, edges, last);  // the single on->off edge happens here
  edges = 0;
  for (int w = 0; w < 5; ++w) TEST_ASSERT_EQUAL_UINT32(0, runWindow(out, t, edges, last));
  TEST_ASSERT_EQUAL_INT(0, edges);
}

void test_tp_force_off_is_immediate() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(1.0f);
  TEST_ASSERT_TRUE(out.update(0));
  out.forceOff();
  TEST_ASSERT_FALSE(out.update(10));
  TEST_ASSERT_FALSE(out.update(5000));
}

void test_tp_rejects_bad_duty() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(NAN);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requestedDuty());
  out.setDuty(-1.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requestedDuty());
  out.setDuty(7.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.requestedDuty());
}

void test_tp_survives_millis_wraparound() {
  TimeProportioningOutput out(4000, 500);
  out.setDuty(0.5f);
  uint32_t t = 0xFFFFFFFFu - 6000;
  int edges = 0;
  bool last = false;
  for (int w = 0; w < 4; ++w) {
    TEST_ASSERT_EQUAL_UINT32(2000, runWindow(out, t, edges, last));
  }
}

// ---- Median filter -------------------------------------------------------

void test_median_rejects_spike() {
  MedianFilter<5> f;
  TEST_ASSERT_TRUE(std::isnan(f.value()));
  f.push(100);
  f.push(101);
  f.push(900);  // spike
  f.push(102);
  TEST_ASSERT_EQUAL_FLOAT(102.0f, f.value());  // upper median of 4
  f.push(103);
  TEST_ASSERT_EQUAL_FLOAT(102.0f, f.value());
}

// ---- PID -----------------------------------------------------------------

void test_pid_output_limits_and_bad_input() {
  Pid pid;
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(1000, 20, 1));
  pid.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(20, 1000, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(NAN, 20, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(500, NAN, 1));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(500, 20, 0));
}

void test_pid_no_windup_while_saturated() {
  Pid pid;
  for (int i = 0; i < 3600; ++i) pid.update(1000, 20, 1);  // an hour pinned at 100%
  TEST_ASSERT_TRUE(pid.i() < 0.05f);
}

void test_pid_no_derivative_kick_on_setpoint_change() {
  Pid pid;
  for (int i = 0; i < 100; ++i) pid.update(500, 500, 1);
  pid.update(800, 500, 1);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.d());
}

// Drive KilnModel through the real SSR window logic at 10 Hz.
struct LoopResult {
  float overshoot;
  float minutes_to_band;  // first time within 5 C, -1 if never
  float final_error;
  int max_edges_per_window;
};

LoopResult closedLoop(float start_c, float sp, float hours) {
  KilnModel m;
  while (m.thermocouple() < start_c) m.step(1.0f, 1.0f);
  Pid pid(kMidSizeGains);
  TimeProportioningOutput out(4000, 500);
  LoopResult r{-1e9f, -1.0f, 0.0f, 0};
  bool last = false;
  int edges = 0;
  uint32_t ms = 0;
  for (int s = 0; s < hours * 36000; ++s, ms += 100) {
    if (s % 10 == 0) out.setDuty(pid.update(sp, m.thermocouple(), 1.0f));
    if (s % 40 == 0) {
      if (edges > r.max_edges_per_window) r.max_edges_per_window = edges;
      edges = 0;
    }
    const bool on = out.update(ms);
    if (on != last) ++edges;
    last = on;
    m.step(on ? 1.0f : 0.0f, 0.1f);
    const float pv = m.thermocouple();
    if (pv - sp > r.overshoot) r.overshoot = pv - sp;
    if (r.minutes_to_band < 0 && std::fabs(pv - sp) < 5) r.minutes_to_band = s / 600.0f;
  }
  r.final_error = m.thermocouple() - sp;
  return r;
}

void test_closed_loop_low_setpoint() {
  const LoopResult r = closedLoop(22, 300, 3);
  TEST_ASSERT_TRUE(r.overshoot < 10);
  TEST_ASSERT_TRUE(r.minutes_to_band > 0 && r.minutes_to_band < 30);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, r.final_error);
  TEST_ASSERT_TRUE(r.max_edges_per_window <= 2);
}

void test_closed_loop_high_setpoint() {
  const LoopResult r = closedLoop(600, 1250, 4);
  TEST_ASSERT_TRUE(r.overshoot < 10);
  TEST_ASSERT_TRUE(r.minutes_to_band > 0 && r.minutes_to_band < 120);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, r.final_error);
  TEST_ASSERT_TRUE(r.max_edges_per_window <= 2);
}

// ---- Programs --------------------------------------------------------------

Program makeProgram(std::initializer_list<ProgramPoint> pts) {
  Program p{};
  p.layout = kProgramLayout;
  p.wait = 1;
  for (const ProgramPoint& q : pts) p.pts[p.count++] = q;
  return p;
}

// 20 -> 200 C over 3 h, hold 1 h, -> 1000 C by 8 h, hold 15 min.
const Program kBisque =
    makeProgram({{0, 20}, {180, 200}, {240, 200}, {480, 1000}, {495, 1000}});

void test_program_validation() {
  TEST_ASSERT_NULL(validateProgram(kBisque, 1400));
  TEST_ASSERT_NOT_NULL(validateProgram(makeProgram({{0, 20}}), 1400));
  TEST_ASSERT_NOT_NULL(validateProgram(makeProgram({{5, 20}, {60, 100}}), 1400));
  TEST_ASSERT_NOT_NULL(validateProgram(makeProgram({{0, 20}, {60, 100}, {60, 200}}), 1400));
  TEST_ASSERT_NOT_NULL(validateProgram(makeProgram({{0, 20}, {60, 1500}}), 1400));
  TEST_ASSERT_NOT_NULL(validateProgram(makeProgram({{0, 20}, {60, NAN}}), 1400));
  Program bad = kBisque;
  bad.layout = 99;
  TEST_ASSERT_NOT_NULL(validateProgram(bad, 1400));
}

void test_program_setpoint_interpolates() {
  TEST_ASSERT_EQUAL_FLOAT(20, programSetpoint(kBisque, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 110, programSetpoint(kBisque, 90 * 60));
  TEST_ASSERT_EQUAL_FLOAT(200, programSetpoint(kBisque, 200 * 60));  // hold
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 600, programSetpoint(kBisque, 360 * 60));
  TEST_ASSERT_EQUAL_FLOAT(1000, programSetpoint(kBisque, 99999 * 60));
  TEST_ASSERT_EQUAL_FLOAT(495 * 60, programDurationS(kBisque));
  TEST_ASSERT_EQUAL_INT(1, programSegment(kBisque, 0));
  TEST_ASSERT_EQUAL_INT(2, programSegment(kBisque, 200 * 60));
  TEST_ASSERT_EQUAL_INT(4, programSegment(kBisque, 490 * 60));
}

void test_runner_starts_at_zero_even_when_warm() {
  // A kiln still warm from a stopped firing: the program starts at t = 0
  // and the clock runs, because the kiln is ahead of the rising profile.
  ProgramRunner r;
  r.start(kBisque);
  TEST_ASSERT_EQUAL_FLOAT(0, r.elapsedS());
  TEST_ASSERT_EQUAL_FLOAT(20, r.update(150, 0.0f));
  r.update(150, 600);
  TEST_ASSERT_FALSE(r.waiting());
  TEST_ASSERT_EQUAL_FLOAT(600, r.elapsedS());
}

void test_runner_waits_only_when_behind() {
  // Cooling 300 -> 100 C over 60 min, then a hold.
  const Program cool = makeProgram({{0, 300}, {60, 100}, {90, 100}});
  ProgramRunner r;
  r.start(cool);
  r.update(300, 1800);  // halfway down: profile 200 C
  r.update(260, 60);    // kiln 60 C too hot on a cooling segment: wait
  TEST_ASSERT_TRUE(r.waiting());
  r.update(150, 60);    // kiln colder than a cooling profile is ahead: run
  TEST_ASSERT_FALSE(r.waiting());
  // In the hold, a kiln above the profile is ahead (no wait), below is behind.
  r.resume(cool, 70 * 60);
  r.update(140, 60);
  TEST_ASSERT_FALSE(r.waiting());
  r.update(70, 60);
  TEST_ASSERT_TRUE(r.waiting());
}

void test_runner_waits_for_lagging_kiln() {
  ProgramRunner r;
  r.start(kBisque);
  r.update(20, 1200);  // on profile: clock advances 20 min
  TEST_ASSERT_EQUAL_FLOAT(1200, r.elapsedS());
  TEST_ASSERT_FALSE(r.waiting());
  r.update(20, 600);  // profile now at 40 C, kiln still 20: 20 C behind > band
  TEST_ASSERT_TRUE(r.waiting());
  TEST_ASSERT_EQUAL_FLOAT(1200, r.elapsedS());
  r.update(NAN, 600);  // no reading: never advance
  TEST_ASSERT_EQUAL_FLOAT(1200, r.elapsedS());
  r.update(38, 600);  // caught up to within the band: advances again
  TEST_ASSERT_FALSE(r.waiting());
  TEST_ASSERT_EQUAL_FLOAT(1800, r.elapsedS());
}

void test_runner_without_wait_follows_clock() {
  Program p = kBisque;
  p.wait = 0;
  ProgramRunner r;
  r.start(p);
  r.update(20, 1200);
  r.update(20, 600);  // 20 C behind the profile: would wait, but waiting is off
  TEST_ASSERT_FALSE(r.waiting());
  TEST_ASSERT_EQUAL_FLOAT(1800, r.elapsedS());
  for (int i = 0; i < 1000 && r.active(); ++i) r.update(20, 60);  // kiln never heats
  TEST_ASSERT_TRUE(r.finished());
}

void test_runner_finishes() {
  ProgramRunner r;
  r.start(kBisque);
  for (int i = 0; i < 10000 && r.active(); ++i) {
    r.update(programSetpoint(kBisque, r.elapsedS()), 60);  // perfect kiln
  }
  TEST_ASSERT_TRUE(r.finished());
  TEST_ASSERT_FALSE(r.active());
  TEST_ASSERT_EQUAL_FLOAT(495 * 60, r.elapsedS());
}

void test_program_closed_loop_tracks_profile() {
  KilnModel m;
  Pid pid(kMidSizeGains);
  ProgramRunner r;
  TimeProportioningOutput out(4000, 500);
  r.start(kBisque);
  float worst = 0, sim_s = 0;
  uint32_t ms = 0;
  for (int s = 0; r.active() && s < 20 * 36000; ++s, ms += 100) {
    if (s % 10 == 0) {
      const float pv = m.thermocouple();
      const float sp = r.update(pv, 1.0f);
      worst = std::fmax(worst, std::fabs(pv - sp));
      out.setDuty(pid.update(sp, pv, 1.0f));
    }
    m.step(out.update(ms) ? 1.0f : 0.0f, 0.1f);
    sim_s += 0.1f;
  }
  TEST_ASSERT_TRUE(r.finished());
  TEST_ASSERT_TRUE(worst < ProgramRunner::kWaitBandC + 5);
  // Little waiting needed: done within 10% of the nominal 8 h 15 min.
  TEST_ASSERT_TRUE(sim_s < 495 * 60 * 1.10f);
}

// ---- Tuning: schedule, feed-forward, step test ---------------------------

TuningTable twoPointTable() {
  TuningTable t = kEmptyTuning;
  t.count = 2;
  t.pts[0] = {750, 0.02f, 0.00002f, 0.0f, 0.30f};  // deliberately out of order
  t.pts[1] = {150, 0.05f, 0.00010f, 3.0f, 0.04f};
  return t;
}

void test_tuning_validation_sorts_and_rejects() {
  TuningTable t = twoPointTable();
  TEST_ASSERT_NULL(validateTuning(t));
  TEST_ASSERT_EQUAL_FLOAT(150, t.pts[0].temp_c);  // sorted
  TuningTable bad = twoPointTable();
  bad.pts[0].kp = -1;
  TEST_ASSERT_NOT_NULL(validateTuning(bad));
  bad = twoPointTable();
  bad.pts[1].hold_duty = 1.5f;
  TEST_ASSERT_NOT_NULL(validateTuning(bad));
  bad = twoPointTable();
  bad.pts[1].temp_c = 750;
  TEST_ASSERT_NOT_NULL(validateTuning(bad));  // same temperature twice
  bad = twoPointTable();
  bad.base.ki = -1;
  TEST_ASSERT_NOT_NULL(validateTuning(bad));  // baseline gains checked too
}

void test_gains_interpolate_by_setpoint() {
  TuningTable t = twoPointTable();
  validateTuning(t);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, gainsAt(t, 20).kp);    // below: first point
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.035f, gainsAt(t, 450).kp);  // halfway
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.5f, gainsAt(t, 450).kd);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, gainsAt(t, 1200).kp);  // above: last point
  TuningTable empty = kEmptyTuning;
  TEST_ASSERT_EQUAL_FLOAT(kDefaultGains.kp, gainsAt(empty, 500).kp);
}

void test_baseline_gains_apply_without_points() {
  TuningTable t = kEmptyTuning;
  t.base = {0.05f, 0.001f, 0.5f, 99.0f};
  TEST_ASSERT_NULL(validateTuning(t));
  const PidGains g = gainsAt(t, 300);
  TEST_ASSERT_EQUAL_FLOAT(0.05f, g.kp);
  TEST_ASSERT_EQUAL_FLOAT(0.001f, g.ki);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, g.kd);
  TEST_ASSERT_EQUAL_FLOAT(kDefaultGains.d_tau_s, g.d_tau_s);  // filter isn't user-set
  t = twoPointTable();
  t.base.kp = 1.0f;
  validateTuning(t);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.035f, gainsAt(t, 450).kp);  // points win
}

void test_feedforward_interpolates_and_extrapolates() {
  TuningTable t = twoPointTable();
  validateTuning(t);
  TEST_ASSERT_EQUAL_FLOAT(0, feedforwardAt(t, 20, 20));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02f, feedforwardAt(t, 85, 20));    // half of 150's
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.17f, feedforwardAt(t, 450, 20));   // between points
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.43f, feedforwardAt(t, 1050, 20));  // slope continues
  TuningTable empty = kEmptyTuning;
  TEST_ASSERT_EQUAL_FLOAT(0, feedforwardAt(empty, 900, 20));
}

// Run a step test against the model through the real SSR window logic.
StepTestResult runStepTest(KilnModel& m, float target, StepTest::Phase* phase_out,
                           const PidGains& gains = kMidSizeGains, float* peak_heating = nullptr) {
  StepTest test;
  StepTestConfig cfg;
  cfg.target_c = target;
  test.start(cfg, gains);
  TimeProportioningOutput out(4000, 500);
  uint32_t ms = 0;
  for (int s = 0; test.active() && s < 10 * 36000; ++s, ms += 100) {
    if (s % 10 == 0) out.setDuty(test.update(m.thermocouple(), 1.0f));
    m.step(out.update(ms) ? 1.0f : 0.0f, 0.1f);
    if (peak_heating && (test.phase() == StepTest::Phase::Heating || test.phase() == StepTest::Phase::Settling)) {
      *peak_heating = std::fmax(*peak_heating, m.thermocouple());
    }
  }
  *phase_out = test.phase();
  return test.result();
}

void test_step_test_measures_low_and_high_temperature() {
  KilnModel m;
  StepTest::Phase phase;
  const StepTestResult lo = runStepTest(m, 150, &phase);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StepTest::Phase::Done), static_cast<int>(phase));
  TEST_ASSERT_FLOAT_WITHIN(0.015f, m.holdDuty(150), lo.hold_duty);
  TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.35f, lo.rate_c_per_s);  // heat_c_per_s, less losses
  TEST_ASSERT_TRUE(lo.dead_s > 10 && lo.dead_s < 150);      // ~ the 40 s sensor lag

  const StepTestResult hi = runStepTest(m, 750, &phase);  // continues from warm
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StepTest::Phase::Done), static_cast<int>(phase));
  TEST_ASSERT_FLOAT_WITHIN(0.03f, m.holdDuty(750), hi.hold_duty);
  TEST_ASSERT_TRUE(hi.hold_duty > 5 * lo.hold_duty);  // radiation
  TEST_ASSERT_TRUE(hi.gains.kp > 0 && lo.gains.kp > 0);
}

// The firmware defaults on a fast kiln (no heat-up ramp): modest
// overshoot, and the hold still measures the true holding power.
void test_step_test_on_fast_kiln_with_defaults() {
  KilnModel m(quickFire());
  StepTest::Phase phase;
  float peak = 0;
  const StepTestResult r = runStepTest(m, 150, &phase, kDefaultGains, &peak);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StepTest::Phase::Done), static_cast<int>(phase));
  TEST_ASSERT_TRUE(peak < 150 + 10);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, m.holdDuty(150), r.hold_duty);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.5f, r.rate_c_per_s);
}

// Near full power there's no room to step up: it steps down instead and
// measures the cooling response.
void test_step_test_steps_down_near_full_power() {
  KilnModel m;
  float target = 600;
  while (m.holdDuty(target) < 0.96f) target += 10;
  StepTest::Phase phase;
  const StepTestResult r = runStepTest(m, target, &phase);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StepTest::Phase::Done), static_cast<int>(phase));
  TEST_ASSERT_TRUE(r.step_duty < 0);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, m.holdDuty(target), r.hold_duty);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.35f, r.rate_c_per_s);  // same heating rate as stepping up
  TEST_ASSERT_TRUE(r.gains.kp > 0);
}

void test_step_test_fails_safely() {
  StepTest test;
  StepTestConfig cfg;
  test.start(cfg, kDefaultGains);
  test.update(NAN, 1);  // sensor lost
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StepTest::Phase::Failed), static_cast<int>(test.phase()));
  TEST_ASSERT_EQUAL_FLOAT(0, test.update(100, 1));  // no heat once failed
}

void test_scheduled_control_tracks_with_feedforward() {
  // Tune at 150 and 750, then fire the bisque with the schedule.
  KilnModel tune_kiln;
  StepTest::Phase phase;
  const StepTestResult lo = runStepTest(tune_kiln, 150, &phase);
  const StepTestResult hi = runStepTest(tune_kiln, 750, &phase);
  TuningTable t = kEmptyTuning;
  t.count = 2;
  t.pts[0] = {lo.target_c, lo.gains.kp, lo.gains.ki, lo.gains.kd, lo.hold_duty};
  t.pts[1] = {hi.target_c, hi.gains.kp, hi.gains.ki, hi.gains.kd, hi.hold_duty};
  TEST_ASSERT_NULL(validateTuning(t));

  KilnModel m;
  Pid pid;
  ProgramRunner r;
  TimeProportioningOutput out(4000, 500);
  Program p = kBisque;
  p.wait = 0;
  r.start(p);
  float worst = 0;
  uint32_t ms = 0;
  for (int s = 0; r.active() && s < 20 * 36000; ++s, ms += 100) {
    if (s % 10 == 0) {
      const float pv = m.thermocouple();
      const float sp = r.update(pv, 1.0f);
      if (s > 600) worst = std::fmax(worst, std::fabs(pv - sp));  // after the first minute
      pid.setGains(gainsAt(t, sp));
      out.setDuty(pid.update(sp, pv, 1.0f, feedforwardAt(t, sp, m.ambient())));
    }
    m.step(out.update(ms) ? 1.0f : 0.0f, 0.1f);
  }
  TEST_ASSERT_TRUE(r.finished());
  TEST_ASSERT_TRUE(worst < 12);  // tracks the whole firing closely
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_typeK_forward_matches_nist_table);
  RUN_TEST(test_typeK_inverse_round_trips);
  RUN_TEST(test_typeK_inverse_rejects_out_of_range);
  RUN_TEST(test_decode_known_frames);
  RUN_TEST(test_decode_linearizes_high_temperatures);
  RUN_TEST(test_decode_faults);
  RUN_TEST(test_decode_bus_faults);
  RUN_TEST(test_tp_half_duty);
  RUN_TEST(test_tp_small_duty_carries_forward);
  RUN_TEST(test_tp_high_duty_carries_forward);
  RUN_TEST(test_tp_full_and_zero_never_switch);
  RUN_TEST(test_tp_force_off_is_immediate);
  RUN_TEST(test_tp_rejects_bad_duty);
  RUN_TEST(test_tp_survives_millis_wraparound);
  RUN_TEST(test_median_rejects_spike);
  RUN_TEST(test_pid_output_limits_and_bad_input);
  RUN_TEST(test_pid_no_windup_while_saturated);
  RUN_TEST(test_pid_no_derivative_kick_on_setpoint_change);
  RUN_TEST(test_closed_loop_low_setpoint);
  RUN_TEST(test_closed_loop_high_setpoint);
  RUN_TEST(test_program_validation);
  RUN_TEST(test_program_setpoint_interpolates);
  RUN_TEST(test_runner_starts_at_zero_even_when_warm);
  RUN_TEST(test_runner_waits_only_when_behind);
  RUN_TEST(test_runner_waits_for_lagging_kiln);
  RUN_TEST(test_runner_without_wait_follows_clock);
  RUN_TEST(test_runner_finishes);
  RUN_TEST(test_program_closed_loop_tracks_profile);
  RUN_TEST(test_tuning_validation_sorts_and_rejects);
  RUN_TEST(test_gains_interpolate_by_setpoint);
  RUN_TEST(test_baseline_gains_apply_without_points);
  RUN_TEST(test_feedforward_interpolates_and_extrapolates);
  RUN_TEST(test_step_test_measures_low_and_high_temperature);
  RUN_TEST(test_step_test_on_fast_kiln_with_defaults);
  RUN_TEST(test_step_test_steps_down_near_full_power);
  RUN_TEST(test_step_test_fails_safely);
  RUN_TEST(test_scheduled_control_tracks_with_feedforward);
  return UNITY_END();
}
