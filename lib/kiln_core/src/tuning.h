// Temperature-dependent tuning: a small table of points measured by step
// tests, interpolated by setpoint (gain scheduling), plus the holding power
// at each point used as a feed-forward baseline.
#pragma once

#include <cstdint>

#include "pid.h"

namespace kiln {

constexpr int kMaxTunePoints = 4;

struct TunePoint {
  float temp_c;     // temperature the point was measured at
  float kp, ki, kd;
  float hold_duty;  // power needed to hold temp_c steady, 0..1
};

struct TuningTable {
  uint8_t layout;  // bump kTuningLayout if this struct changes (NVS blob)
  uint8_t count;   // 0: use `base` and no feed-forward
  TunePoint pts[kMaxTunePoints];  // sorted by temp_c, increasing
  PidGains base;   // gains with no points (d_tau_s is always kDefaultGains')
};
constexpr uint8_t kTuningLayout = 2;

// No points, built-in baseline gains.
constexpr TuningTable kEmptyTuning{kTuningLayout, 0, {}, kDefaultGains};

// nullptr if usable, otherwise a short reason. Sorts the points.
const char* validateTuning(TuningTable& t);

// Gains for a setpoint: linear between points, the end values outside them.
PidGains gainsAt(const TuningTable& t, float setpoint_c);

// Holding-power feed-forward for a setpoint: linear between points, falling
// to 0 at ambient below the first point, extrapolated (clamped to 0..1)
// above the last. 0 with an empty table.
float feedforwardAt(const TuningTable& t, float setpoint_c, float ambient_c = 20.0f);

// ---- Step test ---------------------------------------------------------------

struct StepTestConfig {
  float target_c = 150.0f;
  float approach_c_per_min = 0.0f;  // > 0: heat-up ramps its setpoint at this rate
  float step_duty = 0.20f;       // added to the holding power (or taken off, near full power)
  float settle_band_c = 5.0f;    // "steady" means within this of the target...
  float settle_s = 300.0f;       // ...continuously for this long
  float settle_drift_c = 5.0f;   // ...and moved no more than this over it
  float max_rise_c = 50.0f;      // stop the step after this much rise (or fall)
  float max_step_s = 3600.0f;    // ...or after this long
  float max_total_s = 6 * 3600.0f;
};

struct StepTestResult {
  float target_c;
  float hold_duty;    // power to hold the target: the steady window's average, corrected for its drift
  float step_duty;    // the step actually applied (negative: stepped down)
  float rate_c_per_s; // peak heating rate per unit of power step
  float dead_s;       // dead time (tangent method)
  PidGains gains;     // suggested gains
};

// Gains from a step test (SIMC rule for a lag-dominant, integrating-like
// process, tuned fast: close to critically damped). A small D sized from the
// dead time damps the heating elements' thermal momentum.
PidGains suggestGains(float rate_c_per_s, float dead_s);

class StepTest {
 public:
  enum class Phase : uint8_t { Idle, Heating, Settling, Stepping, Done, Failed };

  // `hold_gains` control the kiln while heating to and holding the target.
  void start(const StepTestConfig& cfg, const PidGains& hold_gains);
  void abort(const char* why);

  // Advance by dt_s with the current reading; returns the duty to apply.
  float update(float pv_c, float dt_s);

  Phase phase() const { return phase_; }
  bool active() const { return phase_ == Phase::Heating || phase_ == Phase::Settling || phase_ == Phase::Stepping; }
  float elapsedS() const { return total_s_; }
  float phaseElapsedS() const { return phase_s_; }
  // Settling: how long the current steady window has lasted, and how far
  // the kiln has moved since it began. Stepping: rise since the step began.
  float steadyS() const { return in_band_s_; }
  float driftC() const { return phase_ == Phase::Settling && in_band_s_ > 0 ? last_pv_ - band_pv0_ : 0.0f; }
  float riseC() const { return phase_ == Phase::Stepping ? last_pv_ - pv0_ : 0.0f; }
  const StepTestConfig& config() const { return cfg_; }
  const StepTestResult& result() const { return result_; }
  const char* failure() const { return failure_; }
  static const char* phaseName(Phase p);

 private:
  void finishStep();

  StepTestConfig cfg_{};
  Pid pid_;
  Phase phase_ = Phase::Idle;
  float total_s_ = 0, phase_s_ = 0;
  float in_band_s_ = 0;
  float band_pv0_ = 0;  // temperature when the steady window began
  double duty_sum_ = 0;  // integral of duty over the steady window
  float hold_duty_ = 0, step_duty_ = 0;
  float hold_drift_ = 0;  // C/s the kiln moved over the steady window
  float pv0_ = 0;
  float last_pv_ = 0;
  float ramp_sp_ = 0;  // heat-up setpoint, ramping toward target_c
  bool ramp_primed_ = false;
  // Slope tracking during the step: PV sampled every kSampleS.
  static constexpr float kMinStep = 0.05f;  // smallest usable power step
  static constexpr int kSamples = 7;
  static constexpr float kSampleS = 10.0f;
  float samples_[kSamples] = {};
  int n_samples_ = 0;
  float since_sample_ = 0;
  float max_slope_ = 0, t_at_max_ = 0, pv_at_max_ = 0;
  StepTestResult result_{};
  const char* failure_ = nullptr;
};

}  // namespace kiln
