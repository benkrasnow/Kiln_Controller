#include "tuning.h"

#include <cmath>

namespace kiln {
namespace {

float lerp(float a, float b, float f) { return a + (b - a) * f; }
float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

const char* validateTuning(TuningTable& t) {
  if (t.layout != kTuningLayout) return "bad layout";
  if (t.count > kMaxTunePoints) return "too many points";
  const PidGains& b = t.base;
  if (!std::isfinite(b.kp) || !std::isfinite(b.ki) || !std::isfinite(b.kd)) return "bad number";
  if (b.kp < 0 || b.ki < 0 || b.kd < 0) return "gains must not be negative";
  t.base.d_tau_s = kDefaultGains.d_tau_s;
  for (int i = 0; i < t.count; ++i) {  // insertion sort by temperature
    for (int j = i; j > 0 && t.pts[j].temp_c < t.pts[j - 1].temp_c; --j) {
      const TunePoint tmp = t.pts[j];
      t.pts[j] = t.pts[j - 1];
      t.pts[j - 1] = tmp;
    }
  }
  for (int i = 0; i < t.count; ++i) {
    const TunePoint& p = t.pts[i];
    if (!std::isfinite(p.temp_c) || !std::isfinite(p.kp) || !std::isfinite(p.ki) ||
        !std::isfinite(p.kd) || !std::isfinite(p.hold_duty)) {
      return "bad number";
    }
    if (p.kp < 0 || p.ki < 0 || p.kd < 0) return "gains must not be negative";
    if (p.hold_duty < 0 || p.hold_duty > 1) return "hold power must be 0..1";
    if (i > 0 && p.temp_c - t.pts[i - 1].temp_c < 1.0f) return "temperatures must differ";
  }
  return nullptr;
}

PidGains gainsAt(const TuningTable& t, float sp) {
  if (t.count == 0 || std::isnan(sp)) return t.base;
  const TunePoint* a = &t.pts[0];
  const TunePoint* b = &t.pts[0];
  if (sp >= t.pts[t.count - 1].temp_c) {
    a = b = &t.pts[t.count - 1];
  } else if (sp > t.pts[0].temp_c) {
    int i = 1;
    while (t.pts[i].temp_c < sp) ++i;
    a = &t.pts[i - 1];
    b = &t.pts[i];
  }
  const float f = b == a ? 0.0f : (sp - a->temp_c) / (b->temp_c - a->temp_c);
  return {lerp(a->kp, b->kp, f), lerp(a->ki, b->ki, f), lerp(a->kd, b->kd, f), kDefaultGains.d_tau_s};
}

float feedforwardAt(const TuningTable& t, float sp, float ambient_c) {
  if (t.count == 0 || std::isnan(sp) || sp <= ambient_c) return 0.0f;
  const TunePoint& first = t.pts[0];
  if (sp <= first.temp_c) {  // ambient (no loss) up to the first point
    return first.temp_c > ambient_c ? first.hold_duty * (sp - ambient_c) / (first.temp_c - ambient_c) : 0.0f;
  }
  for (int i = 1; i < t.count; ++i) {
    const TunePoint& a = t.pts[i - 1];
    const TunePoint& b = t.pts[i];
    if (sp <= b.temp_c) return lerp(a.hold_duty, b.hold_duty, (sp - a.temp_c) / (b.temp_c - a.temp_c));
  }
  // Above the last point: continue the last segment's slope (radiation loss
  // keeps rising), or the ambient-to-point slope with a single point.
  const TunePoint& b = t.pts[t.count - 1];
  const TunePoint* a = t.count > 1 ? &t.pts[t.count - 2] : nullptr;
  const float a_t = a ? a->temp_c : ambient_c;
  const float a_d = a ? a->hold_duty : 0.0f;
  if (b.temp_c - a_t < 1.0f) return clamp(b.hold_duty, 0, 1);
  return clamp(b.hold_duty + (b.hold_duty - a_d) * (sp - b.temp_c) / (b.temp_c - a_t), 0.0f, 1.0f);
}

PidGains suggestGains(float rate, float dead_s) {
  // SIMC for an integrating process with dead time, closed-loop time
  // constant tau_c = dead time / 2 (fast, close to critically damped; a
  // little overshoot is acceptable for a kiln):
  //   Kc = 1 / (k' (tau_c + theta)),  Ti = 4 (tau_c + theta)
  // plus Td = theta / 2 to damp overshoot from the elements' heat capacity.
  const float theta = dead_s < 10.0f ? 10.0f : dead_s;  // the 4 s SSR window etc.
  const float tc = 0.5f * theta;
  const float kc = 1.0f / (rate * (tc + theta));
  const float ti = 4.0f * (tc + theta);
  const float td = 0.5f * theta;
  return {kc, kc / ti, kc * td, kDefaultGains.d_tau_s};
}

const char* StepTest::phaseName(Phase p) {
  switch (p) {
    case Phase::Heating: return "heating";
    case Phase::Settling: return "settling";
    case Phase::Stepping: return "stepping";
    case Phase::Done: return "done";
    case Phase::Failed: return "failed";
    default: return "idle";
  }
}

void StepTest::start(const StepTestConfig& cfg, const PidGains& hold_gains) {
  cfg_ = cfg;
  pid_ = Pid(hold_gains);
  phase_ = Phase::Heating;
  total_s_ = phase_s_ = in_band_s_ = 0;
  duty_sum_ = 0;
  n_samples_ = 0;
  since_sample_ = 0;
  max_slope_ = 0;
  ramp_primed_ = false;
  failure_ = nullptr;
  result_ = {};
}

void StepTest::abort(const char* why) {
  if (!active()) return;
  phase_ = Phase::Failed;
  failure_ = why;
}

float StepTest::update(float pv, float dt) {
  if (!active()) return 0.0f;
  if (std::isnan(pv)) {
    abort("no temperature reading");
    return 0.0f;
  }
  last_pv_ = pv;
  total_s_ += dt;
  phase_s_ += dt;
  if (total_s_ > cfg_.max_total_s) {
    abort("took too long");
    return 0.0f;
  }

  if (phase_ == Phase::Heating || phase_ == Phase::Settling) {
    // Heat-up ramps its setpoint from the starting temperature, so a fast
    // kiln doesn't charge its elements at full power and coast far past the
    // target. The hold uses the target itself.
    float sp = cfg_.target_c;
    if (phase_ == Phase::Heating && cfg_.approach_c_per_min > 0.0f) {
      if (!ramp_primed_) {
        ramp_sp_ = pv;
        ramp_primed_ = true;
      }
      ramp_sp_ += cfg_.approach_c_per_min * dt / 60.0f;
      if (ramp_sp_ > cfg_.target_c) ramp_sp_ = cfg_.target_c;
      sp = ramp_sp_;
    }
    const float duty = pid_.update(sp, pv, dt);
    if (phase_ == Phase::Heating) {
      if (sp >= cfg_.target_c && pv >= cfg_.target_c - cfg_.settle_band_c) {
        phase_ = Phase::Settling;
        phase_s_ = in_band_s_ = 0;
        duty_sum_ = 0;
      }
      return duty;
    }
    // Settling: average the duty over a continuous in-band window. The
    // window must also be flat: while the kiln still drifts (e.g. cooling
    // off an overshoot) the average power is not the holding power.
    if (std::fabs(pv - cfg_.target_c) <= cfg_.settle_band_c) {
      if (in_band_s_ == 0) band_pv0_ = pv;
      in_band_s_ += dt;
      duty_sum_ += duty * dt;
    } else {
      in_band_s_ = 0;
      duty_sum_ = 0;
    }
    if (in_band_s_ >= cfg_.settle_s && std::fabs(pv - band_pv0_) > cfg_.settle_drift_c) {
      in_band_s_ = 0;  // still drifting: start a new window
      duty_sum_ = 0;
    }
    if (in_band_s_ >= cfg_.settle_s) {
      hold_duty_ = static_cast<float>(duty_sum_ / in_band_s_);
      hold_drift_ = (pv - band_pv0_) / in_band_s_;
      // Step up if there's room; near full power (very hot), step down
      // instead and measure the cooling response.
      step_duty_ = clamp(hold_duty_ + cfg_.step_duty, 0.0f, 1.0f) - hold_duty_;
      if (step_duty_ < kMinStep) step_duty_ = -std::fmin(cfg_.step_duty, hold_duty_);
      if (std::fabs(step_duty_) < kMinStep) {
        abort("holding power leaves no room for a step");
        return 0.0f;
      }
      phase_ = Phase::Stepping;
      phase_s_ = 0;
      pv0_ = pv;
      samples_[0] = pv;
      n_samples_ = 1;
      since_sample_ = 0;
      max_slope_ = 0;
    }
    return duty;
  }

  // Stepping: fixed power, record the response.
  since_sample_ += dt;
  if (since_sample_ >= kSampleS) {
    since_sample_ = 0;
    if (n_samples_ < kSamples) {
      samples_[n_samples_++] = pv;
    } else {
      for (int i = 1; i < kSamples; ++i) samples_[i - 1] = samples_[i];
      samples_[kSamples - 1] = pv;
    }
    if (n_samples_ == kSamples) {
      // Slope over the last minute, centred in the window.
      const float span_s = (kSamples - 1) * kSampleS;
      // In the step's direction, so a downward step reads as positive.
      const float dir = step_duty_ > 0 ? 1.0f : -1.0f;
      const float slope = dir * (samples_[kSamples - 1] - samples_[0]) / span_s;
      if (slope > max_slope_) {
        max_slope_ = slope;
        t_at_max_ = phase_s_ - span_s / 2;
        pv_at_max_ = 0.5f * (samples_[0] + samples_[kSamples - 1]);
      }
      // The response has bent over: the peak rate is behind us.
      const bool past_peak = max_slope_ > 0 && slope < 0.7f * max_slope_ && phase_s_ > 300.0f;
      if (past_peak || dir * (pv - pv0_) >= cfg_.max_rise_c || phase_s_ >= cfg_.max_step_s) finishStep();
    }
  }
  return phase_ == Phase::Stepping ? hold_duty_ + step_duty_ : 0.0f;
}

void StepTest::finishStep() {
  if (max_slope_ <= 0) {
    abort(step_duty_ > 0 ? "kiln did not heat during the step" : "kiln did not cool during the step");
    return;
  }
  // Tangent at the steepest point; where it crosses the starting
  // temperature gives the dead time.
  const float dir = step_duty_ > 0 ? 1.0f : -1.0f;
  float dead = t_at_max_ - dir * (pv_at_max_ - pv0_) / max_slope_;
  if (dead < 0) dead = 0;
  result_.target_c = cfg_.target_c;
  // The steady window allows some drift (e.g. still cooling off an
  // overshoot), so its average power is off by the power that drift took:
  // drift rate / heating rate per unit power.
  const float rate = max_slope_ / std::fabs(step_duty_);
  result_.hold_duty = clamp(hold_duty_ - hold_drift_ / rate, 0.0f, 1.0f);
  result_.step_duty = step_duty_;
  result_.rate_c_per_s = rate;
  result_.dead_s = dead;
  result_.gains = suggestGains(result_.rate_c_per_s, dead);
  phase_ = Phase::Done;
}

}  // namespace kiln
