#include "pid.h"

#include <cmath>

namespace kiln {

namespace {
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
}  // namespace

void Pid::reset() {
  integral_ = 0.0f;
  d_rate_ = 0.0f;
  primed_ = false;
  p_ = d_ = 0.0f;
}

float Pid::update(float sp, float pv, float dt_s, float ff) {
  if (std::isnan(sp) || std::isnan(pv) || !(dt_s > 0.0f)) return 0.0f;

  if (!primed_) {
    last_pv_ = pv;
    primed_ = true;
  }
  const float rate = (pv - last_pv_) / dt_s;
  last_pv_ = pv;
  const float alpha = dt_s / (dt_s + g_.d_tau_s);
  d_rate_ += alpha * (rate - d_rate_);

  const float e = sp - pv;
  p_ = g_.kp * e;
  d_ = -g_.kd * d_rate_;

  if (std::isnan(ff)) ff = 0.0f;
  const float i_next = integral_ + g_.ki * e * dt_s;
  const float u = ff + p_ + i_next + d_;
  const bool pushing_high = u > 1.0f && e > 0.0f;
  const bool pushing_low = u < 0.0f && e < 0.0f;
  // The integral may go negative to cancel a feed-forward that is too high.
  if (!pushing_high && !pushing_low) integral_ = i_next < -1.0f ? -1.0f : (i_next > 1.0f ? 1.0f : i_next);

  return clamp01(ff + p_ + integral_ + d_);
}

}  // namespace kiln
