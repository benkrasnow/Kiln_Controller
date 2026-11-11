#include "tp_output.h"

namespace kiln {

TimeProportioningOutput::TimeProportioningOutput(uint32_t window_ms, uint32_t min_ms)
    : window_ms_(window_ms < 1000 ? 1000 : window_ms),
      min_ms_(min_ms * 2 > window_ms_ ? window_ms_ / 2 : min_ms) {}

void TimeProportioningOutput::setDuty(float duty) {
  if (!(duty > 0.0f)) duty = 0.0f;  // also catches NaN
  if (duty > 1.0f) duty = 1.0f;
  duty_ = duty;
}

void TimeProportioningOutput::forceOff() {
  duty_ = 0.0f;
  carry_ms_ = 0.0f;
  on_ms_ = 0;
}

bool TimeProportioningOutput::update(uint32_t now_ms) {
  // Unsigned subtraction keeps this correct across millis() wraparound.
  if (!started_ || now_ms - window_start_ >= window_ms_) {
    started_ = true;
    // Restart from "now" rather than advancing by one window, so a stalled
    // loop never produces a burst of catch-up windows.
    window_start_ = now_ms;

    float want = duty_ * window_ms_ + carry_ms_;
    uint32_t on;
    if (want < min_ms_) {
      on = 0;
    } else if (want > window_ms_ - min_ms_) {
      on = window_ms_;
    } else {
      on = static_cast<uint32_t>(want + 0.5f);
    }
    carry_ms_ = duty_ > 0.0f ? want - on : 0.0f;
    on_ms_ = on;
  }
  return now_ms - window_start_ < on_ms_;
}

float TimeProportioningOutput::windowPosition(uint32_t now_ms) const {
  if (!started_) return 0.0f;
  const uint32_t t = now_ms - window_start_;
  return t >= window_ms_ ? 1.0f : static_cast<float>(t) / window_ms_;
}

}  // namespace kiln
