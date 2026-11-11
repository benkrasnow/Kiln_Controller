// Time-proportioning on/off output for a slow SSR.
// Each window has at most one ON period followed by one OFF period, so the
// SSR changes state at most twice per window. Pulses shorter than min_ms are
// suppressed (or extended to a full window) and the error is carried forward
// so the average power still matches the requested duty.
#pragma once

#include <cstdint>

namespace kiln {

class TimeProportioningOutput {
 public:
  TimeProportioningOutput(uint32_t window_ms = 4000, uint32_t min_ms = 500);

  // Requested duty 0..1; takes effect at the start of the next window.
  void setDuty(float duty);

  // Turn off immediately (mid-window) and discard carried error.
  void forceOff();

  // Call often. Returns the state the SSR pin should have right now.
  bool update(uint32_t now_ms);

  float requestedDuty() const { return duty_; }
  // Duty actually being delivered in the current window.
  float windowDuty() const { return static_cast<float>(on_ms_) / window_ms_; }
  uint32_t windowMs() const { return window_ms_; }
  // 0..1 position within the current window.
  float windowPosition(uint32_t now_ms) const;

 private:
  uint32_t window_ms_;
  uint32_t min_ms_;
  float duty_ = 0.0f;
  float carry_ms_ = 0.0f;
  uint32_t window_start_ = 0;
  uint32_t on_ms_ = 0;
  bool started_ = false;
};

}  // namespace kiln
