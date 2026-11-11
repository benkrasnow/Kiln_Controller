// Main screen for the Dial's round 240x240 LCD.
#pragma once

struct UiState {
  float temp_c;          // actual, filtered; NAN if no valid reading
  float setpoint_c;      // shown setpoint (pending value while editing)
  float setpoint_max_c;  // full scale of the ring gauge
  bool editing;          // setpoint is being turned and not yet applied
  bool running;          // heating under control (setpoint or program)
  bool program;          // a firing program is running
  bool waiting;          // program clock paused until the kiln catches up
  int prog_seg;          // current segment, 1-based
  int prog_segs;
  float prog_left_s;     // program time remaining
  float duty;            // 0..1 controller output
  bool ssr_on;
  const char* fault;     // nullptr when OK
  bool simulated;
  const char* toast;     // brief notice (e.g. a change made from the PC)
  bool tuning;           // a step test is running
  const char* detail;    // step-test phase line, or nullptr
};

namespace ui {

void begin();
void draw(const UiState& s);

}  // namespace ui
