// Firing programs: a list of control points (minutes from start, temperature).
// The setpoint ramps linearly between points; equal temperatures are holds.
#pragma once

#include <cstdint>

namespace kiln {

constexpr int kMaxProgramPoints = 32;
constexpr int kMaxProgramName = 32;

struct ProgramPoint {
  float min;  // minutes from program start
  float c;    // temperature to reach at that time
};

struct Program {
  uint8_t layout;  // bump kProgramLayout if this struct changes (NVS blob)
  uint8_t count;
  uint8_t wait;    // 1: pause the clock while the kiln is off-profile
  char name[kMaxProgramName + 1];
  ProgramPoint pts[kMaxProgramPoints];
};
constexpr uint8_t kProgramLayout = 2;

// nullptr if usable, otherwise a short reason.
const char* validateProgram(const Program& p, float max_c);

float programDurationS(const Program& p);

// Profile setpoint at a time since start (clamped to the program's span).
float programSetpoint(const Program& p, float elapsed_s);

// 1-based index of the segment containing elapsed_s.
int programSegment(const Program& p, float elapsed_s);

// Runs a program against the measured temperature: start() from t = 0,
// resume() from any point (after a reboot, or a jump chosen by the user).
// With Program::wait set, the program clock pauses while the kiln is more
// than kWaitBandC *behind* the profile: too cold on a rise or hold, too hot
// on a cooling segment. A slow kiln therefore gets the full ramp and hold
// times ("guaranteed soak"), while a kiln that is ahead (e.g. still warm
// when a program starts) never holds the clock up. Without wait, the
// setpoint follows the clock regardless of the kiln.
class ProgramRunner {
 public:
  static constexpr float kWaitBandC = 15.0f;

  void start(const Program& p);
  void resume(const Program& p, float elapsed_s);
  void stop() { active_ = false; }

  // Advance by dt_s; returns the setpoint to control to.
  float update(float pv_c, float dt_s);

  bool active() const { return active_; }
  bool finished() const { return finished_; }
  bool waiting() const { return waiting_; }
  float elapsedS() const { return elapsed_s_; }
  float durationS() const { return duration_s_; }
  int segment() const;

 private:
  const Program* prog_ = nullptr;
  bool active_ = false;
  bool finished_ = false;
  bool waiting_ = false;
  float elapsed_s_ = 0.0f;
  float duration_s_ = 0.0f;
};

}  // namespace kiln
