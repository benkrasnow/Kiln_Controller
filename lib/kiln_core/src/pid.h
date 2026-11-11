// PID temperature controller producing a 0..1 heater duty.
// - Optional feed-forward: a baseline duty (e.g. the power the kiln needs to
//   hold the setpoint) that the PID terms correct around.
// - Gains can change between calls (gain scheduling) without a bump: the
//   integral is stored in duty units, so new gains only affect what follows.
// - Derivative acts on the measurement (no kick when the setpoint changes)
//   and is low-pass filtered against thermocouple noise.
// - Anti-windup: the integral stops growing while the output is saturated
//   in the direction the error is pushing, and is clamped to -1..1.
#pragma once

namespace kiln {

struct PidGains {
  float kp;        // duty per C of error
  float ki;        // duty per C*s of accumulated error
  float kd;        // duty per C/s of measurement rate
  float d_tau_s;   // derivative filter time constant
};

// Untuned starting point, sized for a small, fast kiln (Paragon QuickFire:
// ~2 C/s at full power, ~10 s dead time, measured at 150 C). This is what
// suggestGains gives for that kiln. Used until a tuning table is stored,
// including for the step test's heat-up and hold.
constexpr PidGains kDefaultGains{0.033f, 0.00055f, 0.27f, 10.0f};

class Pid {
 public:
  explicit Pid(PidGains g = kDefaultGains) : g_(g) {}

  // Call when control starts so the old state can't cause a bump.
  void reset();

  void setGains(const PidGains& g) { g_ = g; }
  const PidGains& gains() const { return g_; }

  // dt_s: seconds since the previous call; ff: feed-forward duty.
  // Returns duty 0..1.
  float update(float setpoint, float measured, float dt_s, float ff = 0.0f);

  float p() const { return p_; }
  float i() const { return integral_; }
  float d() const { return d_; }

 private:
  PidGains g_;
  float integral_ = 0.0f;
  float last_pv_ = 0.0f;
  float d_rate_ = 0.0f;  // filtered dPV/dt
  bool primed_ = false;
  float p_ = 0.0f, d_ = 0.0f;
};

}  // namespace kiln
