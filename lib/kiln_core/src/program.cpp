#include "program.h"

#include <cmath>

namespace kiln {

const char* validateProgram(const Program& p, float max_c) {
  if (p.layout != kProgramLayout) return "bad layout";
  if (p.count < 2) return "needs at least 2 points";
  if (p.count > kMaxProgramPoints) return "too many points";
  if (p.pts[0].min != 0.0f) return "first point must be at 0 min";
  for (int i = 0; i < p.count; ++i) {
    const ProgramPoint& q = p.pts[i];
    if (!std::isfinite(q.min) || !std::isfinite(q.c)) return "bad number";
    if (q.c < 0.0f || q.c > max_c) return "temperature out of range";
    if (i > 0 && !(q.min > p.pts[i - 1].min)) return "times must increase";
  }
  if (p.pts[p.count - 1].min > 7 * 24 * 60) return "longer than a week";
  return nullptr;
}

float programDurationS(const Program& p) {
  return p.count ? p.pts[p.count - 1].min * 60.0f : 0.0f;
}

float programSetpoint(const Program& p, float elapsed_s) {
  if (p.count == 0) return NAN;
  const float t = elapsed_s / 60.0f;
  if (t <= p.pts[0].min) return p.pts[0].c;
  for (int i = 1; i < p.count; ++i) {
    const ProgramPoint& a = p.pts[i - 1];
    const ProgramPoint& b = p.pts[i];
    if (t <= b.min) return a.c + (b.c - a.c) * (t - a.min) / (b.min - a.min);
  }
  return p.pts[p.count - 1].c;
}

int programSegment(const Program& p, float elapsed_s) {
  const float t = elapsed_s / 60.0f;
  for (int i = 1; i < p.count; ++i) {
    if (t < p.pts[i].min) return i;
  }
  return p.count > 1 ? p.count - 1 : 0;
}

void ProgramRunner::start(const Program& p) { resume(p, 0.0f); }

void ProgramRunner::resume(const Program& p, float elapsed_s) {
  prog_ = &p;
  duration_s_ = programDurationS(p);
  elapsed_s_ = elapsed_s < 0.0f ? 0.0f : elapsed_s;
  active_ = true;
  finished_ = false;
  waiting_ = false;
}

float ProgramRunner::update(float pv_c, float dt_s) {
  if (!active_) return NAN;
  const float sp = programSetpoint(*prog_, elapsed_s_);
  if (!prog_->wait) {
    waiting_ = false;
  } else if (std::isnan(pv_c)) {
    waiting_ = true;
  } else {
    // Which way is the profile heading? Look a minute ahead.
    const bool cooling = programSetpoint(*prog_, elapsed_s_ + 60.0f) < sp;
    waiting_ = cooling ? pv_c > sp + kWaitBandC : pv_c < sp - kWaitBandC;
  }
  if (!waiting_) elapsed_s_ += dt_s;
  if (elapsed_s_ >= duration_s_) {
    elapsed_s_ = duration_s_;
    active_ = false;
    finished_ = true;
  }
  return programSetpoint(*prog_, elapsed_s_);
}

int ProgramRunner::segment() const {
  return prog_ ? programSegment(*prog_, elapsed_s_) : 0;
}

}  // namespace kiln
