#include "thermocouple.h"

#include <cmath>

namespace kiln {
namespace {

// Seebeck coefficient the MAX31855K assumes, in mV/C.
constexpr double kMax31855KSlopeMv = 0.041276;

double poly(const double* c, int n, double x) {
  double y = 0.0;
  for (int i = n - 1; i >= 0; --i) y = y * x + c[i];
  return y;
}

// NIST ITS-90 type K, T -> E (mV).
constexpr double kFwdNeg[] = {
    0.0,
    0.394501280250e-01,
    0.236223735980e-04,
    -0.328589067840e-06,
    -0.499048287770e-08,
    -0.675090591730e-10,
    -0.574103274280e-12,
    -0.310888728940e-14,
    -0.104516093650e-16,
    -0.198892668780e-19,
    -0.163226974860e-22,
};
constexpr double kFwdPos[] = {
    -0.176004136860e-01,
    0.389212049750e-01,
    0.185587700320e-04,
    -0.994575928740e-07,
    0.318409457190e-09,
    -0.560728448890e-12,
    0.560750590590e-15,
    -0.320207200030e-18,
    0.971511471520e-22,
    -0.121047212750e-25,
};
constexpr double kFwdA0 = 0.118597600000e+00;
constexpr double kFwdA1 = -0.118343200000e-03;
constexpr double kFwdA2 = 0.126968600000e+03;

// NIST ITS-90 type K, E (mV) -> T.
constexpr double kInvNeg[] = {  // -5.891 .. 0 mV
    0.0,           2.5173462e+01,  -1.1662878e+00,
    -1.0833638e+00, -8.9773540e-01, -3.7342377e-01,
    -8.6632643e-02, -1.0450598e-02, -5.1920577e-04,
};
constexpr double kInvLow[] = {  // 0 .. 20.644 mV
    0.0,          2.508355e+01,  7.860106e-02,
    -2.503131e-01, 8.315270e-02,  -1.228034e-02,
    9.804036e-04, -4.413030e-05, 1.057734e-06,
    -1.052755e-08,
};
constexpr double kInvHigh[] = {  // 20.644 .. 54.886 mV
    -1.318058e+02, 4.830222e+01,  -1.646031e+00, 5.464731e-02,
    -9.650715e-04, 8.802193e-06, -3.110810e-08,
};

template <int N>
constexpr int len(const double (&)[N]) {
  return N;
}

int32_t signExtend(uint32_t value, int bits) {
  const uint32_t sign = 1u << (bits - 1);
  return static_cast<int32_t>((value ^ sign) - sign);
}

}  // namespace

double typeKMillivolts(double t) {
  if (t < 0.0) return poly(kFwdNeg, len(kFwdNeg), t);
  const double d = t - kFwdA2;
  return poly(kFwdPos, len(kFwdPos), t) + kFwdA0 * std::exp(kFwdA1 * d * d);
}

double typeKCelsius(double mv) {
  // Table limits, plus 1 uV so the exact -200 / 1372 C endpoints round-trip.
  if (mv < -5.892 || mv > 54.887) return NAN;
  if (mv < 0.0) return poly(kInvNeg, len(kInvNeg), mv);
  if (mv < 20.644) return poly(kInvLow, len(kInvLow), mv);
  return poly(kInvHigh, len(kInvHigh), mv);
}

double linearizeTypeK(double raw_c, double cj_c) {
  // The chip reports Tcj + Vtc / slope. Recover the thermocouple voltage,
  // add the true cold-junction EMF, and invert the real curve.
  const double v_tc = kMax31855KSlopeMv * (raw_c - cj_c);
  return typeKCelsius(v_tc + typeKMillivolts(cj_c));
}

TcReading decodeMax31855(uint32_t frame) {
  TcReading r{kTcFaultNone, NAN, NAN, NAN};

  // All-zero / all-one frames mean SO is stuck (board missing or unpowered).
  // D17 and D3 are reserved and always read 0.
  if (frame == 0 || frame == 0xFFFFFFFFu || (frame & ((1u << 17) | (1u << 3)))) {
    r.faults = kTcFaultBus;
    return r;
  }

  r.cj_c = signExtend((frame >> 4) & 0x0FFF, 12) * 0.0625f;

  if (frame & (1u << 16)) {
    if (frame & 0x1) r.faults |= kTcFaultOpen;
    if (frame & 0x2) r.faults |= kTcFaultShortGnd;
    if (frame & 0x4) r.faults |= kTcFaultShortVcc;
    if (r.faults == kTcFaultNone) r.faults = kTcFaultBus;  // fault bit, no cause
    // Short-to-GND/VCC still comes with a measurement (see thermocouple.h);
    // an open circuit or an unexplained fault bit doesn't.
    if (r.faults & (kTcFaultOpen | kTcFaultBus)) return r;
  }

  r.raw_c = signExtend(frame >> 18, 14) * 0.25f;
  const double t = linearizeTypeK(r.raw_c, r.cj_c);
  if (std::isnan(t)) {
    r.faults |= kTcFaultRange;
    return r;
  }
  r.temp_c = static_cast<float>(t);
  return r;
}

const char* tcFaultName(uint8_t faults) {
  if (faults == kTcFaultNone) return "OK";
  if (faults & kTcFaultBus) return "NO SENSOR";
  if (faults & kTcFaultOpen) return "TC OPEN";
  if (faults & kTcFaultShortGnd) return "TC SHORT GND";
  if (faults & kTcFaultShortVcc) return "TC SHORT VCC";
  if (faults & kTcFaultRange) return "TC RANGE";
  return "TC FAULT";
}

}  // namespace kiln
