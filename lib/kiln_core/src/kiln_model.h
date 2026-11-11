// Crude lumped thermal model of an electric kiln, for tests and the
// simulation build: one thermal mass, conduction loss proportional to the
// temperature rise plus radiation loss proportional to T^4 (kelvin), and a
// first-order thermocouple lag. Radiation dominates at high temperature, so
// the kiln needs far more power to hold 750 C than 150 C, as real kilns do.
#pragma once

namespace kiln {

class KilnModel {
 public:
  struct Params {
    float ambient_c = 22.0f;
    float heat_c_per_s = 0.35f;    // rise rate at full power with no losses (~1260 C/h)
    float k_cond = 8.2e-5f;        // conduction loss, C/s per C above ambient
    float k_rad = 4.0e-14f;        // radiation loss, C/s per K^4 (T^4 - Ta^4)
    float sensor_tau_s = 40.0f;    // thermocouple + sheath lag
  };
  // With these values: ~3% power holds 150 C, ~29% holds 750 C, and full
  // power tops out a little above 1300 C.

  KilnModel() : KilnModel(Params{}) {}
  explicit KilnModel(const Params& p) : p_(p), kiln_c_(p.ambient_c), tc_c_(p.ambient_c) {}

  void step(float duty, float dt_s) {
    kiln_c_ += (p_.heat_c_per_s * duty - lossCPerS(kiln_c_)) * dt_s;
    tc_c_ += (kiln_c_ - tc_c_) * dt_s / (p_.sensor_tau_s + dt_s);
  }

  // Power fraction needed to hold a temperature steady.
  float holdDuty(float c) const { return lossCPerS(c) / p_.heat_c_per_s; }

  float thermocouple() const { return tc_c_; }
  float ambient() const { return p_.ambient_c; }

 private:
  float lossCPerS(float c) const {
    const double t = c + 273.15, ta = p_.ambient_c + 273.15;
    return p_.k_cond * (c - p_.ambient_c) + static_cast<float>(p_.k_rad * (t * t * t * t - ta * ta * ta * ta));
  }

  Params p_;
  float kiln_c_;
  float tc_c_;
};

}  // namespace kiln
