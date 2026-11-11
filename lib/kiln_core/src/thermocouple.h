// MAX31855 frame decoding and type-K NIST linearization.
// Hardware-independent so it can be unit tested on the host.
#pragma once

#include <cstdint>

namespace kiln {

// Bit flags; a reading may carry several.
enum TcFault : uint8_t {
  kTcFaultNone = 0,
  kTcFaultOpen = 1 << 0,      // thermocouple not connected
  kTcFaultShortGnd = 1 << 1,  // thermocouple shorted to GND
  kTcFaultShortVcc = 1 << 2,  // thermocouple shorted to VCC
  kTcFaultBus = 1 << 3,       // SPI frame implausible (board missing, wiring)
  kTcFaultRange = 1 << 4,     // voltage outside the NIST type-K table
};

// Short-to-GND/VCC flags are warnings, not failures: in a hot kiln the
// elements leak current through the refractory and the thermocouple sheath,
// and the MAX31855 flags that, especially while the SSR is on, but the
// temperature it reports is still good. Such readings keep temp_c.
struct TcReading {
  uint8_t faults;   // TcFault flags
  // temp_c is NAN when there is no usable measurement (open, bus, range);
  // with only kTcFaultShortGnd/kTcFaultShortVcc it is still valid.
  float raw_c;      // MAX31855's own (linear-approximation) hot-junction temp
  float cj_c;       // cold-junction (die) temperature
  float temp_c;     // NIST-linearized hot-junction temperature
};

// Decode a 32-bit MAX31855 frame and linearize it (type K).
TcReading decodeMax31855(uint32_t frame);

// NIST ITS-90 type-K reference function: temperature (C) -> EMF (mV).
// Valid -270..1372 C.
double typeKMillivolts(double temp_c);

// NIST ITS-90 type-K inverse function: EMF (mV) -> temperature (C).
// Valid -5.891..54.886 mV (-200..1372 C). Returns NAN outside that range.
double typeKCelsius(double mv);

// Correct the MAX31855's linear approximation using the NIST tables.
// Returns NAN if the implied EMF is outside the inverse table.
double linearizeTypeK(double raw_c, double cj_c);

const char* tcFaultName(uint8_t faults);

}  // namespace kiln
