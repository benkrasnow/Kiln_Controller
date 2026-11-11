// Interrupt-driven quadrature decoder for the Dial's rotary encoder.
#pragma once

#include <cstdint>

namespace encoder {

void begin(int pin_a, int pin_b);

// Detents turned since the last call (positive = clockwise once kDirection
// is set correctly).
int32_t takeDetents();

}  // namespace encoder
