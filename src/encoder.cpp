#include "encoder.h"

#include <Arduino.h>
#include <soc/gpio_reg.h>

namespace encoder {
namespace {

// kDirection: -1 verified on the Dial v1.1 (clockwise = positive).
// Change kCountsPerDetent if one click moves more or less than one step.
constexpr int kDirection = -1;
constexpr int kCountsPerDetent = 4;

// Transition table indexed by (previous AB << 2) | current AB.
constexpr int8_t kTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

int s_pin_a, s_pin_b;
volatile int32_t s_count = 0;
volatile uint8_t s_state = 0;
int32_t s_consumed = 0;

// Both encoder pins are >= 32, so read GPIO_IN1 directly (IRAM-safe).
inline uint8_t readAB() {
  const uint32_t in = REG_READ(GPIO_IN1_REG);
  return (((in >> (s_pin_a - 32)) & 1) << 1) | ((in >> (s_pin_b - 32)) & 1);
}

void IRAM_ATTR onEdge() {
  s_state = ((s_state << 2) | readAB()) & 0x0F;
  s_count += kTable[s_state];
}

}  // namespace

void begin(int pin_a, int pin_b) {
  s_pin_a = pin_a;
  s_pin_b = pin_b;
  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);
  s_state = readAB();
  attachInterrupt(pin_a, onEdge, CHANGE);
  attachInterrupt(pin_b, onEdge, CHANGE);
}

int32_t takeDetents() {
  const int32_t count = s_count;
  const int32_t detents = (count - s_consumed) / kCountsPerDetent;
  s_consumed += detents * kCountsPerDetent;
  return detents * kDirection;
}

}  // namespace encoder
