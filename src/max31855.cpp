#include "max31855.h"

#include <Arduino.h>

void Max31855::begin() {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(sck_, OUTPUT);
  digitalWrite(sck_, LOW);
  // Pull-up so a missing board reads 0xFFFFFFFF (a detectable bus fault)
  // instead of floating noise.
  pinMode(so_, INPUT_PULLUP);
}

uint32_t Max31855::readFrame() {
  uint32_t frame = 0;
  digitalWrite(cs_, LOW);  // D31 appears on SO; this also stops conversion
  delayMicroseconds(1);
  for (int i = 0; i < 32; ++i) {
    frame = (frame << 1) | (digitalRead(so_) ? 1u : 0u);
    digitalWrite(sck_, HIGH);
    delayMicroseconds(1);
    digitalWrite(sck_, LOW);  // next bit shifts out on the falling edge
    delayMicroseconds(1);
  }
  digitalWrite(cs_, HIGH);  // restarts conversion (~100 ms)
  return frame;
}
