// GPIO assignments. See PROPOSAL.md section 2 for the reasoning.
#pragma once

// Port B (Grove): no I2C pull-ups, so the SSR line can't float high at boot.
// Measured on the Dial: on both ports G1/G15 are the yellow wire and G2/G13
// the white one, the reverse of M5Stack's colour table.
constexpr int kPinSsr = 1;     // SSR driver (yellow); external 10k pull-down required
constexpr int kPinTcSo = 2;    // MAX31855 SO, data out of the chip (white)

// Port A (Grove).
constexpr int kPinTcCs = 13;   // MAX31855 CS, idles high (white)
constexpr int kPinTcSck = 15;  // MAX31855 SCK (yellow)

// Internal to the Dial.
constexpr int kPinEncA = 41;
constexpr int kPinEncB = 40;
