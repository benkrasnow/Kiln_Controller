// Bit-banged MAX31855 reader. Bit-banging keeps it off the LCD's SPI host
// and costs ~70 us per read, which is nothing at the 5 Hz read rate.
#pragma once

#include <cstdint>

class Max31855 {
 public:
  Max31855(int sck, int cs, int so) : sck_(sck), cs_(cs), so_(so) {}
  void begin();
  uint32_t readFrame();

 private:
  int sck_, cs_, so_;
};
