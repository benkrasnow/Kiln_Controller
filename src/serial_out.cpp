#include "serial_out.h"

#include <Arduino.h>
#include <esp32-hal-tinyusb.h>  // tud_cdc_n_connected

namespace serial_out {
namespace {

constexpr size_t kQueueBytes = 4096;

char s_buf[kQueueBytes];
size_t s_head = 0;  // next byte to send
size_t s_used = 0;
unsigned long s_dropped = 0;

void put(const char* data, size_t len) {
  const size_t tail = (s_head + s_used) % kQueueBytes;
  const size_t first = len < kQueueBytes - tail ? len : kQueueBytes - tail;
  memcpy(s_buf + tail, data, first);
  memcpy(s_buf, data + first, len - first);
  s_used += len;
}

}  // namespace

bool sendLine(const char* text, size_t len) {
  if (len + 1 > kQueueBytes - s_used) {
    ++s_dropped;
    return false;
  }
  put(text, len);
  put("\n", 1);
  return true;
}

void pump() {
  // No host has the port open (DTR low): stale data is useless. Use
  // TinyUSB's own check; the Arduino `Serial` flag only becomes true if DTR
  // and RTS change in one particular order, which hosts don't guarantee.
  if (!tud_cdc_n_connected(0)) {
    s_head = s_used = 0;
    return;
  }
  while (s_used > 0) {
    // Never ask USBCDC::write for more than it has room for: then it can't spin.
    const int room = Serial.availableForWrite();
    if (room <= 0) return;
    size_t n = s_used < static_cast<size_t>(room) ? s_used : static_cast<size_t>(room);
    if (n > kQueueBytes - s_head) n = kQueueBytes - s_head;  // contiguous part
    const size_t sent = Serial.write(reinterpret_cast<const uint8_t*>(s_buf + s_head), n);
    if (sent == 0) return;
    s_head = (s_head + sent) % kQueueBytes;
    s_used -= sent;
  }
}

unsigned long dropped() { return s_dropped; }

}  // namespace serial_out
