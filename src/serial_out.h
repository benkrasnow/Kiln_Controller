// Non-blocking USB serial output.
//
// The Arduino core's USBCDC::write spins forever when its 64-byte transmit
// buffer is full and the host keeps the port open without reading (a frozen
// browser tab, a forgotten serial monitor). That would freeze the control
// loop with the heater in whatever state it was. Everything the firmware
// sends goes through this queue instead; pump() moves only as many bytes as
// USB can take right now, and lines that don't fit are dropped.
#pragma once

#include <cstddef>

namespace serial_out {

// Queue one line (a trailing '\n' is added). Returns false if it was
// dropped because the queue is full.
bool sendLine(const char* text, size_t len);

// Call every loop iteration.
void pump();

// Lines dropped since boot (for diagnostics).
unsigned long dropped();

}  // namespace serial_out
