# Kiln Controller — Design Proposal

**Target:** M5Stack Dial v1.1 (K130-V11, Stamp-S3A / ESP32-S3FN8, 8 MB flash, no PSRAM), MAX31855 thermocouple amplifier, one GPIO driving a zero-cross SSR.
**Goal:** A standalone kiln controller that runs setpoints and multi-segment ramp/hold programs on its own, with a zero-install browser UI for writing programs and watching firings.

---

## 1. Summary of recommendations

| Topic | Recommendation |
|---|---|
| MCU | Keep the M5Dial. It has a round 240×240 LCD, encoder, button, buzzer, RTC, 6–36 V DC input and native USB, which covers everything this needs. |
| Wiring | **No soldering needed.** The MAX31855 (3 pins) and the SSR (1 pin) fit on the Dial's two Grove ports (4 GPIOs). Solder to a spare Stamp pad (G39, G43, G44) only for the optional safety contactor. |
| Sensor board | Use a MAX31855 **breakout with an onboard 3.3 V regulator** (e.g. Adafruit 269 style), powered from the Grove 5 V. Consider a **MAX31856** if you want mains-noise filtering and any thermocouple type (Section 3.3). |
| Computer link | **Web Serial** from a single-page app on any Chromium browser (Chrome, Edge, Brave, Opera, Arc) on Windows, macOS, Linux or ChromeOS. The ESP32-S3's native USB is a class-compliant CDC device, so no driver is needed. The page is hosted on HTTPS (GitHub Pages) and the same page can also flash firmware. |
| Wi-Fi | Optional later phase. Wi-Fi credentials are entered **over the same USB page** (Improv-Serial), which solves the "how do I enter the password" problem. The device then serves the same UI at `http://kiln.local` for phone monitoring. |
| Autonomy | The device owns all state. The PC is only a viewer and editor. Programs, the running state and the firing log live on the device, and a power-blip resume is supported. |
| Safety | Add a **mechanical contactor** in series with the SSR on a second GPIO. SSRs usually fail shorted, which in a kiln means full power with no way to stop it. Firmware runs runaway, stuck-relay, thermocouple-fault and over-temperature checks that open the contactor. |
| Power | Power the Dial from its **own supply** (12–24 V DIN-rail PSU into the DC terminal, or a USB wall charger). **Never** power it only from the laptop, or unplugging the laptop stops the firing. |

---

## 2. Pin analysis

### 2.1 What the Dial already uses (from M5Stack docs, esp-bsp `m5dial.h`, M5Unified pin tables)

The Stamp-S3A breaks out 23 GPIOs: **G0–G15, G39–G44, G46**.

| GPIO | Use in Dial | Available? |
|---|---|---|
| G0 | BOOT button on the Stamp (download mode; M5Unified reads it as a second button) | No (strapping) |
| **G1** | **Port B pin** (M5 convention: input) | **Yes, on the Grove connector** |
| **G2** | **Port B pin** (M5 convention: output) | **Yes, on the Grove connector** |
| G3 | Buzzer | No (also a strapping pin) |
| G4 | LCD DC | No |
| G5 | LCD MOSI | No |
| G6 | LCD SCK | No |
| G7 | LCD CS | No |
| G8 | LCD RST (docs also list it near the RFID signals) | No |
| G9 | LCD backlight | No |
| G10 | RFID (WS1850S) signal, IRQ per the docs' grouping | No (driven by the RFID chip) |
| G11 / G12 | Internal I²C SDA / SCL (touch FT3267, RTC BM8563, RFID) | No |
| **G13** | **Port A** (external I²C SDA in M5Unified) | **Yes, on the Grove connector** |
| G14 | Touch INT | No |
| **G15** | **Port A** (external I²C SCL in M5Unified) | **Yes, on the Grove connector** |
| **G39** | Not used | **Yes, solder to the Stamp pad** |
| G40 / G41 | Encoder B / A | No |
| G42 | Front (dial) push button | No |
| **G43** | Not used by the Dial (UART0 TX: the ROM prints a boot log here) | **Yes, solder** (see note) |
| **G44** | Not used by the Dial (UART0 RX) | **Yes, solder** |
| G46 | Power HOLD (must be driven high or battery power cuts out) | No |
| G21, G38 | Internal to the Stamp-S3A (RGB LED, LED power switch), not broken out | No |
| G19 / G20 | Native USB D− / D+ | No |

A few of these assignments (G8 and G10 around the RFID chip) are inferred from the docs' grouping. Check them against `Sch_M5Dial.pdf` before soldering. None of them affect the recommended wiring below, which uses only the Grove ports.

### 2.2 Recommended wiring (no soldering)

The MAX31855 is read-only SPI, so it needs only **SCK, CS and MISO (SO)**. Together with the SSR that is exactly the four Grove GPIOs.

| Function | Dial pin | Connector | Why this pin |
|---|---|---|---|
| **SSR drive** | **G2** | Port B | Not a strapping pin, and M5 already treats it as an output. Port B has no I²C pull-ups, so the line cannot float high and **turn the kiln on during boot or reset**. Add a 10 kΩ pull-down at the SSR driver anyway. |
| MAX31855 **SO / MISO** | **G1** | Port B | Input-only use, which matches the Port B convention. |
| MAX31855 **CS** | **G15** | Port A | If Port A has I²C pull-ups, they hold CS high (deselected) while booting. That is the correct idle state. |
| MAX31855 **SCK** | **G13** | Port A | A pull-up on SCK is harmless at the ≤1 MHz clock used here. |
| MAX31855 VIN / GND | 5 V / GND | Port A | The breakout's regulator makes 3.3 V. Its SO output is 3.3 V and safe for the ESP32-S3. |

> **As built:** the Grove cables' wire colours turned out reversed from M5Stack's table, and the wiring followed the colours. The firmware was changed to match: **SSR on G1, SO on G2, CS on G13, SCK on G15**. Both SSR pins are on Port B, so the boot-safety reasoning above still holds. See [README.md](README.md) for the current table.

The SPI bus is routed through the GPIO matrix to **SPI2 (FSPI)**, since the LCD uses SPI3. At these speeds a bit-banged read works equally well.

**SSR drive circuit:** The GPIO output is 3.3 V and roughly 20 mA. Most large DC-input SSRs (3–32 VDC input) switch fine from 3.3 V, but a small NPN or logic-level MOSFET low-side switch running the SSR input from the Grove **5 V** gives more margin. It also keeps SSR wiring faults away from the MCU and lets the 10 kΩ gate/base pull-down guarantee *off* during reset.

### 2.3 Optional: safety contactor and spare pins (soldered)

| Function | Pin | Notes |
|---|---|---|
| **Contactor / safety relay enable** | **G39** | Free, not a strapping pin, and floats during reset. Use a driver with a pull-down so the contactor drops out whenever the MCU is not running. |
| Spare (e.g. door switch, second thermocouple CS) | G44 | Free. |
| Spare | G43 | Free, but the ROM boot log toggles it at power-up. Use it only for inputs or as a CS for a device that tolerates a few junk conversions. **Never** use it for heater or contactor drive. |

If you would rather not solder at all, you can move the MAX31855 to G39/G43/G44 and keep both Grove ports for SSR plus contactor. The no-solder layout in 2.2 is simpler, though, and the contactor can come later.

---

## 3. Hardware notes and suggestions

### 3.1 Power
- Feed the Dial's **DC terminal with 12 V or 24 V** from a small DIN-rail supply in the controller box. The same rail can drive the contactor coil.
- With USB also plugged in, the Dial should run from whichever source is higher. Before relying on it, check that USB and DC together don't backfeed (a quick multimeter test on the bench).
- The Dial's battery connector can hold a small LiPo so the controller rides through mains blips and logs the outage. The kiln can't heat without mains anyway, but the controller remembers where it was (Section 4.6).

### 3.2 Switching
- Use a **zero-cross SSR** rated at **≥2× element current**, on a real heatsink. Undersized or unsinked SSRs are the most common failure in DIY kiln controllers.
- Add the **contactor** (Section 2.3). Firmware closes it at the start of a firing and opens it at the end or on any fault, so the kiln is never "SSR-only".
- Keep the independent mechanical safeties: a correctly sized breaker or fuse and, ideally, a high-limit thermal cutoff.

### 3.3 Thermocouple and amplifier
- **MAX31855K + type K** covers about 1260 °C continuous (cone 6 is fine). For frequent cone 10 firings, consider **type N** (MAX31855N) or **type S** (MAX31855S). Use a **ceramic-sheathed, ungrounded** thermocouple: the MAX31855 flags a fault if the junction touches ground.
- At high temperature, kiln element insulation leaks into the thermocouple and injects mains noise. Put a **10 nF differential capacitor** across T+/T− at the board, use twisted or shielded thermocouple extension wire, and median-filter the readings in firmware.
- The MAX31855 assumes a linear 41.276 µV/°C. Firmware should back out the hot-junction voltage and apply the **NIST inverse polynomial** to remove errors of several degrees at high temperature.
- Cold-junction compensation uses the chip's own die temperature. Mount the board where the thermocouple wire meets copper, **away from heat** (not on the kiln shell).
- **Upgrade option: MAX31856.** It is the same footprint class and needs one more pin (MOSI). It has 50/60 Hz rejection, 19-bit resolution, any thermocouple type in software, and over- and under-voltage fault detection. For a kiln it is noticeably more robust.

---

## 4. Firmware architecture

### 4.1 Toolchain
- **PlatformIO + Arduino-ESP32 3.x** with **M5Unified / M5GFX** (or the `M5Dial` library) for display, encoder, button, buzzer and RTC. ESP-IDF directly with `esp-bsp/m5dial` is an equally valid choice. Arduino gets to a working device faster.
- **USB mode:** use **TinyUSB CDC (USB-OTG mode)**, not the USB-Serial/JTAG peripheral. The S3's USB-Serial/JTAG can **reset the chip** on certain DTR/RTS sequences, and a browser opening the port must never be able to reboot the controller mid-firing. Firmware updates still work by holding G0 at plug-in, and the web page can trigger a reboot-to-bootloader command.
  - *Found during Phase 0:* the Arduino TinyUSB CDC stack has its own reboot-to-bootloader trigger: a 1200-baud open, or the esptool DTR/RTS sequence. The firmware enables it (`Serial.enableReboot`) only while the heater is disarmed. After that reboot, the ESP32-S3's force-download flag (`RTC_CNTL_OPTION1`) stays set until it is cleared or power is removed, so the upload script and a future web flasher must clear it.

### 4.2 Tasks (FreeRTOS)

| Task | Rate | Job |
|---|---|---|
| `sensor` | 10 Hz | Read the MAX31855 (100 ms conversion), linearize, median-of-5 filter, fault debounce. |
| `control` | 1 Hz | Program executor → setpoint → PID → duty → time-proportioning SSR output. Runs the safety checks. Highest priority, watched by the task WDT. |
| `ui` | ~20 fps | Draws the round display and handles the encoder, button and touch. |
| `comms` | event | Line-based JSON protocol over USB CDC (later also WebSocket). |
| `storage` | event | NVS writes for programs, config and the run checkpoint. Log ring buffer. |

The control loop has **no dependency** on `comms` or `ui`. A dead host, a hung UI or a USB unplug cannot affect heating.

### 4.3 Power output (slow on-off constraint)
- **Time-proportioning window.** The window is configurable from 1–20 s, default 4 s. Each window has at most **one on period and one off period**, which meets the "at most one cycle per second" constraint.
- **Minimum on/off time** is 0.5 s. Duties below the minimum are skipped, and duties above (window − minimum) are forced to 100 %. Errors carry forward to the next window so the average power stays exact.
- Output is computed at the window boundary and the GPIO changes only at window edges. A 4 s window is negligible against a kiln's thermal time constant (minutes) and keeps SSR heating and EMI low.

### 4.4 Control law
- **PID on temperature**, with derivative on measurement (not error), integrator clamping and back-calculation anti-windup.
- **Ramp feed-forward.** During a ramp segment, add duty proportional to the ramp rate, learned or configured. This removes most of the lag that a plain PID shows on long ramps.
- **Gain scheduling** (optional): kilns need more power for the same ramp at high temperature, because losses grow roughly with T⁴. Use two or three gain sets by temperature band, or let the feed-forward term carry it.
- **Autotune** (phase 2): a relay-feedback test around a user-chosen temperature, with gains derived and saved to NVS.

### 4.5 Programs
- A program is a name plus up to about 16 segments. Each segment is `{ rate (°/h, or "FULL"), target (°), hold (min) }`.
- Store about 10 program slots in NVS. They are selectable on the Dial, so a firing can start **without a computer**.
- **Executor behaviour:**
  - ~~The setpoint ramps from the *actual* temperature at start~~ *(Changed during development: programs always start at 0:00, and waiting only triggers when the kiln is behind the profile, so a warm kiln never holds the clock up.)*
  - **Guaranteed ramp/soak:** if actual lags the setpoint by more than a configurable band (default 15 °C), the program clock **pauses** until the kiln catches up, and the UI shows "waiting". Hold timers start only once the target is reached.
  - Cooling segments (rate < 0) work the same way. "Natural cool" segments just switch heat off.
  - Simple **setpoint mode** is a single open-ended segment. The encoder changes the target live.
- Optional built-in presets: bisque and glaze firings for common cones.

### 4.6 Autonomy, persistence and resume
- Every 60 s, and on every state change, checkpoint `{program id, segment, segment elapsed, phase, start timestamp}` to NVS. NVS wear-levels, and one write per minute over a 12-hour firing is trivial.
- At boot, if a checkpoint shows a firing in progress: when the kiln is still within a configurable band of where it should be, or the outage was shorter than N minutes (measured with the BM8563 RTC), **resume**. Otherwise **abort safely** and show what happened.
- The firing log is a ring buffer (time, PV, SP, duty, segment) sampled every 10–30 s, kept in RAM and flushed to a LittleFS file. A full firing is under 100 KB. **When the browser reconnects it downloads the log, so the chart is complete even if the PC was off for the whole firing.**

### 4.7 Safety supervisor (runs inside `control`)

| Condition | Action |
|---|---|
| Thermocouple open, short-to-GND or short-to-VCC for more than 3 s; implausible readings; sensor task stale for more than 3 s | Heat off, contactor open, alarm. |
| PV > `max_temp` (user setting, e.g. 1300 °C) | Heat off, contactor open, alarm. |
| **Runaway:** duty at 100 % for more than N minutes with PV rising less than X °C | Heat off. Probable causes: failed element, open SSR, or thermocouple pulled out of the kiln. |
| **Stuck SSR:** duty 0 % but PV rising for more than N minutes | Contactor open, alarm. |
| Total firing time > limit | End the program. |
| Watchdog reset or brownout | The GPIO resets low and the pull-downs hold the SSR and contactor off, then the resume logic applies. |

Alarms use the buzzer, a red screen, a persistent fault log and an event message to the host.

---

## 5. On-device UI (round 240×240, encoder, button, touch)

- **Home screen:** large actual temperature in the centre, setpoint on an outer arc, power % as a ring gauge, program and segment name, and time remaining or "waiting for kiln". A USB or Wi-Fi icon shows when a host is attached.
- **Rotate** to adjust the setpoint in setpoint mode. The change takes effect only after a **press to confirm** (or about 3 s of no movement), so a bumped knob doesn't change the firing.
- **Press** opens the menu: Start program ▸ (list of stored programs), Setpoint mode, Stop, Settings (units °C/°F, window, max temp, offsets).
- **Long-press (2 s)** means **Stop / heat off** from any screen. This is the universal escape.
- Alarms show full-screen. A press silences the buzzer but keeps the screen until the fault is acknowledged.
- Settings need no computer, but anything with lots of fields (programs, PID gains) is meant for the web UI.

---

## 6. Computer interface

### 6.1 Transport: Web Serial (primary)
- A single static page (HTML/JS, no backend) is hosted at e.g. `https://<you>.github.io/kiln`. Chromium browsers expose the **Web Serial API** on HTTPS pages: the user clicks **Connect**, picks "Kiln Controller", and that's it. Nothing is installed on Windows 10+, macOS or Linux, because the ESP32-S3 TinyUSB CDC uses the OS's built-in class driver.
- **Browser support:** Chrome, Edge, Opera, Brave and Arc on desktop. Firefox and Safari don't support Web Serial, so the page detects this and says so. The page can be made a **PWA** so it works offline after the first visit and can be "installed" as an app icon if wanted (optional, not required).
- **Firmware updates from the same page** use **esptool-js / ESP Web Tools**. The user doesn't need PlatformIO either.
- Set the USB descriptor strings (manufacturer "…", product "Kiln Controller") so the port picker shows a friendly name. Filter `requestPort()` by VID/PID so only the kiln appears.

### 6.2 Protocol: newline-delimited JSON
Human-readable, so it can be debugged in any serial terminal and versioned.

```
→ {"id":1,"cmd":"hello"}
← {"id":1,"ok":true,"fw":"1.0.0","proto":1,"units":"C","uptime":1234}

← {"evt":"status","t":1731000000,"pv":842.5,"sp":850.0,"duty":0.62,
   "state":"running","prog":"Cone 6 glaze","seg":3,"seg_left_s":2710,
   "waiting":false,"faults":[]}           (pushed every 1 s)

→ {"id":2,"cmd":"program.put","slot":2,"program":{"name":"Cone 6 glaze",
   "segments":[{"rate":100,"target":600,"hold":0},
               {"rate":300,"target":1200,"hold":0},
               {"rate":60,"target":1222,"hold":10}]}}
→ {"id":3,"cmd":"run.start","slot":2,"delay_min":0}
→ {"id":4,"cmd":"run.stop"}
→ {"id":5,"cmd":"setpoint","value":500}
→ {"id":6,"cmd":"log.get","since":1730990000}    (chunked reply)
→ {"id":7,"cmd":"config.get"} / {"cmd":"config.set",...}
→ {"id":8,"cmd":"wifi.provision",...}             (or Improv-Serial)
```

- Every command is **idempotent or explicit**. The host never streams the setpoint. It uploads a program and says "start", and the device does the rest. A disconnected host therefore cannot leave the kiln in a half-commanded state.
- Commands that change heating (`run.start`, `setpoint`, `config.set` of safety limits) are also confirmed on the Dial's screen with a toast, so the operator at the kiln sees what a remote user did.

### 6.3 Web app features
- **Live dashboard:** PV/SP/duty chart (uPlot or Chart.js) with the planned profile overlaid, segment progress and ETA.
- **Program editor:** a table of segments plus a live preview graph with total duration. Import and export as JSON, a "library" stored in the browser's localStorage, and upload to any device slot.
- **Firing history:** download the log from the device, view it, and export as CSV.
- **Settings:** PID, window, limits and thermocouple offset, with an autotune button (phase 2).
- **Firmware update** button.
- A transport abstraction (`SerialTransport`, later `WebSocketTransport`) so the same app works over Wi-Fi.

### 6.4 Wi-Fi (phase 2, optional)
- **Provisioning over USB:** the web page sends SSID and password with the **Improv-Serial** standard, which ESP Web Tools already implements. No captive portal, and no typing a password with a knob.
- Once connected, the device serves the **same SPA from its flash** at `http://kiln.local` (mDNS) and uses a WebSocket with the same JSON protocol. That lets a phone or tablet watch a firing from the house.
- Wi-Fi is **monitor-first**. Starting a program remotely can be allowed but gated by a setting, so nobody starts a kiln without someone checking it in person. Optionally, a notification is sent on completion or alarm (ntfy.sh or webhook).
- Hold-off on cloud services. A local network plus USB keeps it zero-account and zero-install.

---

## 7. Development phases

| Phase | Deliverable | Exit criterion |
|---|---|---|
| **0. Bring-up** | PlatformIO project, display/encoder/button/buzzer, MAX31855 read with linearization, SSR toggle on G2 | Correct temperature shown, SSR LED blinks at the window rate, SSR stays OFF through reset and boot (scope check) |
| **1. Setpoint control** | Time-proportioning output, PID, safety supervisor, home screen, encoder setpoint | Holds a setpoint ±3 °C in a test kiln, or a "toaster-oven" bench rig first |
| **2. Programs** | Segment executor, guaranteed soak, NVS program slots, on-device program select, checkpoint and resume | A multi-segment program completes with USB unplugged, and resumes after a power cycle mid-ramp |
| **3. Web Serial app** | Static SPA on GitHub Pages: connect, dashboard, program editor, log download | Program written on a PC, fired, and the full log retrieved after reconnecting at the end |
| **4. Polish** | Ramp feed-forward, autotune, firmware update via the web page, PWA, presets | — |
| **5. Wi-Fi (optional)** | Improv provisioning, mDNS, on-device web UI over WebSocket | Phone shows a live firing on the LAN |

**Testing strategy:** keep the controller core (executor, PID, safety supervisor) in plain C++ with no hardware dependencies, and unit-test it on the PC (PlatformIO `native` env) against a simple first-order-plus-dead-time kiln model. That lets whole 12-hour firings be simulated in milliseconds, including fault injection (thermocouple open, stuck SSR, power loss mid-segment).

---

## 8. Open questions for you
1. What is the maximum firing temperature (cone 6 or cone 10)? It decides the thermocouple type and possibly MAX31855K vs N/S vs MAX31856.
2. Kiln voltage and current: is it single element bank and single SSR, or two legs (240 V split-phase with both legs switched)?
3. Is a series contactor acceptable in your build? (Strongly recommended.)
4. °C or °F by default?
5. Should remote start over Wi-Fi ever be allowed, or only monitoring?
