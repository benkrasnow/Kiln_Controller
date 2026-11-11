// Connection to the kiln controller: newline-delimited JSON in both
// directions. Requests get an "id" that the firmware echoes in its reply;
// status events ({"evt":"status",...}) arrive unsolicited once a second.
//
// Events (all CustomEvent, payload in .detail):
//   "status"     - parsed status object
//   "event"      - any other {"evt":...} message (program_done, ...)
//   "line"       - {dir: "rx"|"tx", text} for the console
//   "disconnect" - the link went away (unplugged, closed, or failed)

const REQUEST_TIMEOUT_MS = 2000;
const ESPRESSIF_VID = 0x303a;

export class Link extends EventTarget {
  #nextId = 1;
  #pending = new Map();
  #rxBuffer = "";

  get connected() {
    return false;
  }

  // Host-side clock used to timestamp samples (overridden by the demo).
  now() {
    return Date.now();
  }

  request(cmd, params = {}) {
    if (!this.connected) return Promise.reject(new Error("not connected"));
    const id = this.#nextId++;
    const text = JSON.stringify({ id, cmd, ...params });
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.#pending.delete(id);
        reject(new Error(`no reply to "${cmd}"`));
      }, REQUEST_TIMEOUT_MS);
      this.#pending.set(id, { resolve, reject, timer });
      this.emit("line", { dir: "tx", text });
      this.writeLine(text).catch((err) => {
        clearTimeout(timer);
        this.#pending.delete(id);
        reject(err);
      });
    });
  }

  // Subclasses call this with received text.
  receive(chunk) {
    this.#rxBuffer += chunk;
    let nl;
    while ((nl = this.#rxBuffer.search(/\r?\n/)) >= 0) {
      const line = this.#rxBuffer.slice(0, nl).trim();
      this.#rxBuffer = this.#rxBuffer.slice(this.#rxBuffer[nl] === "\r" ? nl + 2 : nl + 1);
      if (line) this.#handleLine(line);
    }
    if (this.#rxBuffer.length > 4096) this.#rxBuffer = "";  // garbage, resync
  }

  #handleLine(line) {
    this.emit("line", { dir: "rx", text: line });
    let msg;
    try {
      msg = JSON.parse(line);
    } catch {
      return;  // boot noise or a partial line
    }
    if (msg.evt === "status") {
      this.emit("status", msg);
    } else if (msg.evt) {
      this.emit("event", msg);  // e.g. program_done, program_aborted
    } else if (msg.id != null && this.#pending.has(msg.id)) {
      const { resolve, reject, timer } = this.#pending.get(msg.id);
      clearTimeout(timer);
      this.#pending.delete(msg.id);
      msg.ok ? resolve(msg) : reject(new Error(msg.error || "request failed"));
    }
  }

  linkLost() {
    for (const { reject, timer } of this.#pending.values()) {
      clearTimeout(timer);
      reject(new Error("disconnected"));
    }
    this.#pending.clear();
    this.#rxBuffer = "";
    this.emit("disconnect", {});
  }

  emit(type, detail) {
    this.dispatchEvent(new CustomEvent(type, { detail }));
  }
}

export class SerialLink extends Link {
  #port = null;
  #reader = null;
  #writer = null;
  #closing = false;

  static supported() {
    return "serial" in navigator;
  }

  get connected() {
    return this.#port !== null;
  }

  // Ports this site was given permission for earlier (no picker needed).
  static async knownPorts() {
    const ports = await navigator.serial.getPorts();
    return ports.filter((p) => p.getInfo().usbVendorId === ESPRESSIF_VID);
  }

  // Shows the browser's port picker, filtered to Espressif devices.
  static async pickPort() {
    return navigator.serial.requestPort({ filters: [{ usbVendorId: ESPRESSIF_VID }] });
  }

  async open(port) {
    // The baud rate is ignored by USB CDC, but never use 1200: the firmware
    // treats a 1200-baud open as "reboot into the bootloader".
    await port.open({ baudRate: 115200 });
    // Assert DTR: the firmware's USB stack drops output while DTR is low.
    await port.setSignals({ dataTerminalReady: true, requestToSend: true }).catch(() => {});
    this.#port = port;
    this.#writer = port.writable.getWriter();
    this.#readLoop();
  }

  async writeLine(text) {
    await this.#writer.write(new TextEncoder().encode(text + "\n"));
  }

  async #readLoop() {
    const decoder = new TextDecoder();
    try {
      while (this.#port?.readable) {
        this.#reader = this.#port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await this.#reader.read();
            if (done) break;
            this.receive(decoder.decode(value, { stream: true }));
          }
        } finally {
          this.#reader.releaseLock();
          this.#reader = null;
        }
        if (this.#closing) break;
      }
    } catch {
      // Device unplugged or port failed; fall through to cleanup.
    }
    await this.#cleanup();
  }

  async close() {
    if (!this.#port) return;
    this.#closing = true;
    await this.#reader?.cancel().catch(() => {});
    // #readLoop's cleanup finishes the job.
  }

  async #cleanup() {
    const port = this.#port;
    if (!port) return;
    this.#port = null;
    try {
      this.#writer?.releaseLock();
    } catch {}
    this.#writer = null;
    await port.close().catch(() => {});
    this.#closing = false;
    this.linkLost();
  }
}
