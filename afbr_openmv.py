# afbr_openmv.py - read the MIKROE-6239 (AFBR-S50MX85I) from an OpenMV camera over UART.
#
# The board must run the UART build of the AFBR-S50 Explorer App v1.5.6 (ReleaseUART
# configuration of Projects/e2Studio/AFBR_S50_ExplorerApp_RA4M2; SCI0 on P100/P101, 115200 8N1).
# Wiring (3.3 V logic): board TX -> OpenMV UART RX, board RX -> OpenMV UART TX, GND -> GND.
#
# Copy this file to the OpenMV drive and run it from the OpenMV IDE. It is plain MicroPython
# (no OpenMV-only modules) and also runs under CPython for testing.
import struct
import time

UART_ID = 3          # OpenMV UART number wired to the board; check the N6 pinout
BAUDRATE = 115200    # Explorer App UART default (UART_BAUDRATE in board_config.h)
OUTPUT = "3d"        # "1d", "3d" or "full"
MODE = "long-range"  # see MEASUREMENT_MODES; None keeps the device setting
FRAME_TIME_US = 100000

START, STOP, ESC = 0x02, 0x03, 0x1B
CMD_ACK, CMD_NAK = 0x0A, 0x0B
CMD_SOFTWARE_INFO = 0x05
CMD_START, CMD_STOP = 0x11, 0x12
CMD_OUTPUT_MODE, CMD_MEAS_MODE, CMD_FRAME_TIME = 0x41, 0x42, 0x43
CMD_DATA_FULL, CMD_DATA_3D, CMD_DATA_1D = 0x32, 0x34, 0x36
DATA_CMDS = (0x31, 0x32, 0x33, 0x34, 0x35, 0x36)
OUTPUT_MODES = {"full": 3, "3d": 5, "1d": 7}
MEASUREMENT_MODES = {"short-range": 0x01, "long-range": 0x02,
                     "high-speed-short-range": 0x05, "high-speed-long-range": 0x06}
PIXEL_BAD = 0x08 | 0x20 | 0x40 | 0x80  # INVALID, NO_SIGNAL, OUT_OF_SYNC, STALLED


def _ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.monotonic() * 1000)


def _elapsed_ms(t0):
    try:
        return time.ticks_diff(time.ticks_ms(), t0)
    except AttributeError:
        return _ticks_ms() - t0


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_frame(cmd, payload=b"", address=None):
    body = bytes([cmd | 0x80, address]) if address is not None else bytes([cmd])
    body += payload
    body += bytes([crc8(body)])
    out = bytearray([START])
    for b in body:
        if b in (START, STOP, ESC):
            out.append(ESC)
            out.append(~b & 0xFF)
        else:
            out.append(b)
    out.append(STOP)
    return bytes(out)


class Decoder:
    """Byte-stuffing decoder; yields (cmd, payload) and counts bad frames in .errors."""

    def __init__(self):
        self.buf = None
        self.esc = False
        self.errors = 0

    def feed(self, data):
        frames = []
        for b in data:
            if self.esc:
                self.esc = False
                if b == START:
                    self.errors += 1
                    self.buf = bytearray()
                    continue
                b = ~b & 0xFF
            elif b == ESC:
                self.esc = self.buf is not None
                continue
            elif b == START:
                if self.buf:
                    self.errors += 1
                self.buf = bytearray()
                continue
            elif b == STOP:
                if self.buf is not None:
                    f = self._finish(bytes(self.buf))
                    if f is None:
                        self.errors += 1
                    else:
                        frames.append(f)
                    self.buf = None
                continue
            if self.buf is not None:
                self.buf.append(b)
        return frames

    @staticmethod
    def _finish(raw):
        if len(raw) < 2 or crc8(raw[:-1]) != raw[-1]:
            return None
        cmd = raw[0]
        if cmd & 0x80:
            return (cmd & 0x7F, raw[2:-1]) if len(raw) >= 3 else None
        return (cmd, raw[1:-1])


def _s24(b, i):
    v = (b[i] << 16) | (b[i + 1] << 8) | b[i + 2]
    return v - 0x1000000 if v & 0x800000 else v


def _range_mm(raw):
    return None if raw == 0x7FFFFF else raw * 1000.0 / 16384


def _xy_to_channel(x, y):
    return ((y << 3) & 0x10) | ((x ^ 0x07) << 1) | (y & 0x01)


def parse_measurement(cmd, p):
    """Parses 0x32/0x34/0x36 data frames (Serialize_MeasurementData() in v1.5.6).

    Returns a dict: status, time_s, and range_1d_mm/amplitude_1d/quality (1D, full) and/or
    pixels = [(x, y, status, range_mm or None, amplitude), ...] (3D, full)."""
    status, sec, usec16 = struct.unpack(">hIH", p[:8])
    m = {"cmd": cmd, "status": status, "time_s": sec + usec16 * 16 / 1e6}
    if cmd == CMD_DATA_1D:
        i = 12 if len(p) >= 18 else 10
        _parse_1d(p, i, m)
        return m
    pxen = struct.unpack(">I", p[19:23])[0]
    i = 27
    coords = [(x, y) for x in range(8) for y in range(4) if pxen & (1 << _xy_to_channel(x, y))]
    tail = 20 if cmd == CMD_DATA_FULL else 0
    n = (len(p) - i - tail) // 6
    if n not in (len(coords), len(coords) + 1):
        raise ValueError("pixel count mismatch")
    pixels = []
    for k, (x, y) in enumerate(coords):
        amp = struct.unpack(">H", p[i + 4 * n + 2 * k:i + 4 * n + 2 * k + 2])[0] / 16.0
        pixels.append((x, y, p[i + k], _range_mm(_s24(p, i + n + 3 * k)), amp))
    m["pixels"] = pixels
    m["pixel_mask"] = pxen
    if cmd == CMD_DATA_FULL:
        _parse_1d(p, i + 6 * n, m)
    return m


def _parse_1d(p, i, m):
    m["range_1d_mm"] = _range_mm(_s24(p, i))
    m["amplitude_1d"] = struct.unpack(">H", p[i + 3:i + 5])[0] / 16.0
    m["quality"] = p[i + 5]


def _version(v):
    return "%d.%d.%d" % (v >> 24, (v >> 16) & 0xFF, v & 0xFFFF)


class NakError(Exception):
    pass


class AFBR:
    def __init__(self, uart, address=None, timeout_ms=1000, retries=2, debug=False):
        self.uart = uart
        self.address = address
        self.timeout_ms = timeout_ms
        self.retries = retries
        self.debug = debug
        self.dec = Decoder()
        self.data = []
        self.replies = []

    def _poll(self):
        chunk = self.uart.read()
        if not chunk:
            return
        for cmd, payload in self.dec.feed(chunk):
            if self.debug:
                print("RX", hex(cmd), payload)
            if cmd in DATA_CMDS:
                self.data.append((cmd, payload))
                if len(self.data) > 4:
                    self.data.pop(0)  # keep only the newest frames if the script falls behind
            else:
                self.replies.append((cmd, payload))

    def transact(self, cmd, payload=b"", want_response=False):
        for attempt in range(self.retries + 1):
            self.replies = []
            self.uart.write(encode_frame(cmd, payload, self.address))
            acked, response, t0 = False, None, _ticks_ms()
            while _elapsed_ms(t0) < self.timeout_ms:
                self._poll()
                while self.replies:
                    rcmd, rp = self.replies.pop(0)
                    if rcmd == CMD_ACK and rp and rp[0] & 0x7F == cmd:
                        acked = True
                    elif rcmd == CMD_NAK and rp and rp[0] & 0x7F in (cmd, 0):
                        reason = struct.unpack(">h", rp[1:3])[0] if len(rp) >= 3 else None
                        raise NakError("command 0x%02X rejected, status %s" % (cmd, reason))
                    elif rcmd == cmd:
                        response = rp
                if acked and (response is not None or not want_response):
                    return response
                time.sleep_ms(1) if hasattr(time, "sleep_ms") else time.sleep(0.001)
        raise OSError("timeout waiting for reply to command 0x%02X" % cmd)

    def software_info(self):
        p = self.transact(CMD_SOFTWARE_INFO, want_response=True)
        explorer, api, count = struct.unpack(">IIB", p[:9])
        devices = []
        for k in range(count):
            e = p[9 + 7 * k:16 + 7 * k]
            devices.append({"address": e[0], "module": e[1], "chip": e[2], "laser": e[3],
                            "uid": (e[4] << 16) | (e[5] << 8) | e[6]})
        text = bytes(p[9 + 7 * count:]).decode().rstrip("\0")
        return {"explorer": _version(explorer), "api": _version(api), "devices": devices, "id": text}

    def configure(self, output="3d", mode="long-range", frame_time_us=100000):
        if mode:
            try:
                self.transact(CMD_MEAS_MODE, bytes([MEASUREMENT_MODES[mode]]))
            except NakError as e:
                print("WARN: measurement mode not set:", e)
        self.transact(CMD_OUTPUT_MODE, bytes([OUTPUT_MODES[output]]))
        self.transact(CMD_FRAME_TIME, struct.pack(">I", frame_time_us))

    def start(self):
        self.transact(CMD_START)

    def stop(self):
        self.transact(CMD_STOP)

    def read(self, timeout_ms=1000):
        """Returns the next parsed measurement dict, or None on timeout."""
        t0 = _ticks_ms()
        while True:
            self._poll()
            while self.data:
                cmd, payload = self.data.pop(0)
                if cmd in (CMD_DATA_1D, CMD_DATA_3D, CMD_DATA_FULL):
                    return parse_measurement(cmd, payload)
            if _elapsed_ms(t0) >= timeout_ms:
                return None


def summary(m):
    """One-line text: 1D distance if present, else the mean of the valid pixels."""
    if "range_1d_mm" in m and m["range_1d_mm"] is not None:
        text = "distance %.0f mm  amp %.1f  quality %d%%" % (m["range_1d_mm"], m["amplitude_1d"], m["quality"])
    else:
        text = "distance n/a"
    if "pixels" in m:
        valid = [r for (_, _, st, r, _) in m["pixels"] if r is not None and not st & PIXEL_BAD]
        if valid and "range_1d_mm" not in m:
            text = "distance %.0f mm (mean)" % (sum(valid) / len(valid))
        text += "  pixels sent %d of 32, %d valid" % (len(m["pixels"]), len(valid))
    return text


def main():
    from machine import UART
    uart = UART(UART_ID, BAUDRATE, timeout=0, timeout_char=0)
    s = AFBR(uart)
    try:
        s.stop()  # in case measurements are still running from an earlier run
    except (OSError, NakError):
        pass
    info = s.software_info()
    print("Explorer App", info["explorer"], "API", info["api"], "-", info["id"])
    for d in info["devices"]:
        print("  device", d["address"], "module 0x%02X chip 0x%02X UID 0x%06X" % (d["module"], d["chip"], d["uid"]))
    s.configure(OUTPUT, MODE, FRAME_TIME_US)
    s.start()
    try:
        while True:
            m = s.read()
            print(summary(m) if m else "no data")
    finally:
        s.stop()


if __name__ == "__main__":
    main()
