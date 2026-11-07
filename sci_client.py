#!/usr/bin/env python3
"""
AFBR-S50 SCI client for the Mikroe MIKROE-6239 board (AFBR-S50MX85I + Renesas RA4M2)
running the AFBR-S50 Explorer App firmware v1.5.6.

Ported from the host-side protocol in Broadcom's AFBR-S50-API (tag v1.5.6):
  Sources/ExplorerApp/sci/*            data link (byte stuffing, CRC8), ACK/NAK
  Sources/ExplorerApp/api/*            command codes and payload serialization
  Doxygen/05_2x_explorer_*.md          SCI frame format and command reference

Frame on the wire:  0x02 | CMD [ADDR] PAYLOAD... CRC | 0x03
  * CMD bit 7 set -> "extended" frame, ADDR byte follows CMD.
  * CRC8_SAE_J1850_ZERO (poly 0x1D, init 0x00) over the unescaped CMD..PAYLOAD.
  * 0x02 / 0x03 / 0x1B inside CMD..CRC are sent as 0x1B followed by the inverted byte.
  * All multi-byte values are big endian.

Usage:
  python sci_client.py                 # auto-detect (COM port with VID 0x276E/PID 0x0301, else pyusb)
  python sci_client.py --port COM5     # explicit serial port
  python sci_client.py --usb           # force raw USB (pyusb + WinUSB/libusb driver)
  python sci_client.py --debug         # dump raw TX/RX frames as hex
"""

import argparse
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Union

USB_VID = 0x276E
USB_PID = 0x0301

START_BYTE = 0x02
STOP_BYTE = 0x03
ESCAPE_BYTE = 0x1B
CONTROL_BYTES = (START_BYTE, STOP_BYTE, ESCAPE_BYTE)
EXTENDED_FLAG = 0x80

# Command codes (7-bit) available in Explorer App v1.5.6 (sci_cmd.h / 05_21_explorer_cmd_overview.md).
CMD_PING = 0x01
CMD_SOFTWARE_INFO = 0x05
CMD_LOG_MESSAGE = 0x06
CMD_ACK = 0x0A
CMD_NAK = 0x0B
CMD_MEASUREMENT_START = 0x11
CMD_MEASUREMENT_STOP = 0x12
CMD_DATA_FULL_DEBUG = 0x31
CMD_DATA_FULL = 0x32
CMD_DATA_3D_DEBUG = 0x33
CMD_DATA_3D = 0x34
CMD_DATA_1D_DEBUG = 0x35
CMD_DATA_1D = 0x36
CMD_CFG_DATA_OUTPUT_MODE = 0x41
CMD_CFG_MEASUREMENT_MODE = 0x42
CMD_CFG_FRAME_TIME = 0x43

MEASUREMENT_DATA_CMDS = (CMD_DATA_FULL, CMD_DATA_3D, CMD_DATA_1D)
DEBUG_DATA_CMDS = (CMD_DATA_FULL_DEBUG, CMD_DATA_3D_DEBUG, CMD_DATA_1D_DEBUG)

# data_output_mode_t (non-debug streaming modes)
OUTPUT_MODES = {"full": 3, "3d": 5, "1d": 7}

# argus_mode_t
MEASUREMENT_MODES = {
    "short-range": 0x01,
    "long-range": 0x02,
    "high-speed-short-range": 0x05,
    "high-speed-long-range": 0x06,
}

PIXELS_X = 8
PIXELS_Y = 4

# argus_px_status_t
PIXEL_STATUS_FLAGS = [
    (0x01, "OFF"), (0x02, "SAT"), (0x04, "BIN_EXCL"), (0x08, "INVALID"),
    (0x10, "PREFILTERED"), (0x20, "NO_SIGNAL"), (0x40, "OUT_OF_SYNC"), (0x80, "STALLED"),
]

# 24-bit range value sent for ARGUS_RANGE_MAX (0x7FFFFFFF >> 8): "no object / out of range".
RANGE_MAX_RAW = 0x7FFFFF

MODULE_NAMES = {
    0x00: "none", 0x01: "AFBR-S50MV85G (v1)", 0x02: "AFBR-S50MV85G (v2)", 0x03: "AFBR-S50LV85D",
    0x04: "AFBR-S50MV68B", 0x05: "AFBR-S50MV85I", 0x06: "AFBR-S50SV85K", 0x07: "AFBR-S50MV85G (v3)",
    0x43: "AFBR-S50LX85D", 0x44: "AFBR-S50MX68B", 0x45: "AFBR-S50MX85I", 0x46: "AFBR-S50SX85K",
    0x47: "AFBR-S50MX85G",
}

STATUS_NAMES = {
    0: "STATUS_OK", 1: "STATUS_IGNORE", 2: "STATUS_BUSY", 3: "STATUS_INITIALIZING",
    -1: "ERROR_FAIL", -2: "ERROR_ABORTED", -3: "ERROR_READ_ONLY", -4: "ERROR_OUT_OF_RANGE",
    -5: "ERROR_INVALID_ARGUMENT", -6: "ERROR_TIMEOUT", -7: "ERROR_NOT_INITIALIZED",
    -8: "ERROR_NOT_SUPPORTED", -9: "ERROR_NOT_IMPLEMENTED",
    100: "STATUS_ARGUS_STARTING", 103: "STATUS_ARGUS_ACTIVE", 104: "STATUS_ARGUS_BUFFER_BUSY",
    105: "STATUS_ARGUS_POWERLIMIT", 106: "STATUS_ARGUS_PLL_NOT_LOCKED", 108: "STATUS_ARGUS_NO_OBJECT",
    109: "STATUS_ARGUS_EEPROM_BIT_ERROR", 110: "STATUS_ARGUS_INVALID_EEPROM",
    -101: "ERROR_ARGUS_NOT_CONNECTED", -102: "ERROR_ARGUS_INVALID_CFG", -103: "ERROR_ARGUS_BUFFER_EMPTY",
    -104: "ERROR_ARGUS_INVALID_SLAVE", -105: "ERROR_ARGUS_INVALID_MODE",
    -107: "ERROR_ARGUS_BIAS_VOLTAGE_REINIT", -109: "ERROR_ARGUS_EEPROM_FAILURE",
    -110: "ERROR_ARGUS_STALLED", -111: "ERROR_ARGUS_BGL_EXCEEDANCE",
    -112: "ERROR_ARGUS_XTALK_AMPLITUDE_EXCEEDANCE", -113: "ERROR_ARGUS_LASER_FAILURE",
    -114: "ERROR_ARGUS_DATA_INTEGRITY_LOST", -191: "ERROR_ARGUS_BUSY",
    -197: "ERROR_ARGUS_UNKNOWN_LASER", -198: "ERROR_ARGUS_UNKNOWN_CHIP", -199: "ERROR_ARGUS_UNKNOWN_MODULE",
    -211: "ERROR_SCI_UNKNOWN_COMMAND", -212: "ERROR_SCI_CMD_EXECUTION_FAILURE",
    -213: "ERROR_SCI_INVALID_CMD_CODE", -214: "ERROR_SCI_INVALID_CMD_PARAMETER",
    -215: "ERROR_SCI_CRC_FAILED", -216: "ERROR_SCI_BUFFER_FULL", -217: "ERROR_SCI_INVALID_START_BYTE",
    -218: "ERROR_SCI_INVALID_STOP_BYTE", -219: "ERROR_SCI_RX_BUFFER_FULL",
    -220: "ERROR_SCI_INVALID_ESCAPE_BYTE", -221: "ERROR_SCI_FRAME_TOO_SHORT",
    -222: "ERROR_SCI_FRAME_TOO_LONG", -223: "ERROR_SCI_TX_BUFFER_EXCEEDANCE",
}


def status_name(code: Optional[int]) -> str:
    if code is None:
        return "no reason given"
    return f"{STATUS_NAMES.get(code, 'UNKNOWN')} ({code})"


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


# --------------------------------------------------------------------------------------------
# Data link layer: CRC8, byte stuffing, framing
# --------------------------------------------------------------------------------------------

def _make_crc8_table(poly: int = 0x1D) -> List[int]:
    table = []
    for i in range(256):
        b = i
        for _ in range(8):
            b = ((b << 1) ^ poly) & 0xFF if b & 0x80 else (b << 1) & 0xFF
        table.append(b)
    return table


_CRC8_TABLE = _make_crc8_table()


def crc8(data: bytes, crc: int = 0x00) -> int:
    """CRC8_SAE_J1850_ZERO (poly 0x1D, init 0x00, no reflection, no final XOR)."""
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def stuff_bytes(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in CONTROL_BYTES:
            out.append(ESCAPE_BYTE)
            out.append(b ^ 0xFF)
        else:
            out.append(b)
    return bytes(out)


def encode_frame(cmd: int, payload: bytes = b"", address: Optional[int] = None) -> bytes:
    """Builds a complete wire frame. address=None -> basic frame, otherwise extended frame."""
    if not 0 < cmd < 0x80:
        raise ValueError(f"invalid 7-bit command code 0x{cmd:02X}")
    if address is None:
        body = bytes([cmd]) + payload
    else:
        body = bytes([cmd | EXTENDED_FLAG, address & 0xFF]) + payload
    return bytes([START_BYTE]) + stuff_bytes(body + bytes([crc8(body)])) + bytes([STOP_BYTE])


@dataclass
class Frame:
    cmd: int                      # 7-bit command code
    extended: bool
    address: Optional[int]
    payload: bytes
    raw: bytes = b""              # unstuffed CMD..CRC bytes as received

    def __str__(self) -> str:
        addr = f" addr={self.address}" if self.extended else ""
        return f"Frame(cmd=0x{self.cmd:02X}{addr} len={len(self.payload)})"


@dataclass
class FrameError:
    reason: str
    raw: bytes = b""


class FrameDecoder:
    """Incremental decoder (port of sci_datalink.c RX path): feed() raw bytes,
    get back decoded Frames and FrameErrors (CRC mismatch, bad escape, truncated frame)."""

    def __init__(self) -> None:
        self._buf: Optional[bytearray] = None
        self._escape = False

    def feed(self, data: bytes) -> List[Union[Frame, FrameError]]:
        out: List[Union[Frame, FrameError]] = []
        for b in data:
            if self._escape:
                self._escape = False
                if b == START_BYTE:
                    out.append(FrameError("escape byte followed by start byte", bytes(self._buf or b"")))
                    self._buf = bytearray()
                    continue
                b ^= 0xFF
                if self._buf is not None and b not in CONTROL_BYTES:
                    out.append(FrameError(f"invalid escaped byte 0x{b:02X}", bytes(self._buf)))
                    self._buf = None
                    continue
            elif b == ESCAPE_BYTE:
                self._escape = self._buf is not None
                continue
            elif b == START_BYTE:
                if self._buf:
                    out.append(FrameError("start byte inside frame (frame truncated)", bytes(self._buf)))
                self._buf = bytearray()
                continue
            elif b == STOP_BYTE:
                if self._buf is not None:
                    out.append(self._finish(bytes(self._buf)))
                    self._buf = None
                continue
            if self._buf is not None:
                self._buf.append(b)
        return out

    @staticmethod
    def _finish(raw: bytes) -> Union[Frame, FrameError]:
        if len(raw) < 2:
            return FrameError("frame too short", raw)
        body, rx_crc = raw[:-1], raw[-1]
        calc = crc8(body)
        if calc != rx_crc:
            return FrameError(f"CRC mismatch (received 0x{rx_crc:02X}, calculated 0x{calc:02X})", raw)
        cmd = body[0]
        if cmd & EXTENDED_FLAG:
            if len(body) < 2:
                return FrameError("extended frame without address byte", raw)
            return Frame(cmd & 0x7F, True, body[1], body[2:], raw)
        return Frame(cmd, False, None, body[1:], raw)


def decode_frame(wire: bytes) -> Frame:
    """Decodes exactly one complete wire frame (0x02 ... 0x03). Raises ValueError on errors."""
    results = FrameDecoder().feed(wire)
    if len(results) != 1:
        raise ValueError(f"expected exactly one frame, got {len(results)}")
    if isinstance(results[0], FrameError):
        raise ValueError(results[0].reason)
    return results[0]


# --------------------------------------------------------------------------------------------
# Message layer: payload parsers (explorer_api.c / explorer_api_data.c)
# --------------------------------------------------------------------------------------------

class _Reader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0

    def remaining(self) -> int:
        return len(self.data) - self.pos

    def take(self, n: int) -> bytes:
        if self.remaining() < n:
            raise ValueError(f"payload too short: need {n} bytes at offset {self.pos}, have {self.remaining()}")
        chunk = self.data[self.pos:self.pos + n]
        self.pos += n
        return chunk

    def u8(self) -> int:
        return self.take(1)[0]

    def u16(self) -> int:
        return struct.unpack(">H", self.take(2))[0]

    def s16(self) -> int:
        return struct.unpack(">h", self.take(2))[0]

    def u24(self) -> int:
        return int.from_bytes(self.take(3), "big")

    def s24(self) -> int:
        return int.from_bytes(self.take(3), "big", signed=True)

    def u32(self) -> int:
        return struct.unpack(">I", self.take(4))[0]

    def timestamp(self) -> float:
        sec = self.u32()
        usec16 = self.u16()          # microseconds / 16
        return sec + usec16 * 16 / 1e6


def version_str(v: int) -> str:
    return f"{v >> 24}.{(v >> 16) & 0xFF}.{v & 0xFFFF}"


def parse_ack(frame: Frame) -> int:
    """Returns the acknowledged 7-bit command code."""
    return _Reader(frame.payload).u8() & 0x7F


def parse_nak(frame: Frame) -> Tuple[int, Optional[int]]:
    """Returns (rejected 7-bit command code, signed 16-bit reason status or None)."""
    r = _Reader(frame.payload)
    cmd = r.u8() & 0x7F
    reason = r.s16() if r.remaining() >= 2 else None
    return cmd, reason


@dataclass
class DeviceEntry:
    address: int
    module: int
    chip: int
    laser: int
    uid: int


@dataclass
class SoftwareInfo:
    explorer_version: int
    api_version: int
    devices: List[DeviceEntry]
    id_string: str

    def describe(self) -> str:
        lines = [
            f"Explorer App version : {version_str(self.explorer_version)}",
            f"AFBR-S50 API version : {version_str(self.api_version)}",
            f"Software ID          : {self.id_string}",
            f"Devices              : {len(self.devices)}",
        ]
        for d in self.devices:
            lines.append(
                f"  [addr {d.address}] module={MODULE_NAMES.get(d.module, 'unknown')} (0x{d.module:02X}) "
                f"chip=0x{d.chip:02X} laser=0x{d.laser:02X} UID=0x{d.uid:06X}")
        return "\n".join(lines)


def parse_software_info(payload: bytes) -> SoftwareInfo:
    """Parses the 0x05 Software Info response.

    v1.5.6 (TxCmd_SoftwareInfo): u32 Explorer version, u32 API version, u8 device count N,
    N x (u8 address, u8 module, u8 chip, u8 laser, u24 UID), ASCII ID string.
    Older firmware sent one device without count/address; that layout is the fallback.
    """
    r = _Reader(payload)
    explorer, api = r.u32(), r.u32()
    start = r.pos
    count = r.u8()
    if count <= 16 and r.remaining() >= 7 * count:
        devices = [DeviceEntry(r.u8(), r.u8(), r.u8(), r.u8(), r.u24()) for _ in range(count)]
        text = r.take(r.remaining())
        if all(32 <= c < 127 for c in text.rstrip(b"\0")):
            return SoftwareInfo(explorer, api, devices, text.rstrip(b"\0").decode("ascii"))
    r.pos = start
    dev = DeviceEntry(0, r.u8(), r.u8(), r.u8(), r.u24())
    text = r.take(r.remaining()).rstrip(b"\0").decode("ascii", errors="replace")
    return SoftwareInfo(explorer, api, [dev], text)


def parse_log_message(payload: bytes) -> Tuple[float, str]:
    r = _Reader(payload)
    ts = r.timestamp()
    return ts, r.take(r.remaining()).rstrip(b"\0").decode("ascii", errors="replace")


def range_raw_to_mm(raw: int) -> Optional[float]:
    """Q9.14 meters (signed 24-bit) -> millimeters. None for ARGUS_RANGE_MAX ("no object")."""
    if raw == RANGE_MAX_RAW:
        return None
    return raw * 1000.0 / 16384.0


def amplitude_raw_to_lsb(raw: int) -> float:
    """UQ12.4 -> LSB."""
    return raw / 16.0


def pixel_xy_to_channel(x: int, y: int) -> int:
    """PIXEL_XY2CH from argus_map.h."""
    return ((y << 3) & 0x10) | ((x ^ 0x07) << 1) | (y & 0x01)


def pixel_status_str(status: int) -> str:
    if status == 0:
        return "OK"
    return "|".join(name for bit, name in PIXEL_STATUS_FLAGS if status & bit)


@dataclass
class PixelData:
    x: Optional[int]              # None for the reference pixel
    y: Optional[int]
    status: int
    range_mm: Optional[float]     # None = no object / out of range
    amplitude: float


@dataclass
class MeasurementData:
    cmd: int
    status: int
    timestamp: float
    frame_state: int
    pixels: List[PixelData] = field(default_factory=list)
    reference_pixel: Optional[PixelData] = None
    pixel_enable_mask: Optional[int] = None
    channel_enable_mask: Optional[int] = None
    digital_integration_depth: Optional[int] = None
    analog_integration_depth: Optional[float] = None
    optical_power_ma: Optional[float] = None
    pixel_gain: Optional[int] = None
    range_1d_mm: Optional[float] = None
    amplitude_1d: Optional[float] = None
    signal_quality: Optional[int] = None
    auxiliary: Optional[Dict[str, float]] = None


def parse_measurement_data(cmd: int, payload: bytes) -> MeasurementData:
    """Parses non-debug measurement data frames 0x32 (full), 0x34 (3D) and 0x36 (1D).

    Layout per Serialize_MeasurementData() in explorer_api_data.c (v1.5.6):
      generic : s16 status, u32 seconds, u16 microseconds/16
      frame   : u32 frame state
                [3D/full] u16 digital depth, UQ10.6 analog depth, UQ12.4 optical power [mA],
                          u8 pixel gain, u32 pixel-enable mask, u32 channel-enable mask
      3D      : u8 status[n], Q9.14 (24 bit) range[n] [m], UQ12.4 amplitude[n]
                (n = enabled pixels in x-major / y-minor order, + reference pixel if enabled)
      1D      : Q9.14 (24 bit) range [m], UQ12.4 amplitude, u8 signal quality [%]
      aux     : [full only] 7 x 16 bit UQ12.4 (VDD, VDDL, VSUB, IAPD, TEMP (signed), BGL, SNA)
    """
    if cmd not in MEASUREMENT_DATA_CMDS:
        raise ValueError(f"unsupported measurement data command 0x{cmd:02X}")
    r = _Reader(payload)
    status = r.s16()
    ts = r.timestamp()

    if cmd == CMD_DATA_1D:
        # v1.5.6 sends a 32-bit frame state (18 byte payload); older firmware sent 16 bits (16 bytes).
        state = r.u32() if len(payload) >= 18 else r.u16()
        m = MeasurementData(cmd, status, ts, state)
        _parse_1d(r, m)
        _expect_consumed(r)
        return m

    m = MeasurementData(cmd, status, ts, r.u32())
    m.digital_integration_depth = r.u16()
    m.analog_integration_depth = r.u16() / 64.0
    m.optical_power_ma = r.u16() / 16.0
    m.pixel_gain = r.u8()
    m.pixel_enable_mask = r.u32()
    m.channel_enable_mask = r.u32()

    coords = [(x, y) for x in range(PIXELS_X) for y in range(PIXELS_Y)
              if m.pixel_enable_mask & (1 << pixel_xy_to_channel(x, y))]
    tail = (3 + 2 + 1 + 14) if cmd == CMD_DATA_FULL else 0
    per_pixel_bytes = r.remaining() - tail
    if per_pixel_bytes < 0 or per_pixel_bytes % 6:
        raise ValueError(f"unexpected 3D data length ({r.remaining()} bytes after header)")
    n_values = per_pixel_bytes // 6
    has_ref = n_values == len(coords) + 1
    if not has_ref and n_values != len(coords):
        raise ValueError(f"pixel count mismatch: mask has {len(coords)} pixels, frame has {n_values} values")

    statuses = [r.u8() for _ in range(n_values)]
    ranges = [r.s24() for _ in range(n_values)]
    amps = [r.u16() for _ in range(n_values)]
    slots = coords + ([(None, None)] if has_ref else [])
    for (x, y), st, rg, am in zip(slots, statuses, ranges, amps):
        px = PixelData(x, y, st, range_raw_to_mm(rg), amplitude_raw_to_lsb(am))
        if x is None:
            m.reference_pixel = px
        else:
            m.pixels.append(px)

    if cmd == CMD_DATA_FULL:
        _parse_1d(r, m)
        m.auxiliary = {
            "VDD": r.u16() / 16.0, "VDDL": r.u16() / 16.0, "VSUB": r.u16() / 16.0,
            "IAPD": r.u16() / 16.0, "TEMP_C": r.s16() / 16.0, "BGL": r.u16() / 16.0,
            "SNA": r.u16() / 16.0,
        }
    _expect_consumed(r)
    return m


def _parse_1d(r: _Reader, m: MeasurementData) -> None:
    m.range_1d_mm = range_raw_to_mm(r.s24())
    m.amplitude_1d = amplitude_raw_to_lsb(r.u16())
    m.signal_quality = r.u8()


def _expect_consumed(r: _Reader) -> None:
    if r.remaining():
        raise ValueError(f"{r.remaining()} unexpected trailing bytes in measurement data")


def _fmt_mm(v: Optional[float]) -> str:
    return "    n/a" if v is None else f"{v:7.1f}"


def format_measurement(m: MeasurementData) -> str:
    lines = [f"t={m.timestamp:12.6f}s  status={status_name(m.status)}"]
    if m.amplitude_1d is not None:
        lines.append(f"  1D: distance={_fmt_mm(m.range_1d_mm)} mm  amplitude={m.amplitude_1d:8.2f}  "
                     f"signal quality={m.signal_quality}%")
    if m.pixels:
        grid = {(p.x, p.y): p for p in m.pixels}
        lines.append("  per-pixel distance [mm] / amplitude (rows y=0..3, columns x=0..7, '*' = status != OK):")
        for y in range(PIXELS_Y):
            cells = []
            for x in range(PIXELS_X):
                p = grid.get((x, y))
                if p is None:
                    cells.append("      -      ")
                else:
                    flag = "*" if p.status else " "
                    cells.append(f"{_fmt_mm(p.range_mm)}/{p.amplitude:5.0f}{flag}")
            lines.append("   " + " ".join(cells))
        bad = [f"({p.x},{p.y}):{pixel_status_str(p.status)}" for p in m.pixels if p.status]
        if bad:
            lines.append("  pixel status: " + ", ".join(bad))
    return "\n".join(lines)


# --------------------------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------------------------

class TransportError(Exception):
    pass


class SerialTransport:
    """Virtual COM port (pyserial). The baud rate is irrelevant for USB CDC but used for real UARTs."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise TransportError("pyserial is not installed: pip install pyserial") from e
        try:
            self._ser = serial.Serial(port, baudrate, timeout=0.05, write_timeout=1.0)
        except serial.SerialException as e:
            raise TransportError(f"cannot open {port}: {e}") from e
        self.name = port
        self._ser.reset_input_buffer()

    def write(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()

    def read(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n if n else 1)

    def close(self) -> None:
        self._ser.close()


class UsbTransport:
    """Raw USB bulk access (pyusb + libusb, device bound to WinUSB on Windows).
    The RA4M2 Explorer App exposes a vendor-class interface with bulk IN 0x81 / OUT 0x01."""

    def __init__(self, vid: int = USB_VID, pid: int = USB_PID) -> None:
        try:
            import usb.core  # type: ignore
            import usb.util  # type: ignore
        except ImportError as e:
            raise TransportError("pyusb is not installed: pip install pyusb libusb-package") from e
        self._usb = usb
        backend = None
        try:
            import libusb_package  # type: ignore  # ships libusb-1.0.dll for Windows
            backend = libusb_package.get_libusb1_backend()
        except ImportError:
            pass
        try:
            dev = usb.core.find(idVendor=vid, idProduct=pid, backend=backend)
        except usb.core.NoBackendError as e:
            raise TransportError("no libusb backend found: pip install libusb-package") from e
        if dev is None:
            raise TransportError(f"USB device {vid:04X}:{pid:04X} not found")
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass  # already configured (typical with WinUSB)
        try:
            intf = dev.get_active_configuration()[(0, 0)]
            usb.util.claim_interface(dev, intf)
        except usb.core.USBError as e:
            raise TransportError(f"cannot claim USB interface 0 (is the WinUSB driver installed?): {e}") from e
        out_dir, in_dir = usb.util.ENDPOINT_OUT, usb.util.ENDPOINT_IN
        self._ep_out = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == out_dir)
        self._ep_in = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == in_dir)
        if self._ep_in is None or self._ep_out is None:
            raise TransportError("bulk endpoints not found on USB interface 0")
        self._dev = dev
        self.name = f"USB {vid:04X}:{pid:04X} (bus {dev.bus}, address {dev.address})"

    def write(self, data: bytes) -> None:
        self._ep_out.write(data, timeout=1000)

    def read(self) -> bytes:
        try:
            return bytes(self._ep_in.read(self._ep_in.wMaxPacketSize, timeout=50))
        except self._usb.core.USBTimeoutError:
            return b""
        except self._usb.core.USBError as e:
            if getattr(e, "errno", None) in (110, 10060) or "timed out" in str(e).lower():
                return b""
            raise

    def close(self) -> None:
        self._usb.util.dispose_resources(self._dev)


def find_serial_port() -> Optional[str]:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    return None


def open_transport(port: Optional[str], force_usb: bool, baudrate: int, log: Callable[[str], None]):
    if port:
        return SerialTransport(port, baudrate)
    if not force_usb:
        found = find_serial_port()
        if found:
            log(f"Auto-detected serial port {found} (VID {USB_VID:04X} / PID {USB_PID:04X})")
            return SerialTransport(found, baudrate)
    try:
        return UsbTransport()
    except TransportError as e:
        if force_usb:
            raise
        raise TransportError(
            f"no COM port with VID {USB_VID:04X}/PID {USB_PID:04X} found and raw USB failed ({e}). "
            "Pass --port COMx explicitly or see README.md.") from e


# --------------------------------------------------------------------------------------------
# SCI client
# --------------------------------------------------------------------------------------------

class SciError(Exception):
    pass


class NakError(SciError):
    def __init__(self, cmd: int, reason: Optional[int]) -> None:
        self.cmd = cmd
        self.reason = reason
        super().__init__(f"device NAK for command 0x{cmd:02X}: {status_name(reason)}")


class SciTimeout(SciError):
    pass


_DEFAULT = object()


class SciClient:
    """Command/response handling on top of a transport. A background thread decodes RX frames:
    measurement data goes to a data queue, log messages are printed, the rest feeds transact()."""

    def __init__(self, transport, address: Optional[int] = None, debug: bool = False,
                 timeout: float = 1.0, retries: int = 2, log: Callable[[str], None] = print) -> None:
        self.t = transport
        self.address = address        # None -> basic frames, int -> extended frames
        self.debug = debug
        self.timeout = timeout
        self.retries = retries
        self.log = log
        self.rx_errors = 0
        self._decoder = FrameDecoder()
        self._frames: "queue.Queue[Frame]" = queue.Queue()
        self._data: "queue.Queue[Frame]" = queue.Queue()
        self._stop = threading.Event()
        self._rx_error: Optional[BaseException] = None
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.t.read()
            except Exception as e:  # noqa: BLE001 - reported to the main thread
                self._rx_error = e
                return
            if data:
                self.handle_rx(data)

    def handle_rx(self, data: bytes) -> None:
        for item in self._decoder.feed(data):
            if isinstance(item, FrameError):
                self.rx_errors += 1
                self.log(f"[RX ERROR] {item.reason}; frame dropped: {hexdump(item.raw)}")
                continue
            if self.debug:
                self.log(f"[RX] {hexdump(bytes([START_BYTE]) + stuff_bytes(item.raw) + bytes([STOP_BYTE]))}"
                         f"  -> {item}")
            if item.cmd == CMD_LOG_MESSAGE:
                try:
                    ts, msg = parse_log_message(item.payload)
                    self.log(f"[DEVICE LOG {ts:.3f}s] {msg.rstrip()}")
                except ValueError:
                    self.log(f"[DEVICE LOG] {hexdump(item.payload)}")
            elif item.cmd in MEASUREMENT_DATA_CMDS or item.cmd in DEBUG_DATA_CMDS:
                self._data.put(item)
            else:
                self._frames.put(item)

    def _check_rx(self) -> None:
        if self._rx_error is not None:
            raise SciError(f"transport read failed: {self._rx_error}")

    def transact(self, cmd: int, payload: bytes = b"", expect_response: bool = False,
                 address=_DEFAULT) -> Optional[Frame]:
        """Sends a command, waits for its ACK (and the response frame for getters).
        Retries on timeout, raises NakError on NAK and SciTimeout if all attempts time out."""
        addr = self.address if address is _DEFAULT else address
        wire = encode_frame(cmd, payload, addr)
        last_exc: Optional[SciError] = None
        for attempt in range(1, self.retries + 2):
            while not self._frames.empty():
                stale = self._frames.get_nowait()
                if self.debug:
                    self.log(f"[RX] discarding unexpected {stale}")
            if self.debug:
                self.log(f"[TX] {hexdump(wire)}")
            try:
                self.t.write(wire)
            except Exception as e:  # noqa: BLE001
                raise SciError(f"transport write failed: {e}") from e
            try:
                return self._wait_ack(cmd, expect_response)
            except SciTimeout as e:
                last_exc = e
                self.log(f"[WARN] {e} (attempt {attempt}/{self.retries + 1})")
        raise last_exc  # type: ignore[misc]

    def _wait_ack(self, cmd: int, expect_response: bool) -> Optional[Frame]:
        response: Optional[Frame] = None
        acked = False
        deadline = time.monotonic() + self.timeout
        while True:
            self._check_rx()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                what = "response/ACK" if expect_response else "ACK"
                raise SciTimeout(f"timeout waiting for {what} of command 0x{cmd:02X}")
            try:
                f = self._frames.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                continue
            if f.cmd == CMD_ACK:
                acked_cmd = parse_ack(f)
                if acked_cmd == cmd:
                    if not expect_response or response is not None:
                        return response
                    acked = True  # the firmware may send the ACK ahead of a queued response
                    continue
                self.log(f"[WARN] ACK for unexpected command 0x{acked_cmd:02X}")
            elif f.cmd == CMD_NAK:
                nak_cmd, reason = parse_nak(f)
                if nak_cmd in (cmd, 0):  # 0: device could not decode our frame at all
                    raise NakError(nak_cmd, reason)
                self.log(f"[WARN] NAK for unexpected command 0x{nak_cmd:02X}: {status_name(reason)}")
            elif f.cmd == cmd:
                response = f
                if acked:
                    return response
            elif self.debug:
                self.log(f"[RX] ignoring unexpected {f}")

    def software_info(self) -> SoftwareInfo:
        # Always a basic frame: the device then reports all connected sensor devices.
        f = self.transact(CMD_SOFTWARE_INFO, expect_response=True, address=None)
        return parse_software_info(f.payload)

    def set_data_output_mode(self, mode: int) -> None:
        self.transact(CMD_CFG_DATA_OUTPUT_MODE, bytes([mode]))

    def set_measurement_mode(self, mode: int) -> None:
        self.transact(CMD_CFG_MEASUREMENT_MODE, bytes([mode]))

    def get_measurement_mode(self) -> int:
        return _Reader(self.transact(CMD_CFG_MEASUREMENT_MODE, expect_response=True).payload).u8()

    def set_frame_time_us(self, frame_time_us: int) -> None:
        self.transact(CMD_CFG_FRAME_TIME, struct.pack(">I", frame_time_us))

    def get_frame_time_us(self) -> int:
        return _Reader(self.transact(CMD_CFG_FRAME_TIME, expect_response=True).payload).u32()

    def start_measurements(self) -> None:
        self.transact(CMD_MEASUREMENT_START)

    def stop_measurements(self) -> None:
        self.transact(CMD_MEASUREMENT_STOP)

    def next_data_frame(self, timeout: float) -> Optional[Frame]:
        self._check_rx()
        try:
            return self._data.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        self._rx_thread.join(timeout=1.0)
        self.t.close()


# --------------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    def log(msg: str) -> None:
        print(msg, flush=True)

    try:
        transport = open_transport(args.port, args.usb, args.baudrate, log)
    except TransportError as e:
        log(f"ERROR: {e}")
        return 2
    log(f"Connected via {transport.name}")
    client = SciClient(transport, address=args.address, debug=args.debug,
                       timeout=args.timeout, retries=args.retries, log=log)
    measuring = False
    try:
        try:
            client.stop_measurements()  # in case a previous session left measurements running
        except NakError as e:
            log(f"[WARN] initial stop: {e}")

        log(client.software_info().describe())

        if args.mode != "keep":
            try:
                client.set_measurement_mode(MEASUREMENT_MODES[args.mode])
                log(f"Measurement mode     : {args.mode} (0x{MEASUREMENT_MODES[args.mode]:02X})")
            except NakError as e:
                log(f"[WARN] could not set measurement mode '{args.mode}': {e}; keeping device setting")
        try:
            log(f"Active mode          : 0x{client.get_measurement_mode():02X}")
        except SciError as e:
            log(f"[WARN] could not read measurement mode: {e}")

        client.set_data_output_mode(OUTPUT_MODES[args.output])
        log(f"Data output mode     : {args.output} ({OUTPUT_MODES[args.output]})")
        try:
            client.set_frame_time_us(args.frame_time)
            log(f"Frame time           : {client.get_frame_time_us()} us")
        except NakError as e:
            log(f"[WARN] could not set frame time {args.frame_time} us: {e}")

        measuring = True  # also send stop on exit if the start ACK is lost
        client.start_measurements()
        log("Measurements started - press Ctrl+C to stop.\n")

        last_data = time.monotonic()
        count = 0
        while args.frames <= 0 or count < args.frames:
            f = client.next_data_frame(timeout=0.5)
            if f is None:
                if time.monotonic() - last_data > max(3.0, 5 * args.frame_time / 1e6):
                    log("[WARN] no measurement data received in the last few seconds")
                    last_data = time.monotonic()
                continue
            last_data = time.monotonic()
            if f.cmd in DEBUG_DATA_CMDS:
                log(f"[INFO] debug data frame 0x{f.cmd:02X} not decoded ({len(f.payload)} bytes)")
                continue
            try:
                m = parse_measurement_data(f.cmd, f.payload)
            except ValueError as e:
                log(f"[RX ERROR] cannot parse measurement frame 0x{f.cmd:02X}: {e}: {hexdump(f.raw)}")
                continue
            count += 1
            log(f"#{count} " + format_measurement(m))
    except KeyboardInterrupt:
        log("\nInterrupted.")
    except NakError as e:
        log(f"ERROR: {e}")
        return 3
    except SciError as e:
        log(f"ERROR: {e}")
        return 4
    finally:
        if measuring:
            try:
                client.stop_measurements()
                log("Measurements stopped.")
            except (SciError, KeyboardInterrupt) as e:
                log(f"[WARN] stop failed: {e}")
        client.close()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Read AFBR-S50 distance data via the Explorer App SCI protocol.")
    p.add_argument("--port", help="serial port (e.g. COM5 or /dev/ttyACM0); default: auto-detect")
    p.add_argument("--usb", action="store_true", help="use raw USB bulk transfers (pyusb) instead of a COM port")
    p.add_argument("--baudrate", type=int, default=115200, help="UART baud rate (ignored for USB)")
    p.add_argument("--address", type=int, default=None,
                   help="send extended frames to this SCI device address (default: basic frames)")
    p.add_argument("--output", choices=list(OUTPUT_MODES), default="3d",
                   help="data output mode: 3d = per-pixel (default), full = per-pixel + 1D + aux, 1d = binned only")
    p.add_argument("--mode", choices=list(MEASUREMENT_MODES) + ["keep"], default="long-range",
                   help="measurement mode to configure (default: long-range; 'keep' = don't change)")
    p.add_argument("--frame-time", type=int, default=100000, help="frame time in us (default 100000 = 10 Hz)")
    p.add_argument("--frames", type=int, default=0, help="stop after N frames (default 0 = until Ctrl+C)")
    p.add_argument("--timeout", type=float, default=1.0, help="ACK/response timeout in seconds (default 1.0)")
    p.add_argument("--retries", type=int, default=2, help="retries after a timeout (default 2)")
    p.add_argument("--debug", action="store_true", help="dump raw TX/RX frames as hex")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
