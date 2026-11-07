"""Unit tests for sci_client.py framing/parsing (no hardware required).

Reference frames are taken from the AFBR-S50-API v1.5.6 documentation
(Doxygen/05_20_explorer_app.md, Doxygen/Examples/sci_python_example.py).
Synthetic measurement frames are built with a Python port of the firmware serializer
(Sources/ExplorerApp/api/explorer_api_data.c) to check the decoder end to end.
"""

import struct
import threading
import time
import unittest

import sci_client as sc


def s24(v: int) -> bytes:
    return (v & 0xFFFFFF).to_bytes(3, "big")


def device_frame(cmd: int, payload: bytes, address=None) -> bytes:
    return sc.encode_frame(cmd, payload, address)


def generic_header(status: int = 0, sec: int = 12, usec: int = 345600) -> bytes:
    return struct.pack(">hIH", status, sec, usec // 16)


def serialize_3d_part(pixels, ref=None) -> bytes:
    """pixels: dict {(x, y): (status, range_q9_14, amp_uq12_4)} in firmware x-major order."""
    order = [(x, y) for x in range(8) for y in range(4) if (x, y) in pixels]
    vals = [pixels[k] for k in order] + ([ref] if ref else [])
    out = bytes(v[0] for v in vals)
    out += b"".join(s24(v[1]) for v in vals)
    out += b"".join(struct.pack(">H", v[2]) for v in vals)
    return out


def pixel_mask(coords) -> int:
    m = 0
    for x, y in coords:
        m |= 1 << sc.pixel_xy_to_channel(x, y)
    return m


def frame_header_3d(mask: int, ch_mask: int = 0) -> bytes:
    return struct.pack(">IHHHBII", 0x00000001, 64, 4 * 64, 40 * 16, 0x55, mask, ch_mask)


class CrcTests(unittest.TestCase):
    def test_crc8_sae_j1850_zero_check_value(self):
        # Catalogue check value for CRC-8/SAE-J1850-ZERO ("123456789").
        self.assertEqual(sc.crc8(b"123456789"), 0x37)

    def test_doc_example_crc(self):
        # 05_20_explorer_app.md: CRC of 0x41 0x07 is 0xF5.
        self.assertEqual(sc.crc8(bytes([0x41, 0x07])), 0xF5)


class EncodeTests(unittest.TestCase):
    def test_sdk_example_frames(self):
        # Frames from Doxygen/Examples/sci_python_example.py
        self.assertEqual(sc.encode_frame(0x41, bytes([0x07])), bytes.fromhex("02 41 07 F5 03"))
        self.assertEqual(sc.encode_frame(0x43, struct.pack(">I", 200000)),
                         bytes.fromhex("02 43 00 1B FC 0D 40 85 03"))  # 0x03 escaped as 1B FC
        self.assertEqual(sc.encode_frame(0x11), bytes.fromhex("02 11 D0 03"))
        self.assertEqual(sc.encode_frame(0x12), bytes.fromhex("02 12 F7 03"))

    def test_client_command_encoders(self):
        self.assertEqual(sc.encode_frame(sc.CMD_MEASUREMENT_START), bytes.fromhex("02 11 D0 03"))
        self.assertEqual(sc.encode_frame(sc.CMD_CFG_DATA_OUTPUT_MODE, bytes([sc.OUTPUT_MODES["1d"]])),
                         bytes.fromhex("02 41 07 F5 03"))

    def test_extended_frame(self):
        wire = sc.encode_frame(0x05, b"", address=1)
        self.assertEqual(wire[1], 0x85)
        self.assertEqual(wire[2], 0x01)
        f = sc.decode_frame(wire)
        self.assertTrue(f.extended)
        self.assertEqual((f.cmd, f.address, f.payload), (0x05, 1, b""))

    def test_escape_all_control_bytes_including_crc(self):
        for payload in (b"\x02\x03\x1b", bytes(range(256))):
            wire = sc.encode_frame(0x20, payload)
            self.assertNotIn(0x02, wire[1:-1])
            self.assertNotIn(0x03, wire[1:-1])
            self.assertEqual(sc.decode_frame(wire).payload, payload)
        # find a payload whose CRC is a control byte to make sure the CRC gets escaped too
        for i in range(256):
            body = bytes([0x20, i])
            if sc.crc8(body) in sc.CONTROL_BYTES:
                wire = sc.encode_frame(0x20, bytes([i]))
                self.assertEqual(wire[-3], sc.ESCAPE_BYTE)
                self.assertEqual(sc.decode_frame(wire).payload, bytes([i]))
                break
        else:
            self.fail("no CRC control-byte case found")


class DecodeTests(unittest.TestCase):
    def test_decode_sdk_frames(self):
        f = sc.decode_frame(bytes.fromhex("02 43 00 1B FC 0D 40 85 03"))
        self.assertEqual(f.cmd, 0x43)
        self.assertEqual(struct.unpack(">I", f.payload)[0], 200000)

    def test_crc_error_reported_and_stream_resyncs(self):
        good = sc.encode_frame(0x0A, bytes([0x11]))
        bad = bytearray(sc.encode_frame(0x0A, bytes([0x12])))
        bad[-2] ^= 0x01
        out = sc.FrameDecoder().feed(b"\x55\xAA" + bytes(bad) + good)
        self.assertEqual(len(out), 2)
        self.assertIsInstance(out[0], sc.FrameError)
        self.assertIn("CRC", out[0].reason)
        self.assertIsInstance(out[1], sc.Frame)
        self.assertEqual(sc.parse_ack(out[1]), 0x11)

    def test_truncated_frame_and_invalid_escape(self):
        good = sc.encode_frame(0x0A, bytes([0x11]))
        out = sc.FrameDecoder().feed(b"\x02\x0A\x11" + good + b"\x02\x0A\x1B\x00\x03")
        kinds = [type(o).__name__ for o in out]
        self.assertEqual(kinds, ["FrameError", "Frame", "FrameError"])
        self.assertIn("escaped", out[2].reason)

    def test_byte_by_byte_feed(self):
        wire = sc.encode_frame(0x43, struct.pack(">I", 200000))
        dec = sc.FrameDecoder()
        results = []
        for b in wire:
            results += dec.feed(bytes([b]))
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].payload, struct.pack(">I", 200000))

    def test_ack_nak(self):
        ack = sc.decode_frame(device_frame(sc.CMD_ACK, bytes([0x41])))
        self.assertEqual(sc.parse_ack(ack), 0x41)
        nak = sc.decode_frame(device_frame(sc.CMD_NAK, bytes([0x57]) + struct.pack(">h", -211)))
        self.assertEqual(sc.parse_nak(nak), (0x57, -211))
        self.assertIn("ERROR_SCI_UNKNOWN_COMMAND", str(sc.NakError(*sc.parse_nak(nak))))
        ext = sc.decode_frame(device_frame(sc.CMD_NAK, bytes([0xC2]) + struct.pack(">h", -105), address=1))
        self.assertEqual(sc.parse_nak(ext), (0x42, -105))


class PayloadTests(unittest.TestCase):
    def test_software_info_v156(self):
        payload = struct.pack(">II", (1 << 24) | (5 << 16) | 6, (1 << 24) | (5 << 16) | 6)
        payload += bytes([1, 1, 0x45, 0x05, 0x05]) + (0x123456).to_bytes(3, "big")
        payload += b"AFBR-S50 Explorer App - RA4M2"
        info = sc.parse_software_info(sc.decode_frame(device_frame(0x05, payload)).payload)
        self.assertEqual(sc.version_str(info.explorer_version), "1.5.6")
        self.assertEqual(len(info.devices), 1)
        d = info.devices[0]
        self.assertEqual((d.address, d.module, d.uid), (1, 0x45, 0x123456))
        self.assertIn("AFBR-S50MX85I", info.describe())
        self.assertEqual(info.id_string, "AFBR-S50 Explorer App - RA4M2")

    def test_1d_data_v156(self):
        range_q = int(1.2345 * 16384)
        payload = generic_header(status=0) + struct.pack(">I", 0) + s24(range_q) + struct.pack(">HB", 1234 * 16, 87)
        m = sc.parse_measurement_data(0x36, sc.decode_frame(device_frame(0x36, payload, address=1)).payload)
        self.assertAlmostEqual(m.range_1d_mm, range_q * 1000 / 16384)
        self.assertAlmostEqual(m.amplitude_1d, 1234.0)
        self.assertEqual(m.signal_quality, 87)
        self.assertAlmostEqual(m.timestamp, 12.3456)

    def test_1d_data_legacy_16bit_state(self):
        # Layout used by the SDK python example: status, time, 16-bit state, range, amp, quality.
        payload = generic_header(status=108) + struct.pack(">H", 0) + s24(-16384) + struct.pack(">HB", 16, 0)
        m = sc.parse_measurement_data(0x36, payload)
        self.assertEqual(m.status, 108)
        self.assertAlmostEqual(m.range_1d_mm, -1000.0)

    def test_3d_data_all_pixels_with_ref(self):
        pixels = {(x, y): (0, (x * 100 + y) * 16384 // 1000, (x + 1) * 16 * 10)
                  for x in range(8) for y in range(4)}
        pixels[(7, 3)] = (0x20, 0x7FFFFF, 0)  # no signal / max range
        ref = (0, 1000, 5000)
        payload = generic_header() + frame_header_3d(0xFFFFFFFF, 0x1) + serialize_3d_part(pixels, ref)
        wire = device_frame(0x34, payload, address=1)
        m = sc.parse_measurement_data(0x34, sc.decode_frame(wire).payload)
        self.assertEqual(len(m.pixels), 32)
        self.assertIsNotNone(m.reference_pixel)
        p = {(q.x, q.y): q for q in m.pixels}
        self.assertAlmostEqual(p[(2, 1)].range_mm, (201 * 16384 // 1000) * 1000 / 16384)
        self.assertAlmostEqual(p[(2, 1)].amplitude, 30.0)
        self.assertIsNone(p[(7, 3)].range_mm)
        self.assertEqual(sc.pixel_status_str(p[(7, 3)].status), "NO_SIGNAL")
        self.assertAlmostEqual(m.optical_power_ma, 40.0)
        self.assertEqual(m.pixel_gain, 0x55)
        self.assertIn("NO_SIGNAL", sc.format_measurement(m))

    def test_3d_partial_mask_without_ref(self):
        coords = [(0, 0), (3, 2), (7, 1)]
        pixels = {c: (0, 16384 * (i + 1), 16 * (i + 1)) for i, c in enumerate(coords)}
        payload = generic_header() + frame_header_3d(pixel_mask(coords)) + serialize_3d_part(pixels)
        m = sc.parse_measurement_data(0x34, payload)
        self.assertEqual([(q.x, q.y) for q in m.pixels], coords)
        self.assertEqual([q.range_mm for q in m.pixels], [1000.0, 2000.0, 3000.0])
        self.assertIsNone(m.reference_pixel)

    def test_full_data(self):
        coords = [(x, y) for x in range(8) for y in range(4)]
        pixels = {c: (0, 8192, 160) for c in coords}
        tail = s24(3 * 16384) + struct.pack(">HB", 800, 99)
        aux = struct.pack(">HHHHhHH", 33 * 16, 18 * 16, 1, 2, 25 * 16, 3, 4)
        payload = generic_header() + frame_header_3d(0xFFFFFFFF) + serialize_3d_part(pixels) + tail + aux
        m = sc.parse_measurement_data(0x32, sc.decode_frame(device_frame(0x32, payload)).payload)
        self.assertEqual(len(m.pixels), 32)
        self.assertTrue(all(q.range_mm == 500.0 and q.amplitude == 10.0 for q in m.pixels))
        self.assertEqual(m.range_1d_mm, 3000.0)
        self.assertEqual(m.amplitude_1d, 50.0)
        self.assertEqual(m.auxiliary["TEMP_C"], 25.0)

    def test_bad_length_rejected(self):
        payload = generic_header() + frame_header_3d(0xFFFFFFFF) + b"\x00" * 10
        with self.assertRaises(ValueError):
            sc.parse_measurement_data(0x34, payload)

    def test_pixel_channel_map(self):
        # Every (x, y) maps to a unique channel 0..31 (argus_map.h PIXEL_XY2CH).
        chans = {sc.pixel_xy_to_channel(x, y) for x in range(8) for y in range(4)}
        self.assertEqual(chans, set(range(32)))


class FakeTransport:
    """Loopback transport that answers like the firmware (ACK/NAK + getter responses)."""

    name = "fake"

    def __init__(self, nak_cmds=()):
        self.nak_cmds = nak_cmds
        self.rx = bytearray()
        self.lock = threading.Lock()
        self.decoder = sc.FrameDecoder()

    def write(self, data):
        with self.lock:
            self._respond(data)

    def _respond(self, data):
        for f in self.decoder.feed(data):
            if f.cmd in self.nak_cmds:
                self.rx += sc.encode_frame(sc.CMD_NAK, bytes([f.cmd]) + struct.pack(">h", -211))
                continue
            if f.cmd == sc.CMD_CFG_FRAME_TIME and not f.payload:
                self.rx += sc.encode_frame(f.cmd, struct.pack(">I", 100000))
            self.rx += sc.encode_frame(sc.CMD_ACK, bytes([f.cmd]))

    def read(self):
        with self.lock:
            data, self.rx = bytes(self.rx), bytearray()
        if not data:
            time.sleep(0.005)
        return data

    def close(self):
        pass


class ClientTests(unittest.TestCase):
    def test_transact_ack_response_and_nak(self):
        c = sc.SciClient(FakeTransport(nak_cmds=(sc.CMD_CFG_MEASUREMENT_MODE,)), timeout=0.5,
                         retries=0, log=lambda m: None)
        try:
            c.start_measurements()
            self.assertEqual(c.get_frame_time_us(), 100000)
            with self.assertRaises(sc.NakError) as cm:
                c.set_measurement_mode(0x02)
            self.assertEqual(cm.exception.reason, -211)
        finally:
            c.close()

    def test_timeout(self):
        class Silent(FakeTransport):
            def write(self, data):
                pass

        c = sc.SciClient(Silent(), timeout=0.2, retries=1, log=lambda m: None)
        try:
            with self.assertRaises(sc.SciTimeout):
                c.start_measurements()
        finally:
            c.close()


if __name__ == "__main__":
    unittest.main()
