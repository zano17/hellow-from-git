#!/usr/bin/env python3
"""
Desktop viewer for the MIKROE-6239 (AFBR-S50MX85I, Explorer App firmware v1.5.6).

Tkinter GUI on top of sci_client.py: pick a port (auto-detect / COM port / raw USB /
simulator), connect, choose measurement mode, output mode and frame time, then watch a
live 8x4 per-pixel distance map, the 1D distance and the protocol log.

  python sci_gui.py                    # start the GUI
  python sci_gui.py --port COM5        # preselect a port
  python sci_gui.py --simulate --autostart   # demo without hardware
"""

import argparse
import math
import queue
import random
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from typing import Optional

import sci_client as sc

AUTO = "Auto-detect"
RAW_USB = "Raw USB (pyusb/WinUSB)"
SIMULATOR = "Simulator (no hardware)"


# --------------------------------------------------------------------------------------------
# Simulated device (answers like the Explorer App firmware; for demos and tests)
# --------------------------------------------------------------------------------------------

def _s24(v: int) -> bytes:
    return (v & 0xFFFFFF).to_bytes(3, "big")


class SimTransport:
    """In-process fake of the Explorer App firmware: ACK/NAK, getters, software info and
    streaming of 3D / full / 1D measurement frames with a moving target."""

    name = "Simulator"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rx = bytearray()
        self._decoder = sc.FrameDecoder()
        self._mode = sc.MEASUREMENT_MODES["long-range"]
        self._output = sc.OUTPUT_MODES["3d"]
        self._frame_time_us = 100000
        self._streaming = False
        self._next_frame = 0.0
        self._t0 = time.monotonic()

    def _reply(self, req: sc.Frame, cmd: int, payload: bytes = b"") -> None:
        self._rx += sc.encode_frame(cmd, payload, req.address if req.extended else None)

    def write(self, data: bytes) -> None:
        with self._lock:
            for f in self._decoder.feed(data):
                if isinstance(f, sc.FrameError):
                    continue
                self._handle(f)

    def _handle(self, f: sc.Frame) -> None:
        cmd, p = f.cmd, f.payload
        if cmd == sc.CMD_SOFTWARE_INFO:
            ver = (1 << 24) | (5 << 16) | 6
            info = struct.pack(">II", ver, ver) + bytes([1, 1, 0x45, 0x05, 0x05]) + _s24(0x00C0DE)
            self._reply(f, cmd, info + b"AFBR-S50 Explorer App - Simulator")
        elif cmd == sc.CMD_CFG_MEASUREMENT_MODE:
            if p:
                self._mode = p[0]
            else:
                self._reply(f, cmd, bytes([self._mode]))
        elif cmd == sc.CMD_CFG_DATA_OUTPUT_MODE:
            if p:
                self._output = p[0]
            else:
                self._reply(f, cmd, bytes([self._output]))
        elif cmd == sc.CMD_CFG_FRAME_TIME:
            if p:
                self._frame_time_us = struct.unpack(">I", p[:4])[0]
            else:
                self._reply(f, cmd, struct.pack(">I", self._frame_time_us))
        elif cmd == sc.CMD_MEASUREMENT_START:
            self._streaming = True
            self._next_frame = time.monotonic()
        elif cmd == sc.CMD_MEASUREMENT_STOP:
            self._streaming = False
        else:
            self._reply(f, sc.CMD_NAK, bytes([cmd]) + struct.pack(">h", -211))
            return
        self._reply(f, sc.CMD_ACK, bytes([cmd]))

    def read(self) -> bytes:
        now = time.monotonic()
        with self._lock:
            if self._streaming and now >= self._next_frame:
                self._next_frame = now + self._frame_time_us / 1e6
                self._rx += self._measurement_frame(now - self._t0)
            data, self._rx = bytes(self._rx), bytearray()
        if not data:
            time.sleep(0.005)
        return data

    def _measurement_frame(self, t: float) -> bytes:
        base = 1400 + 1000 * math.sin(t * 0.8)              # target distance in mm
        tilt = 120 * math.sin(t * 0.3)
        pixels = []
        for x in range(sc.PIXELS_X):
            for y in range(sc.PIXELS_Y):
                d = base + tilt * (x - 3.5) + 40 * (y - 1.5) + random.gauss(0, 8)
                amp = max(5.0, 4000.0 * (500.0 / max(d, 100.0)) ** 2)
                status = 0
                if random.random() < 0.03:
                    status, d, amp = 0x20, None, 0.0          # NO_SIGNAL
                elif amp > 3500:
                    status = 0x02                             # SAT
                pixels.append((status, d, amp))
        sec = int(t)
        hdr = struct.pack(">hIH", 0, sec, int((t - sec) * 1e6) // 16)
        raw_range = [0x7FFFFF if d is None else int(d / 1000 * 16384) for _, d, _ in pixels]
        raw_amp = [min(0xFFFF, int(a * 16)) for _, _, a in pixels]
        ok = [i for i, (s, _, _) in enumerate(pixels) if s == 0]
        bin_mm = sum(pixels[i][1] for i in ok) / len(ok) if ok else None
        bin_amp = sum(pixels[i][2] for i in ok) / len(ok) if ok else 0.0
        one_d = _s24(0x7FFFFF if bin_mm is None else int(bin_mm / 1000 * 16384))
        one_d += struct.pack(">HB", min(0xFFFF, int(bin_amp * 16)), 90)

        if self._output == sc.OUTPUT_MODES["1d"]:
            return sc.encode_frame(sc.CMD_DATA_1D, hdr + struct.pack(">I", 0) + one_d, 1)
        frame_cfg = struct.pack(">IHHHBII", 0, 64, 4 * 64, 40 * 16, 0x55, 0xFFFFFFFF, 0)
        # firmware order: x-major / y-minor -> same order as `pixels`
        px = bytes(s for s, _, _ in pixels) + b"".join(_s24(r) for r in raw_range)
        px += b"".join(struct.pack(">H", a) for a in raw_amp)
        if self._output == sc.OUTPUT_MODES["full"]:
            aux = struct.pack(">HHHHhHH", 33 * 16, 18 * 16, 20 * 16, 100, 35 * 16, 50, 10)
            return sc.encode_frame(sc.CMD_DATA_FULL, hdr + frame_cfg + px + one_d + aux, 1)
        return sc.encode_frame(sc.CMD_DATA_3D, hdr + frame_cfg + px, 1)

    def close(self) -> None:
        self._streaming = False


# --------------------------------------------------------------------------------------------
# Background worker: owns transport + SciClient, never touches Tk widgets
# --------------------------------------------------------------------------------------------

class Worker(threading.Thread):
    """Executes connect/start/stop/disconnect requests and streams measurement data.
    Results are posted as (kind, value) tuples to `events` for the Tk thread."""

    def __init__(self, events: "queue.Queue") -> None:
        super().__init__(daemon=True)
        self.events = events
        self.requests: "queue.Queue" = queue.Queue()
        self.client: Optional[sc.SciClient] = None
        self.measuring = False

    def post(self, kind: str, value=None) -> None:
        self.events.put((kind, value))

    def log(self, msg: str) -> None:
        self.post("log", msg)

    def request(self, name: str, *args) -> None:
        self.requests.put((name, args))

    def set_debug(self, on: bool) -> None:
        if self.client:
            self.client.debug = on

    def run(self) -> None:
        while True:
            try:
                name, args = self.requests.get(timeout=0.02 if self.measuring else 0.2)
            except queue.Empty:
                name = None
            if name == "quit":
                self._disconnect()
                return
            if name:
                try:
                    getattr(self, "_" + name)(*args)
                except (sc.SciError, sc.TransportError) as e:
                    self.post("error", str(e))
                    if name == "connect":
                        self._disconnect()
                except Exception as e:  # noqa: BLE001 - keep the GUI alive on unexpected errors
                    self.post("error", f"{type(e).__name__}: {e}")
            if self.measuring:
                self._pump_data()

    def _pump_data(self) -> None:
        try:
            for _ in range(20):
                f = self.client.next_data_frame(timeout=0.01)
                if f is None:
                    return
                if f.cmd in sc.DEBUG_DATA_CMDS:
                    continue
                try:
                    self.post("data", sc.parse_measurement_data(f.cmd, f.payload))
                except ValueError as e:
                    self.log(f"[RX ERROR] cannot parse measurement frame 0x{f.cmd:02X}: {e}")
        except sc.SciError as e:
            self.post("error", str(e))
            self.measuring = False
            self.post("stopped")

    def _connect(self, port: str, address: Optional[int], debug: bool) -> None:
        self._disconnect()
        if port == SIMULATOR:
            transport = SimTransport()
        elif port == RAW_USB:
            transport = sc.UsbTransport()
        elif port == AUTO:
            transport = sc.open_transport(None, False, 115200, self.log)
        else:
            transport = sc.SerialTransport(port.split(" ")[0])
        self.client = sc.SciClient(transport, address=address, debug=debug, log=self.log)
        self.log(f"Connected via {transport.name}")
        try:
            self.client.stop_measurements()
        except sc.NakError as e:
            self.log(f"[WARN] initial stop: {e}")
        info = self.client.software_info()
        self.log(info.describe())
        self.post("connected", (transport.name, info.describe()))

    def _start(self, mode: str, output: str, frame_time_us: int) -> None:
        c = self.client
        if c is None:
            raise sc.SciError("not connected")
        if mode != "keep":
            try:
                c.set_measurement_mode(sc.MEASUREMENT_MODES[mode])
                self.log(f"Measurement mode set to {mode}")
            except sc.NakError as e:
                self.log(f"[WARN] could not set measurement mode '{mode}': {e}; keeping device setting")
        try:
            self.log(f"Active measurement mode: 0x{c.get_measurement_mode():02X}")
        except sc.SciError as e:
            self.log(f"[WARN] could not read measurement mode: {e}")
        c.set_data_output_mode(sc.OUTPUT_MODES[output])
        try:
            c.set_frame_time_us(frame_time_us)
            self.log(f"Frame time: {c.get_frame_time_us()} us")
        except sc.NakError as e:
            self.log(f"[WARN] could not set frame time: {e}")
        self.measuring = True  # also send stop on disconnect if the start ACK is lost
        c.start_measurements()
        self.log("Measurements started")
        self.post("started")

    def _stop(self) -> None:
        if self.client and self.measuring:
            self.measuring = False
            self.client.stop_measurements()
            self.log("Measurements stopped")
        self.post("stopped")

    def _disconnect(self) -> None:
        if self.client is None:
            return
        try:
            if self.measuring:
                self.measuring = False
                self.client.stop_measurements()
        except sc.SciError as e:
            self.log(f"[WARN] stop failed: {e}")
        finally:
            self.client.close()
            self.client = None
            self.post("disconnected")


# --------------------------------------------------------------------------------------------
# Tk GUI
# --------------------------------------------------------------------------------------------

CELL_W, CELL_H, PAD = 96, 74, 4
MONO = ("Consolas", 9)


def distance_colour(mm: float, max_mm: float) -> str:
    """Near = red, far = blue (HSV hue 0 -> 0.66)."""
    import colorsys
    t = min(max(mm / max_mm, 0.0), 1.0)
    r, g, b = colorsys.hsv_to_rgb(0.66 * t, 0.75, 0.95)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    out = []
    for p in lp.comports():
        tag = " [AFBR-S50]" if (p.vid, p.pid) == (sc.USB_VID, sc.USB_PID) else ""
        out.append(f"{p.device} - {p.description}{tag}")
    return out


class App:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.events: "queue.Queue" = queue.Queue()
        self.worker = Worker(self.events)
        self.worker.start()
        self.connected = self.measuring = False
        self.frames = 0
        self.fps_t0, self.fps_n, self.fps = time.monotonic(), 0, 0.0
        root.title("AFBR-S50 Viewer - MIKROE-6239")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._build(args)
        self.refresh_ports(args.port or (SIMULATOR if args.simulate else None))
        root.after(50, self.poll)
        if args.autostart:
            root.after(200, self.toggle_connect)
            self._autostart = True
        else:
            self._autostart = False

    # layout ------------------------------------------------------------------------------
    def _build(self, args) -> None:
        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Label(conn, text="Port:").grid(row=0, column=0, padx=4)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, width=46, state="readonly")
        self.port_box.grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.ext_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(conn, text="Addressed frames (addr 1)", variable=self.ext_var).grid(row=0, column=3, padx=4)
        self.debug_var = tk.BooleanVar(value=args.debug)
        ttk.Checkbutton(conn, text="Debug hex log", variable=self.debug_var,
                        command=lambda: self.worker.set_debug(self.debug_var.get())).grid(row=0, column=4, padx=4)
        self.conn_btn = ttk.Button(conn, text="Connect", command=self.toggle_connect)
        self.conn_btn.grid(row=0, column=5, padx=6)

        meas = ttk.LabelFrame(self.root, text="Measurement")
        meas.grid(row=1, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Label(meas, text="Mode:").grid(row=0, column=0, padx=4)
        self.mode_var = tk.StringVar(value=args.mode)
        ttk.Combobox(meas, textvariable=self.mode_var, state="readonly", width=22,
                     values=list(sc.MEASUREMENT_MODES) + ["keep"]).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(meas, text="Output:").grid(row=0, column=2, padx=4)
        self.output_var = tk.StringVar(value=args.output)
        ttk.Combobox(meas, textvariable=self.output_var, state="readonly", width=6,
                     values=list(sc.OUTPUT_MODES)).grid(row=0, column=3, padx=4)
        ttk.Label(meas, text="Frame time [ms]:").grid(row=0, column=4, padx=4)
        self.ft_var = tk.IntVar(value=args.frame_time // 1000)
        ttk.Spinbox(meas, from_=5, to=1000, increment=5, textvariable=self.ft_var, width=6).grid(row=0, column=5)
        ttk.Label(meas, text="Colour scale max [mm]:").grid(row=0, column=6, padx=4)
        self.scale_var = tk.IntVar(value=3000)
        ttk.Spinbox(meas, from_=100, to=20000, increment=100, textvariable=self.scale_var, width=7).grid(row=0, column=7)
        self.start_btn = ttk.Button(meas, text="Start", command=self.toggle_start, state="disabled")
        self.start_btn.grid(row=0, column=8, padx=8)

        w = sc.PIXELS_X * (CELL_W + PAD) + PAD
        h = sc.PIXELS_Y * (CELL_H + PAD) + PAD
        self.canvas = tk.Canvas(self.root, width=w, height=h, bg="#202020", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=6, pady=4)
        self.cells = {}
        for x in range(sc.PIXELS_X):
            for y in range(sc.PIXELS_Y):
                x0, y0 = PAD + x * (CELL_W + PAD), PAD + y * (CELL_H + PAD)
                rect = self.canvas.create_rectangle(x0, y0, x0 + CELL_W, y0 + CELL_H, fill="#3a3a3a", width=2,
                                                    outline="#3a3a3a")
                text = self.canvas.create_text(x0 + CELL_W / 2, y0 + CELL_H / 2, text=f"({x},{y})",
                                               fill="white", font=("Segoe UI", 10, "bold"), justify="center")
                self.cells[(x, y)] = (rect, text)

        side = ttk.Frame(self.root)
        side.grid(row=2, column=1, sticky="nsew", padx=6)
        ttk.Label(side, text="Distance").pack(anchor="w")
        self.big_var = tk.StringVar(value="--- mm")
        ttk.Label(side, textvariable=self.big_var, font=("Segoe UI", 30, "bold")).pack(anchor="w")
        self.big_src_var = tk.StringVar(value="")
        ttk.Label(side, textvariable=self.big_src_var).pack(anchor="w")
        self.stats_var = tk.StringVar(value="")
        ttk.Label(side, textvariable=self.stats_var, font=MONO, justify="left").pack(anchor="w", pady=6)
        self.info_var = tk.StringVar(value="Not connected")
        ttk.Label(side, textvariable=self.info_var, font=MONO, justify="left", wraplength=420).pack(anchor="w")

        logf = ttk.LabelFrame(self.root, text="Log")
        logf.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=6, pady=4)
        self.log_box = ScrolledText(logf, height=10, font=MONO, state="disabled")
        self.log_box.pack(fill="both", expand=True)
        ttk.Button(logf, text="Clear", command=self.clear_log).pack(anchor="e")
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=4, column=0, columnspan=2, sticky="ew")
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(3, weight=1)

    # actions -----------------------------------------------------------------------------
    def refresh_ports(self, preselect: Optional[str] = None) -> None:
        ports = list_ports()
        values = [AUTO] + ports + [RAW_USB, SIMULATOR]
        self.port_box["values"] = values
        if preselect:
            match = [v for v in values if v == preselect or v.split(" ")[0].upper() == preselect.upper()]
            self.port_var.set(match[0] if match else preselect)
        elif not self.port_var.get():
            tagged = [p for p in ports if p.endswith("[AFBR-S50]")]
            self.port_var.set(tagged[0] if tagged else AUTO)

    def toggle_connect(self) -> None:
        self.conn_btn.configure(state="disabled")
        if self.connected:
            self.worker.request("disconnect")
        else:
            self.status_var.set(f"Connecting to {self.port_var.get()} ...")
            address = 1 if self.ext_var.get() else None
            self.worker.request("connect", self.port_var.get(), address, self.debug_var.get())

    def toggle_start(self) -> None:
        self.start_btn.configure(state="disabled")
        if self.measuring:
            self.worker.request("stop")
        else:
            self.frames = 0
            self.worker.request("start", self.mode_var.get(), self.output_var.get(),
                                max(1, int(self.ft_var.get())) * 1000)

    def clear_log(self) -> None:
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")

    def append_log(self, msg: str) -> None:
        self.log_box.configure(state="normal")
        self.log_box.insert("end", msg + "\n")
        if int(self.log_box.index("end-1c").split(".")[0]) > 2000:
            self.log_box.delete("1.0", "500.0")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def on_close(self) -> None:
        self.worker.request("quit")
        self.worker.join(timeout=2.0)
        self.root.destroy()

    # event loop --------------------------------------------------------------------------
    def poll(self) -> None:
        latest = None
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "data":
                    latest = value
                    self.frames += 1
                    self.fps_n += 1
                elif kind == "log":
                    self.append_log(value)
                elif kind == "error":
                    self.append_log(f"ERROR: {value}")
                    self.status_var.set(f"Error: {value}")
                    self.conn_btn.configure(state="normal")
                    self.start_btn.configure(state="normal" if self.connected else "disabled")
                elif kind == "connected":
                    self.connected = True
                    self.info_var.set(value[1])
                    self.status_var.set(f"Connected via {value[0]}")
                    self.conn_btn.configure(text="Disconnect", state="normal")
                    self.start_btn.configure(state="normal")
                    if self._autostart:
                        self._autostart = False
                        self.toggle_start()
                elif kind == "disconnected":
                    self.connected = self.measuring = False
                    self.info_var.set("Not connected")
                    self.status_var.set("Disconnected")
                    self.conn_btn.configure(text="Connect", state="normal")
                    self.start_btn.configure(text="Start", state="disabled")
                elif kind == "started":
                    self.measuring = True
                    self.start_btn.configure(text="Stop", state="normal")
                    self.status_var.set("Measuring")
                elif kind == "stopped":
                    self.measuring = False
                    self.start_btn.configure(text="Start", state="normal" if self.connected else "disabled")
                    if self.connected:
                        self.status_var.set("Connected - measurements stopped")
        except queue.Empty:
            pass
        now = time.monotonic()
        if now - self.fps_t0 >= 1.0:
            self.fps = self.fps_n / (now - self.fps_t0)
            self.fps_t0, self.fps_n = now, 0
        if latest is not None:
            self.render(latest)
        self.root.after(50, self.poll)

    def render(self, m: sc.MeasurementData) -> None:
        max_mm = max(100, int(self.scale_var.get() or 3000))
        grid = {(p.x, p.y): p for p in m.pixels}
        for (x, y), (rect, text) in self.cells.items():
            p = grid.get((x, y))
            if p is None:
                fill, outline, label = "#3a3a3a", "#3a3a3a", "-" if m.pixels else f"({x},{y})"
            elif p.range_mm is None or p.status & (0x08 | 0x20 | 0x40 | 0x80):
                fill, outline = "#555555", "#ff4040"
                label = f"n/a\n{sc.pixel_status_str(p.status)}"
            else:
                fill = distance_colour(p.range_mm, max_mm)
                outline = "#ff4040" if p.status else fill
                flag = f"\n{sc.pixel_status_str(p.status)}" if p.status else f"\namp {p.amplitude:.0f}"
                label = f"{p.range_mm:.0f} mm{flag}"
            self.canvas.itemconfigure(rect, fill=fill, outline=outline)
            self.canvas.itemconfigure(text, text=label)

        valid = [p.range_mm for p in m.pixels if p.range_mm is not None and not (p.status & 0xE8)]
        if m.amplitude_1d is not None:
            big = m.range_1d_mm
            src = f"1D (binned)  amplitude {m.amplitude_1d:.1f}  quality {m.signal_quality}%"
        else:
            big = sum(valid) / len(valid) if valid else None
            src = f"mean of {len(valid)} valid pixels"
        self.big_var.set("n/a" if big is None else f"{big:.0f} mm")
        self.big_src_var.set(src)
        lines = [f"frames   : {self.frames}  ({self.fps:.1f} fps)",
                 f"time     : {m.timestamp:.3f} s",
                 f"status   : {sc.status_name(m.status)}"]
        if valid:
            lines.append(f"pixels   : min {min(valid):.0f} / max {max(valid):.0f} mm, {len(valid)} valid")
        if m.optical_power_ma is not None:
            lines.append(f"laser    : {m.optical_power_ma:.1f} mA, gain {m.pixel_gain}")
        if m.auxiliary:
            lines.append(f"aux      : VDD {m.auxiliary['VDD']:.2f}  TEMP {m.auxiliary['TEMP_C']:.1f} C")
        self.stats_var.set("\n".join(lines))


def main() -> None:
    p = argparse.ArgumentParser(description="AFBR-S50 desktop viewer (Explorer App SCI protocol)")
    p.add_argument("--port", help="preselect a port (e.g. COM5)")
    p.add_argument("--simulate", action="store_true", help="preselect the built-in simulator")
    p.add_argument("--autostart", action="store_true", help="connect and start measuring immediately")
    p.add_argument("--mode", choices=list(sc.MEASUREMENT_MODES) + ["keep"], default="long-range")
    p.add_argument("--output", choices=list(sc.OUTPUT_MODES), default="3d")
    p.add_argument("--frame-time", type=int, default=100000, help="frame time in us")
    p.add_argument("--debug", action="store_true", help="start with the hex debug log enabled")
    args = p.parse_args()
    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
