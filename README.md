# hellow-from-git

## AFBR-S50 SCI client (`sci_client.py`)

`sci_client.py` reads distance measurements from the **Mikroe MIKROE-6239** board
(Broadcom AFBR-S50MX85I time-of-flight sensor + Renesas RA4M2 running the
**AFBR-S50 Explorer App firmware v1.5.6**) over USB, using Broadcom's SCI protocol
(the same protocol used by the AFBR-S50 Explorer PC application).

The protocol code is a minimal Python port of the host-side SCI implementation in
[Broadcom/AFBR-S50-API @ v1.5.6](https://github.com/Broadcom/AFBR-S50-API/tree/v1.5.6)
(`Sources/ExplorerApp/sci`, `Sources/ExplorerApp/api`, `Doxygen/05_2x_explorer_*.md`).
Only commands that exist in firmware v1.5.6 are used:

| Command | Code | Used for |
|---|---|---|
| Software Info | `0x05` | device / firmware info |
| Measurement Start / Stop | `0x11` / `0x12` | streaming control |
| Data Output Mode | `0x41` | select 3D / full / 1D streaming |
| Measurement Mode | `0x42` | short / long range (high speed) |
| Frame Time | `0x43` | measurement rate |
| Measurement Data | `0x34` (3D), `0x32` (full), `0x36` (1D) | parsed streaming data |

No crosstalk-monitor or DCA configuration commands are sent.

### Setup

Requires Python 3.8+.

```
pip install pyserial                   # COM-port access (required for --port / auto-detect)
pip install pyusb libusb-package       # optional: raw USB access (--usb)
```

The board enumerates with **VID `0x276E` / PID `0x0301`**. Depending on the driver bound
to it, it shows up either as a virtual COM port (use pyserial) or as a vendor-class
USB device with two bulk endpoints (use pyusb):

* **COM port**: if Windows lists the board under *Ports (COM & LPT)*, use `--port COMx`.
* **Raw USB / WinUSB**: if the board appears under *Universal Serial Bus devices* or as an
  unknown device (this is the normal case for the Explorer App firmware's vendor-class
  interface), bind it to **WinUSB** (e.g. with [Zadig](https://zadig.akeo.ie/): *Options ->
  List All Devices*, select the device with USB ID `276E 0301`, driver *WinUSB*, *Replace
  Driver*) and run with `--usb`. `libusb-package` ships the `libusb-1.0.dll` that pyusb needs
  on Windows. Note: replacing the driver may stop the Broadcom Explorer PC app from
  seeing the board until the original driver is restored.

### Finding the COM port on Windows

* **Device Manager** (`devmgmt.msc`) -> *Ports (COM & LPT)*: plug the board in and look for
  the new entry (e.g. `USB Serial Device (COM5)`).
* PowerShell:
  ```
  Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_276E&PID_0301' }
  ```
* pyserial:
  ```
  python -m serial.tools.list_ports -v
  ```
  look for `VID:PID=276E:0301`.

Without `--port`, the script auto-detects a COM port with VID/PID `276E:0301` and falls
back to raw USB (pyusb) if none is found.

### Usage

```
python sci_client.py                    # auto-detect, long-range mode, per-pixel (3D) data at 10 Hz
python sci_client.py --port COM5        # explicit COM port
python sci_client.py --usb              # raw USB bulk transfers via pyusb/WinUSB
python sci_client.py --output full      # per-pixel + binned 1D distance + auxiliary values
python sci_client.py --output 1d        # binned 1D distance/amplitude only
python sci_client.py --mode short-range --frame-time 50000
python sci_client.py --debug            # dump raw TX/RX frames as hex
```

The script prints the device info, configures the measurement mode (default `long-range`;
`--mode keep` leaves the device setting unchanged), the data output mode and the frame
time, starts measurements and prints every frame until **Ctrl+C**, then stops the
measurements. For 3D data it prints an 8x4 grid of `distance[mm]/amplitude` per pixel;
pixels with a non-OK status are marked with `*` and their status flags are listed
(e.g. `SAT`, `NO_SIGNAL`). `n/a` means no object / out of range.

Other options: `--address N` (send extended/addressed frames), `--frames N` (stop after
N frames), `--timeout`, `--retries`, `--baudrate` (only for a real UART). Run
`python sci_client.py --help` for details.

### Error handling

* **NAK** responses are reported with the rejected command and the firmware status
  code (e.g. `ERROR_SCI_UNKNOWN_COMMAND (-211)`). An unsupported measurement mode
  only produces a warning; the device setting is kept.
* **CRC / framing errors** in received frames are reported (`[RX ERROR] ...`) and the
  frame is dropped; the decoder resynchronizes on the next start byte.
* **Timeouts** waiting for an ACK/response are retried (`--retries`) and then
  reported as an error.
* `--debug` prints every TX/RX frame as hex.

### Desktop app (`sci_gui.py`)

A Tkinter window on top of `sci_client.py` (Tkinter ships with Python on Windows):

```
python sci_gui.py                          # start the viewer
python sci_gui.py --port COM5              # preselect a port
python sci_gui.py --simulate --autostart   # demo with the built-in simulator (no hardware)
```

Pick the port (*Auto-detect*, a COM port, *Raw USB (pyusb/WinUSB)* or *Simulator*), click
**Connect** (prints the device info), choose measurement mode / output (`3d`, `full`, `1d`) /
frame time and click **Start**. The 8x4 map shows each pixel's distance in mm (colour scale:
red = near, blue = far; red outline = pixel status not OK, grey = no signal), next to the
1D distance (or the mean of the valid pixels in `3d` mode), the frame rate and the log
(NAKs, CRC errors, timeouts; tick *Debug hex log* for raw TX/RX frames).

#### Standalone `.exe`

```
pip install pyinstaller pyserial pyusb libusb-package
pyinstaller --noconfirm --onefile --windowed --name AFBR-S50-Viewer --collect-all libusb_package ^
    --hidden-import serial.tools.list_ports --hidden-import usb.backend.libusb1 sci_gui.py
```

The result is `dist\AFBR-S50-Viewer.exe`, which runs without a Python installation.

### OpenMV (`afbr_openmv.py`, UART)

`afbr_openmv.py` is a MicroPython script for an OpenMV camera (e.g. OpenMV N6). It talks the
same SCI protocol over a UART instead of USB.

1. **Flash the UART firmware.** The Explorer App uses either USB or UART, chosen at compile
   time (`AFBR_SCI_USB` in `Sources/ExplorerApp/sci/sci_datalink.c`). Open
   `Projects/e2Studio/AFBR_S50_ExplorerApp_RA4M2` from AFBR-S50-API **v1.5.6** in Renesas
   e² studio, select the **ReleaseUART** build configuration (`AFBR_SCI_USB=0`) and build it
   (the v1.5.6 release has no prebuilt binaries). Flash it with a debugger on J5 (SWD), or
   without one via the RA4M2 USB bootloader: power off, put a jumper on J5 pins 7-9, plug in
   USB, flash the `.srec`/`.hex` with Renesas Flash Programmer (RA, COM port), then remove the
   jumper (see `Doxygen/06_reference_board.md` in the SDK).
   With this firmware the board no longer talks over USB, so `sci_client.py`, `sci_gui.py` and
   Broadcom's Explorer won't see it until you flash the USB build (**Release**) back.
2. **Wire it** (3.3 V logic) to the board's UART connector **J3**, a 6-pin JST GH
   (1.25 mm pitch) socket; you need a matching GH 6-pin plug/cable (e.g. housing GHR-06V-S):
   J3-3 TXD -> OpenMV UART RX, J3-2 RXD -> OpenMV UART TX, J3-6 GND -> GND (J3-1 is 5 V,
   J3-4/5 are not connected). The J3 pinout is from Broadcom's reference-board docs; the
   firmware's UART is SCI0 (P100/P101), and J3 being wired to it is not verified here against
   the MIKROE-6239 schematic. Check the OpenMV N6 pinout for the UART you use.
3. **Run it.** Set `UART_ID`, `OUTPUT`, `MODE` and `FRAME_TIME_US` at the top of the script
   (115200 baud is the firmware default), copy it to the OpenMV drive and run it from the
   OpenMV IDE. It prints the device info, then one line per frame, e.g.
   `distance 1532 mm (mean)  pixels sent 32 of 32, 30 valid`.

At 115200 baud a 3D frame (~220 bytes) fits about 50 times per second; use `1d` output for
higher frame rates.

### Tests

The framing, CRC, ACK/NAK and measurement-data parsers are tested against the example
frames from the SDK documentation and against frames built with a port of the firmware
serializer, plus the GUI's built-in simulator (no hardware needed):

```
python -m unittest -v test_sci_client
```
