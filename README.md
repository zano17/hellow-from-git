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

### Tests

The framing, CRC, ACK/NAK and measurement-data parsers are tested against the example
frames from the SDK documentation and against frames built with a port of the firmware
serializer (no hardware needed):

```
python -m unittest -v test_sci_client
```
