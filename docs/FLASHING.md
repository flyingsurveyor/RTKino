# RTKino — Flashing Guide

## Hardware requirements

- **Board**: LOLIN S3 Pro (ESP32-S3, 16MB Flash, 8MB PSRAM)
- **Cable**: USB-C **data** cable (not charge-only!)
- **Drivers**: on Windows you may need the CH343/CH9102 driver — [download](https://www.wch-ic.com/downloads/CH343SER_EXE.html)

---

## Option 1 — Web Flasher (recommended, zero install)

The simplest method. Works directly from your browser.

1. Open **Google Chrome** (or Edge/Brave — any Chromium-based browser)
2. Go to the [Web Flasher](https://flyingsurveyor.github.io/RTKINO/)
3. Connect the ESP32-S3 via USB
4. Click **"Install RTKino"** and select the serial port
5. Wait for completion (~60 seconds)

> **Note**: if the port doesn't appear, hold the **BOOT** button on the board, press **RESET**, then release BOOT. Try again.

---

## Option 2 — Flash script (Windows / macOS / Linux)

### Setup (one-time)

Install `esptool`:
```
pip install esptool
```

### Flashing

1. Download the release ZIP from GitHub
2. Extract the folder
3. Connect the ESP32-S3 via USB
4. Run:
   - **Windows**: double-click `flash.bat`
   - **macOS/Linux**: open a terminal in the folder and run `./flash.sh`

The script automatically detects the port and flashes the merged firmware (`rtkino-merged.bin`).

---

## Option 3 — Manual flash with esptool

For full control:

```bash
esptool.py --chip esp32s3 --port <PORT> --baud 921600 \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x0000  bootloader.bin \
    0x8000  partitions.bin \
    0xe000  boot_app0.bin \
    0x10000 firmware.bin
```

Or with the merged firmware (single file):

```bash
esptool.py --chip esp32s3 --port <PORT> --baud 921600 \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x0 rtkino-merged.bin
```

Typical ports:
- **Windows**: `COM3`, `COM4`, ...
- **macOS**: `/dev/cu.usbmodem*`
- **Linux**: `/dev/ttyACM0`, `/dev/ttyUSB0`

---

## Option 4 — OTA update (firmware already installed)

If RTKino is already running on your board:

1. Download only `firmware.bin` from the release
2. Connect to RTKino's WiFi network
3. Open the web interface → Menu → **OTA Update**
4. Upload `firmware.bin`
5. Wait for the automatic reboot

> OTA requires **only** `firmware.bin`, not the individual binaries.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Port not detected | Try a different USB cable (must be **data**, not charge-only) |
| Permission denied (Linux) | `sudo usermod -a -G dialout $USER` then log out and back in |
| Flash failed | Hold **BOOT**, press **RESET**, release BOOT → try again |
| Firmware corrupted after flash | Erase first: `esptool.py --chip esp32s3 erase_flash` |

---

## Flash memory layout (16MB)

| Partition | Type | Offset | Size | Notes |
|-----------|------|--------|------|-------|
| bootloader | boot | 0x0000 | ~16 KB | ESP-IDF bootloader |
| partitions | data | 0x8000 | 3 KB | Custom partition table |
| boot_app0 | data | 0xE000 | 8 KB | OTA boot selector |
| app0 | app (OTA_0) | 0x10000 | 4 MB | Main firmware |
| app1 | app (OTA_1) | 0x410000 | 4 MB | OTA backup firmware |
| coredump | data | 0x810000 | 64 KB | Crash dump |
| spiffs | data | 0x820000 | ~7.9 MB | Data/config storage |
