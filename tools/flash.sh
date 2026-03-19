#!/usr/bin/env bash
# ============================================================================
# RTKino Flash Tool — macOS / Linux
# Flashes firmware to ESP32-S3 (LOLIN S3 Pro)
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "  ============================================"
echo "    RTKino Flash Tool"
echo "  ============================================"
echo ""

# --- Find esptool ------------------------------------------------------------
ESPTOOL=""

if [ "$(uname -s)" = "Darwin" ] && [ -x "./esptool-macos" ]; then
    ESPTOOL="./esptool-macos"
    echo "[OK] Found bundled esptool (macOS)"
elif [ -x "./esptool-linux" ]; then
    ESPTOOL="./esptool-linux"
    echo "[OK] Found bundled esptool (Linux)"
elif command -v esptool.py >/dev/null 2>&1; then
    ESPTOOL="esptool.py"
    echo "[OK] Found system esptool.py"
elif command -v esptool >/dev/null 2>&1; then
    ESPTOOL="esptool"
    echo "[OK] Found system esptool"
else
    echo "[ERROR] esptool not found in this folder!"
    echo "        Re-download the release ZIP from GitHub."
    echo ""
    read -rp "Press Enter to close..."
    exit 1
fi

# --- Show available ports ----------------------------------------------------
echo ""
echo "Available serial ports:"
echo ""
ls /dev/ttyACM* /dev/ttyUSB* /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* 2>/dev/null || echo "  (none detected — connect the ESP32-S3 via USB)"
echo ""
echo "  Tip — on Linux: sudo usermod -a -G dialout \$USER"
echo ""

if [ -n "${1:-}" ]; then
    PORT="$1"
else
    read -rp "Enter port (e.g. /dev/ttyACM0): " PORT
fi

if [ -z "$PORT" ]; then
    echo "[ERROR] No port entered."
    echo ""
    read -rp "Press Enter to close..."
    exit 1
fi

echo ""
echo "[OK] Using port: $PORT"
echo ""

# --- Flash -------------------------------------------------------------------
if [ -f "rtkino-merged.bin" ]; then
    echo "Flashing rtkino-merged.bin ... this takes about 60 seconds."
    echo ""
    $ESPTOOL --chip esp32s3 --port "$PORT" --baud 921600 \
        write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
        0x0 rtkino-merged.bin
else
    echo "Flashing individual binaries..."
    echo ""
    $ESPTOOL --chip esp32s3 --port "$PORT" --baud 921600 \
        write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
        0x0000 bootloader.bin \
        0x8000 partitions.bin \
        0xe000 boot_app0.bin \
        0x10000 firmware.bin
fi

echo ""
if [ $? -eq 0 ]; then
    echo "  ============================================"
    echo "    Flash completed!"
    echo "  ============================================"
    echo ""
    echo "  Next steps:"
    echo "    1. Connect to WiFi: rtkino_AP"
    echo "       Password: rtkino_zedf9p"
    echo "    2. Open http://192.168.4.1"
    echo "    3. Configure WiFi and NTRIP"
else
    echo "  [ERROR] Flash failed."
    echo ""
    echo "    - Use a DATA cable, not charge-only"
    echo "    - Hold BOOT, press RESET, release BOOT, re-run"
    echo "    - Try a different port"
    echo "    - On Linux: sudo usermod -a -G dialout \$USER"
fi

echo ""
read -rp "Press Enter to close..."
