#!/usr/bin/env bash
#
# Upload LittleFS image to ESP8266 sender node.
#
# Usage:
#   ./tools/upload-littlefs.sh [/dev/ttyUSBx]
#
# Reads data/injection-schedule.json, packs it into a LittleFS image,
# and flashes it to the sender node's filesystem partition.
#
# NodeMCU v2 (4 MB flash) layout:
#   Sketch:   0x000000 – 0x1FFFFF  (2 MB)
#   LittleFS: 0x200000 – 0x3FA000  (~2 MB = 2072576 bytes)
#   EEPROM/config occupy the last 24 KB.

set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DATA_DIR="$REPO_DIR/data"

MKLITTLEFS="$HOME/.arduino15/packages/esp8266/tools/mklittlefs/3.1.0-gcc10.3-e5f9fec/mklittlefs"
ESPTOOL="$(command -v esptool.py 2>/dev/null || echo "$HOME/.arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/esptool/esptool.py")"

FS_START=0x200000
FS_SIZE=2072576    # bytes (0x1FA000)
FS_PAGE=256
FS_BLOCK=8192

IMAGE="/tmp/littlefs.bin"

if [ ! -f "$MKLITTLEFS" ]; then
  echo "ERROR: mklittlefs not found at $MKLITTLEFS"
  echo "       Check your ESP8266 Arduino core installation."
  exit 1
fi

if [ ! -f "$ESPTOOL" ]; then
  echo "ERROR: esptool.py not found at $ESPTOOL"
  exit 1
fi

if [ ! -d "$DATA_DIR" ]; then
  echo "ERROR: data/ directory not found at $DATA_DIR"
  exit 1
fi

echo "=== Building LittleFS image from $DATA_DIR ==="
"$MKLITTLEFS" -c "$DATA_DIR" -p "$FS_PAGE" -b "$FS_BLOCK" -s "$FS_SIZE" "$IMAGE"

echo "=== Uploading to $PORT at flash offset $FS_START ==="
python3 "$ESPTOOL" --port "$PORT" --baud 460800 write_flash "$FS_START" "$IMAGE"

echo "=== Done. Restart the node to load the new config. ==="
