#!/usr/bin/env bash
# OpenQuarkyRover — flash the M1 firmware onto the Quarky Intellio.
#
# Usage:
#   ./scripts/upload.sh                                     # auto-detect port, flash only
#   ./scripts/upload.sh monitor                             # flash + open monitor
#   QUARKY_PORT=/dev/cu.usbserial-10 ./scripts/upload.sh    # explicit port
#
# Requires ESP-IDF v5.x. If `idf.py` is not on PATH, ~/esp/esp-idf/export.sh
# is sourced automatically.

set -euo pipefail

PROJ_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIRMWARE_DIR="$PROJ_ROOT/firmware"

# --- source ESP-IDF if needed ---------------------------------------------
if ! command -v idf.py >/dev/null 2>&1; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        # shellcheck disable=SC1091
        . "$HOME/esp/esp-idf/export.sh" >/dev/null
    else
        echo "ESP-IDF not found on PATH and ~/esp/esp-idf/export.sh missing." >&2
        echo "Install ESP-IDF v5.x or source its export.sh manually." >&2
        exit 1
    fi
fi

# --- resolve serial port --------------------------------------------------
PORT="${QUARKY_PORT:-}"
if [ -z "$PORT" ]; then
    for candidate in /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$candidate" ] || continue
        PORT="$candidate"
        break
    done
fi

if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "No serial port found. Plug the Quarky Intellio in via USB-C and retry." >&2
    echo "To override: QUARKY_PORT=/dev/cu.usbserial-NN $0" >&2
    exit 1
fi

echo "==> port: $PORT"

# --- release any stale port holders (best-effort, silent) -----------------
if [ -x "$PROJ_ROOT/scripts/kill-monitor.sh" ]; then
    QUARKY_PORT="$PORT" "$PROJ_ROOT/scripts/kill-monitor.sh" >/dev/null 2>&1 || true
fi

# --- flash ----------------------------------------------------------------
cd "$FIRMWARE_DIR"
exec idf.py -p "$PORT" flash "$@"
