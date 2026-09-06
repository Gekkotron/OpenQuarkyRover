#!/usr/bin/env bash
# OpenQuarkyRover — run `idf.py monitor` while recording every byte of the
# session to a timestamped log file. Also drops a cleaned copy with ANSI
# escapes stripped, ready to grep or paste back into a chat.
#
# Usage:
#   ./scripts/monitor.sh                                    # auto-detect port
#   QUARKY_PORT=/dev/cu.usbserial-10 ./scripts/monitor.sh   # explicit port
#
# Exit the monitor the usual way: Ctrl+].

set -euo pipefail

PROJ_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FIRMWARE_DIR="$PROJ_ROOT/firmware"
LOG_DIR="$PROJ_ROOT/logs"

# --- source ESP-IDF if not on PATH ---------------------------------------
if ! command -v idf.py >/dev/null 2>&1; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        # shellcheck disable=SC1091
        . "$HOME/esp/esp-idf/export.sh" >/dev/null
    else
        echo "ESP-IDF not on PATH and ~/esp/esp-idf/export.sh missing." >&2
        echo "Install ESP-IDF v5.x or source its export.sh manually." >&2
        exit 1
    fi
fi

# --- resolve serial port -------------------------------------------------
PORT="${QUARKY_PORT:-}"
if [ -z "$PORT" ]; then
    for c in /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$c" ] || continue
        PORT="$c"
        break
    done
fi
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "No serial port found. Plug the Quarky Intellio in via USB-C and retry." >&2
    echo "To override: QUARKY_PORT=/dev/cu.usbserial-NN $0" >&2
    exit 1
fi

# --- release any stale port holders (best-effort) ------------------------
if [ -x "$PROJ_ROOT/scripts/kill-monitor.sh" ]; then
    QUARKY_PORT="$PORT" "$PROJ_ROOT/scripts/kill-monitor.sh" >/dev/null 2>&1 || true
fi

# --- prepare log path ----------------------------------------------------
mkdir -p "$LOG_DIR"
TS=$(date +%Y%m%d-%H%M%S)
LOG_FILE="$LOG_DIR/session-$TS.log"
CLEAN_FILE="$LOG_DIR/session-$TS.clean.log"

echo "==> port    : $PORT"
echo "==> raw log : $LOG_FILE"
echo "==> Ctrl+]  : exit monitor"
echo

cd "$FIRMWARE_DIR"

# --- run monitor under script(1) ----------------------------------------
# BSD (macOS): script [-q] file command...
# util-linux : script -q -c "command" file
if [ "$(uname)" = "Darwin" ]; then
    script -q "$LOG_FILE" idf.py -p "$PORT" monitor || true
else
    script -q -c "idf.py -p '$PORT' monitor" "$LOG_FILE" || true
fi

# --- post-process: strip ANSI escapes into a readable copy --------------
if [ -f "$LOG_FILE" ]; then
    sed -E $'s/\x1b\\[[0-9;?]*[a-zA-Z]//g; s/\r//g' "$LOG_FILE" 2>/dev/null > "$CLEAN_FILE" || true
    echo
    echo "==> raw   log : $LOG_FILE"
    echo "==> clean log : $CLEAN_FILE"
fi
