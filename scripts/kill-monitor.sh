#!/usr/bin/env bash
# OpenQuarkyRover — release any process holding a Quarky serial port.
#
# Also cleans up orphan serial-monitor processes (esp_idf_monitor in a
# reconnect loop, a Python miniterm whose device was unplugged, etc.)
# that no longer hold a live port node. Without this, `idf.py monitor`
# happily loops forever trying to reopen /dev/cu.usbserial-* after the
# ESP resets, blocking the next `flash`.
#
# We only touch processes owned by $USER and only if their command
# matches a known serial tool.
#
# Usage:
#   ./scripts/kill-monitor.sh                                    # every port + orphans
#   QUARKY_PORT=/dev/cu.usbserial-10 ./scripts/kill-monitor.sh   # only that port

set -euo pipefail

# --- collect ports to inspect ---------------------------------------------
if [ -n "${QUARKY_PORT:-}" ]; then
    PORTS="$QUARKY_PORT"
    scan_orphans=false
else
    PORTS=""
    for candidate in /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$candidate" ] || continue
        PORTS="$PORTS $candidate"
    done
    scan_orphans=true
fi

# --- helper: request-then-force kill sequence -----------------------------
kill_pid() {
    local pid="$1"
    kill "$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
    fi
}

killed=0

# --- port-scoped kill: processes actively holding one of the ports --------
if [ -n "$PORTS" ]; then
    for port in $PORTS; do
        pids=$(lsof -t "$port" 2>/dev/null || true)
        if [ -z "$pids" ]; then
            echo "$port: free"
            continue
        fi

        for pid in $pids; do
            owner=$(ps -p "$pid" -o user= 2>/dev/null | tr -d ' ' || true)
            if [ "$owner" != "$USER" ]; then
                echo "$port: PID $pid owned by '$owner' (not $USER) — skipping"
                continue
            fi

            cmd=$(ps -ww -p "$pid" -o command= 2>/dev/null || true)
            short=$(printf '%s' "$cmd" | cut -c1-90)

            case "$cmd" in
                *esptool*|*esp_idf_monitor*|*idf_monitor*|*idf.py*|*Python*|*python*|*screen*|*minicom*|*miniterm*|*picocom*|*platformio*|*pio*)
                    echo "$port: killing PID $pid ($short)"
                    kill_pid "$pid"
                    killed=$((killed + 1))
                    ;;
                *)
                    echo "$port: PID $pid ($short) — not a recognized serial tool, skipping"
                    ;;
            esac
        done
    done
else
    echo "No serial port found."
fi

# --- name-scoped fallback: catch orphan monitors with no live port --------
# Runs only when the caller did NOT pin QUARKY_PORT (targeting a specific
# port shouldn't surprise-kill unrelated tools). Patterns stay tight —
# bare "python" or "idf.py" would nuke unrelated work like a build.
if [ "$scan_orphans" = true ]; then
    # pgrep is used (instead of ps | grep) because it excludes itself
    # from matches — a plain grep would match its own command line since
    # the pattern text contains "esptool", "esp_idf_monitor", etc.
    orphan_pattern='esp_idf_monitor|idf_monitor|esptool|miniterm|idf\.py[[:space:]]+monitor|(picocom|minicom|screen)[[:space:]]+/dev/'

    orphan_pids=$(pgrep -f -u "$USER" -- "$orphan_pattern" 2>/dev/null || true)
    for pid in $orphan_pids; do
        [ "$pid" = "$$" ] && continue
        # already killed in the port loop, or exited on its own?
        kill -0 "$pid" 2>/dev/null || continue
        cmd=$(ps -ww -p "$pid" -o command= 2>/dev/null || true)
        short=$(printf '%s' "$cmd" | cut -c1-90)
        echo "orphan: killing PID $pid ($short)"
        kill_pid "$pid"
        killed=$((killed + 1))
    done
fi

if [ "$killed" -gt 0 ]; then
    echo "Released $killed process(es)."
else
    echo "Nothing to release."
fi
