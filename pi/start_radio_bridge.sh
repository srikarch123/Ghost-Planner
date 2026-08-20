#!/usr/bin/env bash
# Starts the radio bridge on the Pi: receives drive commands over the
# RFD900x radio and drives the Cube locally over USB via rover_control.py.
# Both the radio and the Cube are auto-detected by USB VID:PID -- no fixed
# device names needed, survives unplug/replug or reboots even if the
# /dev/ttyUSBx or /dev/ttyACMx numbers change.
# Runs in the foreground -- Ctrl+C stops it cleanly. Safe to re-run: stops
# any previous instance (background or foreground) first.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

RADIO_PORT="${1:-}"      # optional override; leave empty to auto-detect
RADIO_BAUD="${2:-57600}"
CUBE_PORT="${3:-}"       # optional override; leave empty to auto-detect
CUBE_BAUD="${4:-115200}"

# stop any previous instance (bracket trick avoids pkill matching itself)
pkill -f '[r]adio_bridge.py' 2>/dev/null || true
sleep 1

echo "Starting radio bridge -- Ctrl+C to stop"
exec python3 -u radio_bridge.py "$RADIO_PORT" "$RADIO_BAUD" "$CUBE_PORT" "$CUBE_BAUD"
