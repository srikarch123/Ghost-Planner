#!/usr/bin/env bash
# Starts the manual-control API backend on this PC (talks to the Pi over the
# RFD900x radio). The radio is auto-detected by USB VID:PID -- no fixed
# device name needed, survives unplug/replug even if the /dev/ttyUSBx number
# changes. Runs in the foreground -- Ctrl+C stops it cleanly. Safe to re-run:
# stops any previous instance (background or foreground) first.
#
# The UI itself is the React app in frontend/ -- run it separately with
# ./start_frontend.sh during development (see README.md).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

HTTP_PORT="${1:-5050}"
RADIO_PORT="${2:-}"     # optional override; leave empty to auto-detect
RADIO_BAUD="${3:-57600}"

# stop any previous instance (bracket trick avoids pkill matching itself)
pkill -f '[b]ackend/server.py' 2>/dev/null || true
sleep 1

echo "Starting backend API -- Ctrl+C to stop"
echo "API: http://localhost:${HTTP_PORT}/  (run ./start_frontend.sh separately for the UI)"
exec ./backend/venv/bin/python3 backend/server.py "$HTTP_PORT" "$RADIO_PORT" "$RADIO_BAUD"
