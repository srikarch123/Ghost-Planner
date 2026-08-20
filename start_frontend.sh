#!/usr/bin/env bash
# Starts the React frontend dev server (Vite). Talks to the backend API
# (./start_backend.sh, default http://localhost:5050) over HTTP/CORS.
# Runs in the foreground -- Ctrl+C stops it cleanly.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR/frontend"

exec npm run dev
