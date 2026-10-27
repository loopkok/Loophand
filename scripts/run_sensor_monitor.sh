#!/usr/bin/env bash
# Interactive GEN3 tactile sensor monitor (raw protocol + auto-push stream).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PYTHON:-python}" "$ROOT/tools/read_tactile_sensor.py" "$@"
