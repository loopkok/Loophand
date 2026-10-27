#!/usr/bin/env bash
# Launch the Loophand control centre without installing the package.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

exec "${PYTHON:-python}" -m loophand.app.main_gui "$@"
