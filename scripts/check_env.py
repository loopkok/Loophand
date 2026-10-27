#!/usr/bin/env python3
"""Check that the runtime dependencies of the grasp stack are importable."""

from __future__ import annotations

import importlib.util
import sys

CHECKS = [
    ("numpy", "pip install numpy"),
    ("serial", "pip install pyserial"),
    ("pinocchio", "source /opt/ros/humble/setup.bash, or use a conda pinocchio build"),
    ("pyqtgraph", "pip install pyqtgraph"),
    ("PyQt5", "pip install PyQt5"),
    ("trimesh", "pip install trimesh"),
    ("scipy", "pip install scipy"),
    ("aero_open_sdk", "pip install -e third_party/aero-hand-open/sdk"),
]


def main() -> int:
    failed = False
    print(f"python: {sys.executable}")
    for name, hint in CHECKS:
        ok = importlib.util.find_spec(name) is not None
        print(f"[{'OK' if ok else 'MISSING'}] {name}")
        if not ok:
            failed = True
            print(f"        -> {hint}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
