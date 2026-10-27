"""Filesystem locations for packaged assets.

Keeping these in one module means the controllers and the GUI can be started
from any working directory (``python -m ...``, console script, IDE runner).
"""

from __future__ import annotations

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
ASSETS_DIR = PACKAGE_DIR / "assets"

#: Left-hand URDF used for kinematics (Pinocchio) and 3D visualisation.
URDF_PATH = ASSETS_DIR / "aero_hand_open_left.urdf"

#: STL meshes referenced by the URDF.
MESHES_DIR = ASSETS_DIR / "meshes"

#: Scratch directory for URDFs rewritten with absolute mesh paths.
GENERATED_DIR = ASSETS_DIR / "generated"

__all__ = [
    "PACKAGE_DIR",
    "ASSETS_DIR",
    "URDF_PATH",
    "MESHES_DIR",
    "GENERATED_DIR",
]
