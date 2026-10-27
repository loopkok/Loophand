"""Loophand: tactile shear-force grasp control for the Aero Hand Open.

This package contains the self-developed control stack:

    driver    -- GEN3 tactile sensor serial driver
    controller-- PID, admittance and shear/slip grasp controllers
    viz       -- 3D hand visualisation and tuning scope
    app       -- PyQt5 control centre

The upstream Aero Hand Open repository (hardware, firmware, SDK, ROS 2)
lives in ``third_party/aero-hand-open``.
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
