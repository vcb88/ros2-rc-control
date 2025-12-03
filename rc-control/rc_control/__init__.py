"""
Hexapod Robot Control Package

This package provides ROS2 nodes for controlling a  robot with various 
input methods including RC receivers (PPM/SBUS), keyboard, and joystick.

Main components:
- Movement controller for hexapod gaits and actions
- Servo driver for hardware control
- Multiple input controller nodes (PPM, SBUS, keyboard, joystick)
- RC controller configuration and utilities

Author: Slava Karpizin
License: Apache License 2.0
"""

from .controller_config import TX16SConfig
from importlib.metadata import version

try:
    __version__ = version("rc-control")
except Exception:
    __version__ = "0.0.1"

# Module level constants
SERVO_COUNT = 18
LEGS_COUNT = 6
JOINTS_PER_LEG = 3

# Default movement parameters
DEFAULT_MOVEMENT_SPEED = 0.5
DEFAULT_TURNING_RADIUS = 0.2
DEFAULT_STEP_HEIGHT = 0.05
DEFAULT_STANCE_HEIGHT = 0.15

# Available gait types
GAIT_TYPES = [
    'tripod',     # Classic tripod gait
    'wave',       # Wave gait
    'ripple'      # Ripple gait
]

# Module exports
__all__ = [
    'TX16SConfig',
    'SERVO_COUNT',
    'LEGS_COUNT',
    'JOINTS_PER_LEG',
    'DEFAULT_MOVEMENT_SPEED',
    'DEFAULT_TURNING_RADIUS',
    'DEFAULT_STEP_HEIGHT',
    'DEFAULT_STANCE_HEIGHT',
    'GAIT_TYPES',
] 
