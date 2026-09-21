# control_modes.py

from enum import Enum, auto

class ControlMode(Enum):
    ZEROING = auto()
    COMPRESSIONS = auto()
    HOLD_POSITION = auto()
    PAUSE_RETRACT = auto()
    ABORT_RETRACT = auto()
    