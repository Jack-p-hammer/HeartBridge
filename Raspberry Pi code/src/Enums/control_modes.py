# control_modes.py

from enum import Enum, auto

class ControlMode(Enum):
    ZEROING = auto()
    COMPRESSIONS = auto()
    # Deprecated: the main loop streams every position and velocity along the stroke profile, instead of the moteus
    # planning each stroke (COMPRESSIONS). Kept for profiles the moteus planner can't produce
    COMPRESSIONS_STREAMED_DEPRECATED = auto()
    HOLD_POSITION = auto()
    PAUSE_RETRACT = auto()
    ABORT_RETRACT = auto()
    