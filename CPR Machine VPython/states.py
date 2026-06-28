# states.py

from enum import Enum, auto

class CPRState(Enum):
    STARTUP             = auto()
    UNFOLD_CUT_CLOTHES  = auto()
    ALIGNMENT           = auto()
    ZEROING_PREP        = auto()
    ZEROING             = auto()
    COMPRESSION_PREP    = auto()
    COMPRESSION         = auto()
    PAUSE               = auto()
    ABORT               = auto()
    KNEEL_FAILURE       = auto()