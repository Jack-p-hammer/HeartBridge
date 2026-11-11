# states.py

from enum import Enum, auto


class CPRState(Enum):
    INIT_STARTUP = auto()
    WAIT = auto()
    STARTUP = auto()
    UNFOLD_CUT_CLOTHES = auto()
    ALIGNMENT = auto()
    KNEELING = auto()
    ZEROING = auto()
    COMPRESSION_PREP = auto()
    COMPRESSION = auto()
    PAUSE = auto()
    ABORT = auto()
    KNEEL_FAILURE = auto()
    RESET_DEMO = auto()