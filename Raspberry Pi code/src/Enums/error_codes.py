# error_codes.py

from enum import Enum, auto


class ErrorCode(Enum):
    EXIT_UNKNOWN = auto()
    ERROR_INIT_FAILURE = auto()
    ERROR_UNKNOWN_IMAGE = auto()
    ERROR_PYGAME_INIT_FAILURE = auto() # Particularly bad, dedicated fatal error
    ERROR_PYGAME_FAILURE = auto()
    WARNING_PYGAME_PUMP_FAILURE = auto()
    ERROR_PI_DAEMON_FAILURE = auto()
    ERROR_SENSOR_FAILURE = auto()
    ERROR_IMU_KNEEL_FAILURE = auto()
    ERROR_MOTOR_FAILURE = auto()
    ERROR_ZEROING_FAILURE = auto()
    ERROR_LOW_BATTERY = auto()
    ZEROING_FINISHED = auto()
    NORMAL_OPERATION = 0x7F
