# error_codes.py

from enum import Enum

class ErrorCode(Enum):
    EXIT_UNKNOWN               = 0x00
    ERROR_INIT_FAILURE         = 0x01
    ERROR_UNKNOWN_IMAGE        = 0x02
    ERROR_SENSOR_FAILURE       = 0x03
    ERROR_IMU_KNEEL_FAILURE    = 0x04
    ERROR_ZEROING_FAILURE      = 0x05
    NORMAL_OPERATION           = 0x7F