# actuation.py

import moteus
from error_codes import ErrorCode


def init_motor() -> ErrorCode:
    """Initialize moteus-x1 motor driver and controller settings

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    # TODO: moteus CAN init
    return ErrorCode.NORMAL_OPERATION


def zeroing() -> ErrorCode:
    """Perform the zeroing procedure. Non-blocking.

    Returns:
        ErrorCode: Normal while zeroing, ZEROING_FINISHED when finished, ERROR_ZEROING_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION


def init_compressions() -> ErrorCode:
    """Perform pre-compressions setup

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION


def compressions() -> ErrorCode:
    """Perform compressions. Non-blocking

    Returns:
        ErrorCode: Normal operation while compressing, ERROR_SENSOR_FAILURE if sensor failure detected
    """
    return ErrorCode.NORMAL_OPERATION


def stop_compressions() -> ErrorCode:
    """Return plunger to zeroed position for compressions pause

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION
