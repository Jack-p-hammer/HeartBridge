# actuation.py
"""Functions that dictate actuation of the plunger, including zeroing and compressions. These functions are non-blocking.

NOTE: ALL SETPOINTS ARE IN METERS, AND COMPRESSION SETPOINTS ARE RELATIVE TO ZEROED POSITION. DO NOT CONVERT TO ROTATIONS, THAT IS ALL DONE WITHIN THE MOTEUS THREAD
"""

# External imports
import logging
import time
import math

# Internal imports
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from motor import MoteusThread, CONTROLLER_ID


COMPRESSION_DEPTH_CM: float = 7.0 # depth of compressions in cm
zeroing_start_time: float = 0.0 # time.monotonic() of when zeroing started
compression_start_time: float = 0.0 # time.monotonic() of when compressions started, used to compute trapezoidal waveform

starting_position: float = 0.0 # The position of the plunger on startup
zeroed_position: float = 0.0 # The position of the plunger when zeroing is complete

_motor_controller: MoteusThread  # The shared moteus controller instance

def init_motor() -> ErrorCode:
    """Initialize moteus-x1 motor driver thread and controller settings.
    Motor controller communication is done via CAN-FD board over USB (from raspi side)

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    global _motor_controller 
    logging.debug("Initializing motor driver")
    
    _motor_controller = MoteusThread(controller_id=CONTROLLER_ID)
    motor_error: ErrorCode = _motor_controller.start()
    
    if motor_error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor controller initialization failed: {motor_error}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION

def init_zeroing() -> ErrorCode:
    """Perform pre-zeroing setup. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    global zeroing_start_time
    zeroing_start_time = time.monotonic()
    logging.debug("Zeroing initialized")
    
    return ErrorCode.NORMAL_OPERATION

def zeroing() -> ErrorCode:
    """Perform the zeroing procedure. Non-blocking.
    The procedure is as follows:
    1. Move the plunger down at a constant velocity
    2. Repeatedly poll the force sensor (via read_sensors) until the force exceeds the threshold
    3. Stop the plunger and set the current position as zero

    Returns:
        ErrorCode: Normal while zeroing, ZEROING_FINISHED when finished, ERROR_ZEROING_FAILURE if failed (timeout, max extension reached)
    """
    return ErrorCode.NORMAL_OPERATION


def init_compressions() -> ErrorCode:
    """Perform pre-compressions setup. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    global compression_start_time
    compression_start_time = time.monotonic()
    logging.debug("Compressions initialized")
    
    return ErrorCode.NORMAL_OPERATION


def compressions() -> ErrorCode:
    """Perform compressions. Non-blocking.

    Returns:
        ErrorCode: Normal operation while compressing, ERROR_SENSOR_FAILURE if sensor failure detected
    """
    return ErrorCode.NORMAL_OPERATION


def stop_compressions() -> ErrorCode:
    """Return plunger to zeroed position for compressions pause. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    logging.debug("Stopping compressions")
    return ErrorCode.NORMAL_OPERATION

def computeCompressionSetpoint() -> float:
    """Computes the current compression setpoint from the trapezoidal waveform, returns position relative to zeroed position

    Returns:
        float: compression setpoint, relative to zeroed position, in meters
    """
    time_sec: float = (time.monotonic() - compression_start_time)
    cycle_time: float = time_sec % 0.56 # One cycle takes 0.56 seconds
    outputPos_cm: float = 0.0
    
    # Piecewise trapezoidal profile (periodic)
    # Positive rotation of motor is down on rack, so down is positive up is negative
    if cycle_time < 0.12:
        outputPos_cm = 0.0
    elif cycle_time < 0.24:
        outputPos_cm = COMPRESSION_DEPTH_CM * (cycle_time - 0.12) / 0.12
    elif cycle_time < 0.323:
        outputPos_cm = COMPRESSION_DEPTH_CM
    elif cycle_time < 0.56:
        outputPos_cm = COMPRESSION_DEPTH_CM * (1.0 - (cycle_time - 0.323) / (0.56 - 0.323))
    
    # Convert to meters and return
    return outputPos_cm / 100.0