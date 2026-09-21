# actuation.py
"""Functions that dictate actuation of the plunger, including zeroing and compressions. These functions are non-blocking.

NOTE: ALL SETPOINTS ARE IN METERS, AND COMPRESSION SETPOINTS ARE RELATIVE TO ZEROED POSITION. DO NOT CONVERT TO ROTATIONS, THAT IS ALL DONE WITHIN THE MOTEUS THREAD
"""

# External imports
import logging
import time
import math

# Internal imports
import sensing # So long as init_motor never needs anything from sensing, this is fine. If it does, we have a circular import problem
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from moteus_thread import MoteusThread, CONTROLLER_ID, ZEROING_VELOCITY_MPS


COMPRESSION_DEPTH_CM: float = 7.0 # depth of compressions in cm, because compression setpoint math is in cm
EXTENSION_STROKE_LIMIT_M: float = 0.0254 * 8 # maximum extension of plunger in meters, sets extension limit for zeroing, 8" in meters

# TODO: Determine if this timeout should have extra buffer on top of time to take max extension
ZEROING_TIMEOUT_SEC: float = EXTENSION_STROKE_LIMIT_M / ZEROING_VELOCITY_MPS # maximum time to spend zeroing, in seconds

# Battery voltage threshold for low battery detection. TODO: Calibrate this value based on actual battery performance.
BATTERY_THRESHOLD_V: float = 21.6  # 6S LiPo battery, 3.6V per cell minimum, 4.2V per cell maximum. 6S = 21.6V minimum, 25.2V maximum.

# See moteus_thread.py for hardware and velocity constants
# See sensing.py for force threshold constants

zeroing_start_time: float = 0.0 # time.monotonic() of when zeroing started
compression_start_time: float = 0.0 # time.monotonic() of when compressions started, used to compute trapezoidal waveform

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
    global ZEROING_TIMEOUT_SEC, EXTENSION_STROKE_LIMIT_M
    global _motor_controller, zeroing_start_time, zeroed_position
    
    # Check for timeout
    if time.monotonic() - zeroing_start_time > ZEROING_TIMEOUT_SEC:
        logging.error("Zeroing failed: timeout")
        return ErrorCode.ERROR_ZEROING_FAILURE

    # Check for motor errors
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed: motor error {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
        
    # Get motor state
    motor_state = _motor_controller.get_state()
    
    # Check for max extension
    # TODO: Check if this returns in revolutions or meters, and convert if necessary
    if motor_state.position > EXTENSION_STROKE_LIMIT_M:
        logging.error("Zeroing failed: max extension reached")
        return ErrorCode.ERROR_ZEROING_FAILURE
    
    # Set motor command to zeroing, can be done repeatedly since it is non-blocking
    _motor_controller.set_target(ControlMode.ZEROING)
    
    # Read sensors to determine if chest reached
    # ZEROING_FINISHED exists to transmit zeroing success to state machine, just pass along results of sensor read
    # All sensor reading logic is in sensing.py
    return sensing.read_sensors(ControlMode.ZEROING)


def init_compressions() -> ErrorCode:
    """Perform pre-compressions setup. Non-blocking.

    Returns:
        ErrorCode: Normal operation
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
    global _motor_controller
    
    # Read sensors
    error = sensing.read_sensors(ControlMode.COMPRESSIONS)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on sensors: {error}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    # Check for motor failure
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on motor: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Update compression setpoint
    error = _motor_controller.set_target(ControlMode.COMPRESSIONS, computeCompressionSetpoint())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
    
    return ErrorCode.NORMAL_OPERATION


def pause_compressions() -> ErrorCode:
    """Pause compressions by returning to zeroed position. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_SENSOR_FAILURE if failed
    """
    global _motor_controller
    
    # Read sensors
    error = sensing.read_sensors(ControlMode.PAUSE_RETRACT)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Pause retract failed on sensors: {error}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    # Check for motor failure
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Pause retract failed on motor: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
    
    # Update motor command to pause retract
    # This works because motor setpoints are retained between calls
    error = _motor_controller.set_target(ControlMode.PAUSE_RETRACT)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Pause retract failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
    return ErrorCode.NORMAL_OPERATION


def abort_compressions() -> ErrorCode:
    """Return plunger to starting position for compressions abort. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    global _motor_controller
    
    # Abort process does not call sensors, just return to absolute zero position ASAP
    # If this happens we don't care about state, but whatever
    
    # Update motor command to pause retract, non-blocking so it can be called repeatedly
    error = _motor_controller.set_target(ControlMode.ABORT_RETRACT)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Abort compressions failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
    
    logging.info("Stopping compressions")
    return ErrorCode.NORMAL_OPERATION


def computeCompressionSetpoint() -> float:
    """Computes the current compression setpoint from the trapezoidal waveform, returns position relative to zeroed position

    Returns:
        float: compression setpoint, relative to zeroed position, in meters
    """
    global compression_start_time
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


def read_rotary_encoder() -> float:
    """Read the rotary encoder from the moteus-x1 and return the value in rotations

    Returns:
        float: Rotary position in rotations
    """
    # TODO: Implement actual rotary encoder reading logic
    global _motor_controller
    return _motor_controller.get_rotary_position()

def battery_check() -> ErrorCode:
    """Ensure sufficient charge for operation. 
    TODO: Split into startup battery check and continuous monitoring. Higher threshold on startup to prevent mid-operation shutdown, but still check battery voltage during operation.

    Returns:
        ErrorCode: Normal if battery is sufficient, otherwise ERROR_LOW_BATTERY
    """
    global _motor_controller
    try:
        battery_voltage: float = _motor_controller.get_battery_voltage()
    except Exception as e:
        logging.error(f"Battery voltage read failed: {e}")
        # Motor failure because moteus let us down
        return ErrorCode.ERROR_MOTOR_FAILURE
    
    if battery_voltage < BATTERY_THRESHOLD_V:
        return ErrorCode.ERROR_LOW_BATTERY
    return ErrorCode.NORMAL_OPERATION