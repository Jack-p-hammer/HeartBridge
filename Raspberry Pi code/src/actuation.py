# actuation.py
"""Functions that dictate actuation of the plunger, including zeroing and compressions. These functions are non-blocking.

The plunger is driven along a single ball screw by the moteus-driven BLDC (moteus_thread.py), which covers the whole
stroke: zeroing (bringing the plunger down to the chest), compressions, and retracting on pause/abort.

The plunger is assumed to start fully retracted, at the top of the ball screw.

NOTE: ALL SETPOINTS ARE IN METERS, AND COMPRESSION SETPOINTS ARE RELATIVE TO ZEROED POSITION. DO NOT CONVERT TO ROTATIONS, THAT IS ALL DONE WITHIN moteus_thread.py
"""

# External imports
import logging
import time
import math

# Internal imports
import sensing
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from moteus_thread import (MoteusThread, MotorState, CONTROLLER_ID, BLDC_ROTATIONS_PER_M, BALL_SCREW_TRAVEL_M,
                           ZEROING_VELOCITY_MPS, MOTEUS_FAULT_MODE)


COMPRESSION_DEPTH_CM: float = 7.0 # depth of compressions in cm, because compression setpoint math is in cm
EXTENSION_STROKE_LIMIT_M: float = BALL_SCREW_TRAVEL_M - COMPRESSION_DEPTH_CM / 100.0 # maximum plunger extension in meters, zeroing fails if the chest isn't found by here
PLUNGER_HOME_TOLERANCE_M: float = 0.001 # meters from the startup position that counts as fully retracted, see retract_plunger()

# 1.5x buffer on top of the time to reach max extension, for accel ramps. TODO: Tune
ZEROING_TIMEOUT_SEC: float = 1.5 * EXTENSION_STROKE_LIMIT_M / ZEROING_VELOCITY_MPS # maximum time to spend zeroing, in seconds

# See moteus_thread.py for BLDC/ball screw and velocity constants
# See sensing.py for the zeroing current spike and force threshold constants

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
    logging.debug("Initializing motor...")
    
    _motor_controller = MoteusThread(controller_id=CONTROLLER_ID)
    motor_error: ErrorCode = _motor_controller.start()
    
    if motor_error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor controller initialization failed: {motor_error}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION

def get_motor_controller() -> MoteusThread:
    """Get the shared moteus controller instance.

    Returns:
        MoteusThread: The shared moteus controller instance
    """
    global _motor_controller
    return _motor_controller

def init_zeroing() -> ErrorCode:
    """Perform pre-zeroing setup. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    global zeroing_start_time

    zeroing_start_time = time.monotonic()

    # Start a fresh current spike detection run
    error: ErrorCode = sensing.init_zeroing()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing initialization failed: {error}")
        return ErrorCode.ERROR_ZEROING_FAILURE

    logging.debug("Zeroing initialized")
    return ErrorCode.NORMAL_OPERATION


def zeroing() -> ErrorCode:
    """Perform the zeroing procedure. Non-blocking.
    The procedure is as follows:
    1. Move the plunger down the ball screw at a constant velocity, with a reduced torque cap
    2. Repeatedly check the motor's q current (via read_sensors) until it spikes above its free-running baseline,
       same as demo/current_zeroing/current_zeroing.py
    3. Stop the plunger there. sensing.zero_position() then records it as the zeroed position

    Returns:
        ErrorCode: Normal while zeroing, ZEROING_FINISHED when finished, ERROR_ZEROING_FAILURE if failed (timeout, max extension reached)
    """
    global ZEROING_TIMEOUT_SEC, EXTENSION_STROKE_LIMIT_M
    global _motor_controller, zeroing_start_time

    # Check for timeout
    if time.monotonic() - zeroing_start_time > ZEROING_TIMEOUT_SEC:
        logging.error("Zeroing failed: timeout")
        return ErrorCode.ERROR_ZEROING_FAILURE

    # Check for motor errors
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed: motor error {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Check for a latched controller fault, the q current is meaningless once the motor stops driving
    fault_state: MotorState = _motor_controller.get_state()
    if fault_state.mode == MOTEUS_FAULT_MODE:
        logging.error(f"Zeroing failed: moteus fault {fault_state.fault}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Check for max extension, in meters of plunger travel down from the startup (top of ball screw) position
    travel_m: float = (_motor_controller.get_rotary_position() - sensing.get_rotary_absolute_zero_position()) / BLDC_ROTATIONS_PER_M
    if travel_m >= EXTENSION_STROKE_LIMIT_M:
        logging.error(f"Zeroing failed: max extension reached at {travel_m} m")
        return ErrorCode.ERROR_ZEROING_FAILURE

    # Move the plunger down at zeroing velocity, can be done repeatedly since it is non-blocking
    error = _motor_controller.set_target(ControlMode.ZEROING,
                                         sensing.get_rotary_zero_position(),
                                         sensing.get_rotary_absolute_zero_position())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Read sensors (including the motor current spike check) to determine if chest reached
    # ZEROING_FINISHED exists to transmit zeroing success to state machine, just pass along results of sensor read
    # All sensor reading logic is in sensing.py
    result: ErrorCode = sensing.read_sensors(ControlMode.ZEROING)
    if result != ErrorCode.NORMAL_OPERATION:
        # Chest found (or a sensor failure), stop the plunger either way
        error = _motor_controller.set_target(ControlMode.HOLD_POSITION,
                                             sensing.get_rotary_zero_position(),
                                             sensing.get_rotary_absolute_zero_position())
        if error != ErrorCode.NORMAL_OPERATION:
            logging.error(f"Zeroing failed to stop the plunger: {error}")
            return ErrorCode.ERROR_MOTOR_FAILURE
        if result == ErrorCode.ZEROING_FINISHED:
            logging.info(f"Plunger zeroed at {travel_m} m")
    return result


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
    if error == ErrorCode.ERROR_IMU_KNEEL_FAILURE:
        logging.error(f"Compressions failed on IMU kneel: {error}")
        return ErrorCode.ERROR_IMU_KNEEL_FAILURE
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on sensors: {error}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    # Check for motor failure
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on motor: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Update compression setpoint
    error = _motor_controller.set_target(ControlMode.COMPRESSIONS, 
                                         sensing.get_rotary_zero_position(), 
                                         sensing.get_rotary_absolute_zero_position(), 
                                         computeCompressionSetpoint())
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
    if error == ErrorCode.ERROR_IMU_KNEEL_FAILURE:
        logging.error(f"Compressions failed on IMU kneel: {error}")
        return ErrorCode.ERROR_IMU_KNEEL_FAILURE
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
    error = _motor_controller.set_target(ControlMode.PAUSE_RETRACT, 
                                         sensing.get_rotary_zero_position(), 
                                         sensing.get_rotary_absolute_zero_position())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Pause retract failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
    return ErrorCode.NORMAL_OPERATION


def abort_compressions() -> ErrorCode:
    """Return plunger to the top of its ball screw travel (the startup position) for compressions abort. Non-blocking.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    global _motor_controller
    
    # Abort process does not call sensors, just return to absolute zero position ASAP
    # If this happens we don't care about state, but whatever
    
    # Update motor command to pause retract, non-blocking so it can be called repeatedly
    error = _motor_controller.set_target(ControlMode.ABORT_RETRACT, 
                                         sensing.get_rotary_zero_position(), 
                                         sensing.get_rotary_absolute_zero_position())
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
    # Down (towards the chest) is positive. moteus_thread.py converts this to motor rotations, including the gear train's reversal
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

def retract_plunger() -> tuple[ErrorCode, bool]:
    """Retract the plunger to the top of its ball screw travel (the startup position), and report once it is there.
    Same motor command as abort_compressions(), without the log message, so it can be called every tick until the
    plunger is home. Like abort_compressions(), does not read sensors.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_MOTOR_FAILURE if failed
        bool: True if the plunger is within PLUNGER_HOME_TOLERANCE_M of the startup position, False otherwise
    """
    global _motor_controller

    error = _motor_controller.set_target(ControlMode.ABORT_RETRACT,
                                         sensing.get_rotary_zero_position(),
                                         sensing.get_rotary_absolute_zero_position())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Plunger retract failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE, False

    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Plunger retract failed on motor: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE, False

    travel_m: float = (_motor_controller.get_rotary_position() - sensing.get_rotary_absolute_zero_position()) / BLDC_ROTATIONS_PER_M
    if abs(travel_m) <= PLUNGER_HOME_TOLERANCE_M:
        logging.info(f"Plunger retracted to {travel_m} m")
        return ErrorCode.NORMAL_OPERATION, True
    return ErrorCode.NORMAL_OPERATION, False
