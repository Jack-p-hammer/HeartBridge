# actuation.py
"""Functions that dictate actuation of the plunger, including zeroing and compressions. These functions are non-blocking.

The plunger is a nested two-stage actuator:
- Two lead screws, driven by steppers sharing one STEP/DIR input (stepper.py), move the whole carriage. Used for zeroing (bringing the plunger down to the chest) and fully retracting on abort.
- The moteus-driven BLDC moves the plunger along a ball screw within the carriage (moteus_thread.py). Used for compressions, with only enough travel for compressions plus safety room.

The system is assumed to start fully retracted on both stages.

NOTE: ALL SETPOINTS ARE IN METERS, AND COMPRESSION SETPOINTS ARE RELATIVE TO ZEROED POSITION. DO NOT CONVERT TO ROTATIONS OR STEPS, THAT IS ALL DONE WITHIN moteus_thread.py AND stepper.py
"""

# External imports
import logging
import time
import math

# Internal imports
import sensing
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from moteus_thread import MoteusThread, CONTROLLER_ID
import stepper
from stepper import LEAD_SCREW_TRAVEL_M, ZEROING_VELOCITY_MPS, RETRACT_VELOCITY_MPS


COMPRESSION_DEPTH_CM: float = 7.0 # depth of compressions in cm, because compression setpoint math is in cm
EXTENSION_STROKE_LIMIT_M: float = LEAD_SCREW_TRAVEL_M # maximum carriage extension in meters, zeroing fails if the chest isn't found by here

# 1.5x buffer on top of the time to reach max extension, for accel ramps. TODO: Tune
ZEROING_TIMEOUT_SEC: float = 1.5 * EXTENSION_STROKE_LIMIT_M / ZEROING_VELOCITY_MPS # maximum time to spend zeroing, in seconds

# See moteus_thread.py for BLDC/ball screw constants, stepper.py for stepper/lead screw constants
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
    logging.debug("Initializing motor...")
    
    _motor_controller = MoteusThread(controller_id=CONTROLLER_ID)
    motor_error: ErrorCode = _motor_controller.start()
    
    if motor_error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor controller initialization failed: {motor_error}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION

def init_steppers(pi) -> ErrorCode:
    """Initialize the lead screw steppers on the shared pigpio instance.

    Args:
        pi (pigpio.pi): The shared pigpio instance from HMI.init_Pi()

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    error: ErrorCode = stepper.init_steppers(pi)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Stepper initialization failed: {error}")
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
    logging.debug("Zeroing initialized")
    
    return ErrorCode.NORMAL_OPERATION


def zeroing() -> ErrorCode:
    """Perform the zeroing procedure. Non-blocking.
    The procedure is as follows:
    1. Hold the plunger just below the top of its ball screw travel
    2. Command the steppers towards max carriage extension at zeroing velocity
    3. Repeatedly poll the force sensor (via read_sensors) until the force exceeds the threshold
    4. Halt the steppers. The carriage holds there for the rest of compressions

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
        
    # Hold the plunger in place on the ball screw, can be done repeatedly since it is non-blocking
    error = _motor_controller.set_target(ControlMode.ZEROING,
                                         sensing.get_rotary_zero_position(),
                                         sensing.get_rotary_absolute_zero_position())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Check for max extension
    if stepper.get_position() >= EXTENSION_STROKE_LIMIT_M:
        logging.error("Zeroing failed: max extension reached")
        return ErrorCode.ERROR_ZEROING_FAILURE

    # Lower the carriage towards max extension, can be done repeatedly since it is non-blocking
    error = stepper.set_target(EXTENSION_STROKE_LIMIT_M, ZEROING_VELOCITY_MPS)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed on stepper: {error}")
        stepper.halt()
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Read sensors to determine if chest reached
    # ZEROING_FINISHED exists to transmit zeroing success to state machine, just pass along results of sensor read
    # All sensor reading logic is in sensing.py
    result: ErrorCode = sensing.read_sensors(ControlMode.ZEROING)
    if result != ErrorCode.NORMAL_OPERATION:
        # Chest found (or a sensor failure), stop the carriage either way
        stepper.halt()
        if result == ErrorCode.ZEROING_FINISHED:
            logging.info(f"Carriage zeroed at {stepper.get_position()} m")
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

    # Carriage holds at its zeroed position, finish playing out any steps still queued from zeroing
    error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on stepper: {error}")
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

    # Carriage stays at its zeroed position so compressions can resume without re-zeroing
    error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Pause retract failed on stepper: {error}")
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
    """Return plunger to the top of its ball screw travel for compressions abort. Non-blocking.
    NOTE: This does not move the carriage, since PAUSE also calls this every tick. Use retract_carriage() to fully retract.

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


def retract_carriage() -> ErrorCode:
    """Retract the carriage to its home (startup) position with the steppers. Non-blocking, must be called every tick
    until the carriage is home. Like abort_compressions(), does not read sensors.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_MOTOR_FAILURE if failed
    """
    error = stepper.set_target(0.0, RETRACT_VELOCITY_MPS)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Carriage retract failed on stepper: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
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
    # Positive rotation of motor is down on ball screw, so down is positive up is negative
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