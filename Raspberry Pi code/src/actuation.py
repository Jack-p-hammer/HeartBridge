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
from zeroing_key_test import move_carriage


COMPRESSION_DEPTH_CM: float = 5.08 # depth of compressions in cm (2 in, same as demo/jog_compressions.py). AHA adult depth is 2 to 2.4 in
COMPRESSIONS_PER_MIN: float = 110.0 # AHA rate is 100 to 120 per minute, same as demo/jog_compressions.py
STROKE_SEC: float = 60.0 / COMPRESSIONS_PER_MIN / 2.0 # time for one stroke in one direction, half a compression
# Compressions work like demo/jog_compressions.py: the moteus is only ever sent the extreme of the current stroke (the
# bottom, then the top), with its speed capped at COMPRESSION_SPEED_MPS and its acceleration capped at
# COMPRESSION_ACCEL_MPS2. So each stroke speeds up at the acceleration cap, cruises at the speed cap, and slows down at
# the acceleration cap to stop at the extreme. These caps make that take exactly STROKE_SEC, a third each:
# reaching the speed cap in a third of the stroke time needs a cap of 1.5 * depth / time and 4.5 * depth / time^2
COMPRESSION_SPEED_MPS: float = 1.5 * (COMPRESSION_DEPTH_CM / 100.0) / STROKE_SEC # speed cap, meters per second
COMPRESSION_ACCEL_MPS2: float = 4.5 * (COMPRESSION_DEPTH_CM / 100.0) / STROKE_SEC ** 2 # acceleration cap, meters per second squared
EXTENSION_STROKE_LIMIT_M: float = LEAD_SCREW_TRAVEL_M # maximum carriage extension in meters, zeroing fails if the chest isn't found by here

# 1.5x buffer on top of the time to reach max extension, for accel ramps. TODO: Tune
ZEROING_TIMEOUT_SEC: float = 1.5 * EXTENSION_STROKE_LIMIT_M / ZEROING_VELOCITY_MPS # maximum time to spend zeroing, in seconds

# See moteus_thread.py for BLDC/ball screw constants, stepper.py for stepper/lead screw constants
# See sensing.py for force threshold constants

zeroing_start_time: float = 0.0 # time.monotonic() of when zeroing started
compression_start_time: float = 0.0 # time.monotonic() of when compressions started, used to compute trapezoidal waveform
retract_target_m: float | None = None # Carriage position the current retraction ends at, set on its first tick, None while not retracting

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
    global retract_target_m
    retract_target_m = None  # stepper.init_steppers() zeroes the extending step count, so no retraction is in progress

    error: ErrorCode = stepper.init_steppers(pi)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Stepper initialization failed: {error}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION

def check_motor() -> ErrorCode:
    """Check the motor is responding (replying and following commands, see MoteusThread.get_last_error()). Non-blocking.
    Call every tick once the motor is up, so the motor not responding aborts from any state.

    Returns:
        ErrorCode: NORMAL_OPERATION if the motor is responding, ERROR_MOTOR_FAILURE otherwise
    """
    global _motor_controller
    error = _motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor check failed: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE
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

    # Update compression setpoint: the extreme of the current stroke. The speed and acceleration caps set how the moteus
    # moves there, so this only changes twice per compression
    error = _motor_controller.set_target(ControlMode.COMPRESSIONS,
                                         sensing.get_rotary_zero_position(),
                                         sensing.get_rotary_absolute_zero_position(),
                                         computeCompressionSetpoint(),
                                         compressionVelocityLimit=COMPRESSION_SPEED_MPS,
                                         compressionAccelLimit=COMPRESSION_ACCEL_MPS2)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Compressions failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    return ErrorCode.NORMAL_OPERATION


def compressions_streamed_deprecated() -> ErrorCode:
    """DEPRECATED: use compressions(). Perform compressions by streaming every position and velocity along the stroke
    profile from the main loop, instead of sending the stroke's extreme with speed and acceleration caps. Kept for
    profiles that capping speed and acceleration can't produce (e.g. a hold at the bottom of each stroke).
    Follows the main loop's tick, so late ticks and lost frames show up in the motion. Non-blocking.
    Call init_compressions() first, same as compressions().

    Returns:
        ErrorCode: Normal operation while compressing, ERROR_SENSOR_FAILURE if sensor failure detected
    """
    global _motor_controller

    # Read sensors. Same sensor limits as compressions(), so the sensing modules don't need the deprecated mode
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

    # Update compression setpoint: the current position along the profile, and its velocity
    position_m, velocity_mps = computeCompressionSetpoint_streamed_deprecated()
    error = _motor_controller.set_target(ControlMode.COMPRESSIONS_STREAMED_DEPRECATED,
                                         sensing.get_rotary_zero_position(),
                                         sensing.get_rotary_absolute_zero_position(),
                                         position_m,
                                         compressionVelocity=velocity_mps)
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


def reset_motor() -> ErrorCode:
    """Put the motor back the way it is after a power cycle: target back to the startup default (hold position) and
    the same stop sequence the moteus thread sends at startup. Non-blocking, the stop goes out on the moteus thread's
    next cycle.

    Returns:
        ErrorCode: Normal operation
    """
    global _motor_controller
    _motor_controller.request_stop_sequence()
    logging.info("Motor reset requested")
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
    

def computeCompressionSetpoint() -> float:
    """Computes the current compression setpoint, returns position relative to zeroed position.

    Same as demo/jog_compressions.py: the setpoint is the extreme of the current stroke, COMPRESSION_DEPTH_CM down for
    the first half of each cycle, then back to the zeroed position for the second half, at COMPRESSIONS_PER_MIN.
    The motion in between comes from the moteus capping speed at COMPRESSION_SPEED_MPS and acceleration at
    COMPRESSION_ACCEL_MPS2 on its way to the extreme, which takes exactly STROKE_SEC, so it stops there just as the
    setpoint flips to the other extreme.

    Returns:
        float: compression setpoint, relative to zeroed position, in meters. Down (towards the chest) is positive
    """
    global compression_start_time
    cycle_time: float = (time.monotonic() - compression_start_time) % (2.0 * STROKE_SEC)
    going_down: bool = cycle_time < STROKE_SEC
    return COMPRESSION_DEPTH_CM / 100.0 if going_down else 0.0


def computeCompressionSetpoint_streamed_deprecated() -> tuple[float, float]:
    """DEPRECATED: used by compressions_streamed_deprecated(). Computes the current position and velocity along the
    stroke profile, relative to zeroed position.

    Same motion the speed and acceleration caps give compressions(): each cycle is a down stroke of
    COMPRESSION_DEPTH_CM then an up stroke back to the zeroed position, with no hold at either end, at
    COMPRESSIONS_PER_MIN. Each stroke takes STROKE_SEC, a third each: speed up at COMPRESSION_ACCEL_MPS2, cruise at
    COMPRESSION_SPEED_MPS, slow down at COMPRESSION_ACCEL_MPS2 to a stop. Here every position and velocity along it is
    computed in Python instead.

    Returns:
        tuple[float, float]: position in meters and velocity in meters per second, relative to zeroed position.
            Down (towards the chest) is positive for both
    """
    global compression_start_time
    time_sec: float = time.monotonic() - compression_start_time
    cycle_time: float = time_sec % (2.0 * STROKE_SEC)
    going_down: bool = cycle_time < STROKE_SEC
    stroke_time: float = cycle_time if going_down else cycle_time - STROKE_SEC  # time into the current stroke

    # Each third covers a quarter, a half and a quarter of the stroke
    depth_m: float = COMPRESSION_DEPTH_CM / 100.0
    speed_up_end: float = STROKE_SEC / 3.0
    slow_down_start: float = 2.0 * STROKE_SEC / 3.0
    if stroke_time < speed_up_end:
        stroke_m: float = 0.5 * COMPRESSION_ACCEL_MPS2 * stroke_time ** 2
        speed_mps: float = COMPRESSION_ACCEL_MPS2 * stroke_time
    elif stroke_time < slow_down_start:
        stroke_m: float = 0.25 * depth_m + COMPRESSION_SPEED_MPS * (stroke_time - speed_up_end)
        speed_mps: float = COMPRESSION_SPEED_MPS
    else:
        stroke_m: float = depth_m - 0.5 * COMPRESSION_ACCEL_MPS2 * (STROKE_SEC - stroke_time) ** 2
        speed_mps: float = COMPRESSION_ACCEL_MPS2 * (STROKE_SEC - stroke_time)

    # The up stroke runs the same profile from the bottom back to the zeroed position, moving up (negative)
    if going_down:
        return stroke_m, speed_mps
    return depth_m - stroke_m, -speed_mps


def DEPRICATED_computeCompressionSetpoint() -> float:
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

def retract_carriage() -> tuple[ErrorCode, bool]:
    """Retract the carriage to its home (startup) position with the steppers, by retracting exactly as many steps as
    have been commanded in the extending direction (stepper.get_extended_distance()). Non-blocking, must be called
    every tick until the carriage is home. Like abort_compressions(), does not read sensors.
    Once home, the extending step count is reset so the next retraction only undoes extension commanded after this one.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_MOTOR_FAILURE if failed
        bool: True if the carriage is home, False otherwise
    """
    global retract_target_m

    # First tick of this retraction: stop where the already-commanded steps end, then set the target exactly the
    # commanded extension above it. Kept fixed until home, since the extending step count doesn't change while retracting
    if retract_target_m is None:
        stepper.halt()
        retract_target_m = stepper.get_setpoint() - stepper.get_extended_distance()
        logging.info(f"Retracting carriage {stepper.get_extended_distance()} m, from {stepper.get_setpoint()} m to {retract_target_m} m")

    error = stepper.set_target(retract_target_m, RETRACT_VELOCITY_MPS)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Carriage retract failed on stepper: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE, False

    if stepper.at_target():
        logging.info(f"Carriage retracted to {stepper.get_position()} m")
        stepper.reset_extended_steps()
        retract_target_m = None
        return ErrorCode.NORMAL_OPERATION, True
    return ErrorCode.NORMAL_OPERATION, False