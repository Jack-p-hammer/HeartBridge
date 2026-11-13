# actuation.py (demo)
"""
Demo stand-in for src/actuation.py. demo/main.py puts this folder ahead of src/ on the import path, so
`import actuation` finds this file instead of the real one.

Two changes from src/actuation.py:
- zeroing() drives the carriage straight to the hardcoded DEMO_ZERO_POSITION_M, instead of lowering until the force
  sensor detects the chest
- lower_carriage() is new: it lowers the carriage STARTUP_LOWER_M during the STARTUP state
Every other function is src/actuation.py's, unaltered, and re-exported below, so the shared motor controller and the
compression and retraction state all live in src/actuation.py.

NOTE: ALL SETPOINTS ARE IN METERS. DO NOT CONVERT TO ROTATIONS OR STEPS, THAT IS ALL DONE WITHIN moteus_thread.py AND stepper.py
"""

# External imports
import importlib.util
import logging
import sys
import time
from pathlib import Path

# Internal imports
import sensing  # demo/sensing.py
import stepper
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from stepper import ZEROING_VELOCITY_MPS

# Load src/actuation.py under its own name. A plain `import actuation` here would find this file again, since demo/
# is first on the import path
_SRC_ACTUATION_PATH = Path(__file__).resolve().parent.parent / "src" / "actuation.py"
_spec = importlib.util.spec_from_file_location("src_actuation", _SRC_ACTUATION_PATH)
assert _spec is not None and _spec.loader is not None, f"Could not load {_SRC_ACTUATION_PATH}"
_src = importlib.util.module_from_spec(_spec)
sys.modules["src_actuation"] = _src
_spec.loader.exec_module(_src)

# Unaltered from src/actuation.py. These share src/actuation.py's module state (motor controller, compression start
# time, retraction target), so main.py calls them through this module as usual
COMPRESSION_DEPTH_CM = _src.COMPRESSION_DEPTH_CM
init_motor = _src.init_motor
init_steppers = _src.init_steppers
get_motor_controller = _src.get_motor_controller
init_compressions = _src.init_compressions
compressions = _src.compressions
pause_compressions = _src.pause_compressions
abort_compressions = _src.abort_compressions
computeCompressionSetpoint = _src.computeCompressionSetpoint
retract_carriage = _src.retract_carriage
check_motor = _src.check_motor
reset_motor = _src.reset_motor

# Carriage position zeroing drives to, in meters from home (startup position), positive is down.
# Must be within stepper.LEAD_SCREW_TRAVEL_M, or stepper.set_target() rejects it and zeroing aborts.
# Must also be at or below STARTUP_LOWER_M, i.e. >= since down is positive (checked below)
DEMO_ZERO_POSITION_M: float = 0.08-(2*0.0254)  # 0.05 TODO: Placeholder, measure on the rig

# Carriage position STARTUP lowers to, in meters from home, positive is down. Assumes the carriage starts fully
# retracted, since stepper positions are measured from wherever it was at startup
STARTUP_LOWER_M: float = .01 #0.0254*0.25  # 0.25 inch

# retract_carriage() (src/actuation.py) only undoes downward steps, so the carriage must never move up except while
# retracting. Zeroing would move it up from where STARTUP leaves it, and RESET_DEMO/ABORT could then never get it home
if DEMO_ZERO_POSITION_M < STARTUP_LOWER_M:
    raise ValueError(f"DEMO_ZERO_POSITION_M ({DEMO_ZERO_POSITION_M} m) must be at or below (>=) "
                     f"STARTUP_LOWER_M ({STARTUP_LOWER_M} m)")

# Same as src/actuation.py (time to reach max extension, plus a buffer). Far longer than a demo zeroing takes, so it
# only trips if the carriage stalls
ZEROING_TIMEOUT_SEC: float = _src.ZEROING_TIMEOUT_SEC

zeroing_start_time: float = 0.0  # time.monotonic() of when zeroing started


def init_zeroing() -> ErrorCode:
    """Perform pre-zeroing setup. Non-blocking.

    Returns:
        ErrorCode: Normal operation
    """
    global zeroing_start_time

    zeroing_start_time = time.monotonic()
    logging.debug(f"Zeroing initialized (demo: hardcoded at {DEMO_ZERO_POSITION_M} m)")

    return ErrorCode.NORMAL_OPERATION


def zeroing() -> ErrorCode:
    """
    Perform the hardcoded zeroing procedure. Non-blocking.
    The procedure is as follows:
    1. Hold the plunger just below the top of its ball screw travel
    2. Command the steppers to DEMO_ZERO_POSITION_M at zeroing velocity
    3. Report ZEROING_FINISHED once every step to it has played. The carriage holds there for the rest of compressions

    Returns:
        ErrorCode: Normal while zeroing, ZEROING_FINISHED when finished, ERROR_ZEROING_FAILURE on timeout,
            ERROR_MOTOR_FAILURE on a motor or stepper failure (including DEMO_ZERO_POSITION_M outside the lead screw travel)
    """
    global zeroing_start_time

    # Check for timeout
    if time.monotonic() - zeroing_start_time > ZEROING_TIMEOUT_SEC:
        logging.error("Zeroing failed: timeout")
        return ErrorCode.ERROR_ZEROING_FAILURE

    # Check for motor errors
    motor_controller = get_motor_controller()
    error = motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed: motor error {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Hold the plunger in place on the ball screw, can be done repeatedly since it is non-blocking
    error = motor_controller.set_target(ControlMode.ZEROING,
                                        sensing.get_rotary_zero_position(),
                                        sensing.get_rotary_absolute_zero_position())
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed on motor command: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Drive the carriage to the hardcoded zero, can be done repeatedly since it is non-blocking
    error = stepper.set_target(DEMO_ZERO_POSITION_M, ZEROING_VELOCITY_MPS)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Zeroing failed on stepper: {error}")
        stepper.halt()
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Done once every step to the target has played. Checked in steps with at_target(), not by comparing
    # get_position() to DEMO_ZERO_POSITION_M: the target rounds to a whole step, so the position can read just under it
    if stepper.at_target():
        logging.info(f"Carriage zeroed at {stepper.get_position()} m (demo: hardcoded at {DEMO_ZERO_POSITION_M} m)")
        return ErrorCode.ZEROING_FINISHED
    return ErrorCode.NORMAL_OPERATION


def lower_carriage() -> tuple[ErrorCode, bool]:
    """Lower the carriage to STARTUP_LOWER_M at zeroing velocity, for the STARTUP state. Non-blocking, must be called
    every tick until it reports done, since the steppers only move while being updated. The plunger isn't commanded.
    Like retract_carriage(), does not read sensors.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_MOTOR_FAILURE if failed (including STARTUP_LOWER_M outside
            the lead screw travel)
        bool: True once the carriage is at STARTUP_LOWER_M, False otherwise
    """
    error = stepper.set_target(STARTUP_LOWER_M, ZEROING_VELOCITY_MPS)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Lowering carriage failed on stepper: {error}")
        stepper.halt()
        return ErrorCode.ERROR_MOTOR_FAILURE, False

    # Checked in steps with at_target(), for the same reason as in zeroing()
    return ErrorCode.NORMAL_OPERATION, stepper.at_target()
