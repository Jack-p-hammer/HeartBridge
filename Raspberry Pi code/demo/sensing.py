# sensing.py (demo)
"""
Demo stand-in for src/sensing.py. demo/main.py puts this folder ahead of src/ on the import path, so src/main.py and
src/actuation.py import this module instead of the real one. Everything else runs from src/ unaltered.

Same public functions as src/sensing.py, but none of the sensor suite is used: no force sensor/ADC, ToF sensor, IMU,
I2C bus, or sensor validation. Readings that come from the moteus itself (rotary position, battery voltage) are kept,
since actuation.py needs the rotary zero positions to command the BLDC.

Zeroing finishes when the carriage reaches DEMO_ZERO_POSITION_M instead of when the force sensor detects the chest.

The states the sensor suite would trigger are reached with simulate_fault(), which demo/main.py calls from the keyboard:
ERROR_IMU_KNEEL_FAILURE goes to KNEEL_FAILURE and ERROR_SENSOR_FAILURE goes to ABORT, same as in src/main.py.

BEFORE AND DURING ZEROING, THE zero_position VARIABLES MUST EQUAL THE absolute_zero_position VARIABLES
"""
# External imports
import pigpio
import logging

# Internal imports (from src/)
from Enums.error_codes import ErrorCode
from moteus_thread import MoteusThread
from Enums.control_modes import ControlMode
import stepper

# Global variables for shared instances
_pi: pigpio.pi
_motor_controller: MoteusThread

# Carriage position that counts as reaching the chest, in meters from home. Must be below stepper.LEAD_SCREW_TRAVEL_M
# The carriage stops up to CHUNK_DURATION_S * CHUNK_QUEUE_DEPTH of zeroing travel past this (see stepper.halt())
DEMO_ZERO_POSITION_M: float = 0.05  # TODO: Placeholder

# Battery voltage threshold for low battery detection. TODO: Calibrate this value based on actual battery performance.
BATTERY_THRESHOLD_V: float = 21.6  # 6S LiPo battery, 3.6V per cell minimum, 4.2V per cell maximum. 6S = 21.6V minimum, 25.2V maximum.

# Global variables for sensor starting positions
rotary_absolute_zero_position: float = 0.0  # The absolute position of rotary encoder on startup

# Global variables for sensor zeroed positions
rotary_zero_position: float = 0.0  # The zeroed position of the rotary encoder after zeroing

# Fault for the next read_sensors() to report once, see simulate_fault(). None for no fault
_simulated_fault: ErrorCode | None = None


def init_sensors(motor_controller: MoteusThread, pi: pigpio.pi) -> ErrorCode:
    """Store the shared instances and record the rotary encoder's startup position. No sensor hardware is touched."""
    global _motor_controller, _pi
    global rotary_absolute_zero_position, rotary_zero_position

    logging.debug("Initializing sensors (demo: sensing disabled)...")

    # Pull the shared moteus controller instance from actuation.py
    _motor_controller = motor_controller
    _pi = pi

    # Zero the rotary encoder
    try:
        rotary_zero_position = read_rotary_encoder()
        rotary_absolute_zero_position = rotary_zero_position
    except Exception as e:
        logging.error(f"Rotary encoder absolute zeroing failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    logging.info(f"Sensors initialized (demo: zeroing stops at {DEMO_ZERO_POSITION_M} m)")
    return ErrorCode.NORMAL_OPERATION


def simulate_fault(error: ErrorCode) -> None:
    """Make the next read_sensors() report error once, as if a sensor had tripped. Sensors are only read while
    zeroing, compressing and pausing, so the fault waits for the next of those.

    Args:
        error (ErrorCode): ERROR_IMU_KNEEL_FAILURE for KNEEL_FAILURE, ERROR_SENSOR_FAILURE for ABORT
    """
    global _simulated_fault
    logging.warning(f"Demo: {error.name} queued for the next sensor read")
    _simulated_fault = error


def read_sensors(control_mode: ControlMode) -> ErrorCode:
    """No sensors are read. While zeroing, reports ZEROING_FINISHED once the carriage reaches DEMO_ZERO_POSITION_M

    Returns:
        ErrorCode: A fault queued by simulate_fault(), else ZEROING_FINISHED as above, NORMAL_OPERATION otherwise,
            ERROR_SENSOR_FAILURE on an invalid control mode
    """
    global _simulated_fault
    match control_mode:
        case ControlMode.ZEROING | ControlMode.COMPRESSIONS | ControlMode.PAUSE_RETRACT:
            pass
        case _:
            logging.error(f"Invalid control mode for sensor reading: {control_mode}")
            return ErrorCode.ERROR_SENSOR_FAILURE

    if _simulated_fault is not None:
        error: ErrorCode = _simulated_fault
        _simulated_fault = None
        logging.warning(f"Demo: reporting simulated {error.name}")
        return error

    carriage_position_m: float = stepper.get_position()
    if (control_mode == ControlMode.ZEROING and
        carriage_position_m >= DEMO_ZERO_POSITION_M
    ):
        logging.info(f"Zeroing complete: Carriage {carriage_position_m} m reached demo zero position {DEMO_ZERO_POSITION_M} m")
        return ErrorCode.ZEROING_FINISHED

    return ErrorCode.NORMAL_OPERATION

def zero_position() -> ErrorCode:
    """Set the zeroed position for the rotary encoder. This should be called after zeroing is complete, and the plunger is at the zeroed position.

    Returns:
        ErrorCode: Error code describing the success or failure of the operation. Normal if successful, ERROR_SENSOR_FAILURE if failed.
    """
    global rotary_zero_position

    error = read_sensors(ControlMode.ZEROING)
    if error != ErrorCode.ZEROING_FINISHED:
        logging.error(f"Zeroing not finished, cannot set zeroed positions: {error}")
        return ErrorCode.ERROR_SENSOR_FAILURE

    if zero_rotary_encoder() != ErrorCode.NORMAL_OPERATION:
        logging.error("Failed to set zeroed positions for sensors")
        return ErrorCode.ERROR_SENSOR_FAILURE

    logging.info(f"Zeroed positions set: Rotary {rotary_zero_position}")
    return ErrorCode.NORMAL_OPERATION

def get_rotary_zero_position() -> float:
    """Get the zeroed position of the rotary encoder in rotations.

    Returns:
        float: Zeroed position of the rotary encoder in rotations.
    """
    global rotary_zero_position
    return rotary_zero_position

def get_rotary_absolute_zero_position() -> float:
    """Get the absolute zeroed position of the rotary encoder in rotations.

    Returns:
        float: Absolute zeroed position of the rotary encoder in rotations.
    """
    global rotary_absolute_zero_position
    return rotary_absolute_zero_position


def battery_check() -> ErrorCode:
    """Ensure sufficient charge for operation. Reads the moteus, not the sensor suite, so this is the same as src/sensing.py

    Returns:
        ErrorCode: Normal if battery is sufficient, otherwise ERROR_LOW_BATTERY
    """
    global _motor_controller
    logging.debug("Checking battery voltage...")

    try:
        battery_voltage: float = _motor_controller.get_battery_voltage()
    except Exception as e:
        logging.error(f"Battery voltage read failed: {e}")
        # Motor failure because moteus let us down
        return ErrorCode.ERROR_MOTOR_FAILURE

    if battery_voltage < BATTERY_THRESHOLD_V:
        logging.error(f"Low battery: {battery_voltage:.2f} V, threshold {BATTERY_THRESHOLD_V} V")
        return ErrorCode.ERROR_LOW_BATTERY
    logging.debug(f"Battery voltage {battery_voltage:.2f} V")
    return ErrorCode.NORMAL_OPERATION


def check_lead_screw_position(position_m: float, setpoint_m: float, encoder_positions_m: tuple[float, ...]) -> ErrorCode:
    """No lead screw encoders are checked in the demo.

    Returns:
        ErrorCode: Always NORMAL_OPERATION
    """
    return ErrorCode.NORMAL_OPERATION

# -------- PRIVATE FUNCTIONS --------

def zero_rotary_encoder() -> ErrorCode:
    """Zero the rotary encoder by setting the current position as zero.

    Returns:
        ErrorCode: Normal if successful, ERROR_SENSOR_FAILURE if failed
    """
    global rotary_zero_position
    try:
        rotary_zero_position = read_rotary_encoder()
        return ErrorCode.NORMAL_OPERATION
    except Exception as e:
        logging.error(f"Failed to zero rotary encoder: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE


def read_rotary_encoder() -> float:
    """Read the rotary encoder from the moteus-x1 and return the value in rotations

    Returns:
        float: Rotary position in rotations
    """
    global _motor_controller
    return _motor_controller.get_rotary_position()
