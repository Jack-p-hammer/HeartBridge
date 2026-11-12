# sensing.py (demo)
"""
Demo stand-in for src/sensing.py. demo/main.py puts this folder ahead of src/ on the import path, so src/main.py and
demo/actuation.py and src/actuation.py import this module instead of the real one.

Same public functions as src/sensing.py, but none of the sensor suite is used: no force sensor/ADC, ToF sensor, IMU,
I2C bus, or sensor validation. Readings that come from the moteus itself (rotary position, battery voltage) are kept,
since actuation.py needs the rotary zero positions to command the BLDC.

Zeroing doesn't wait on any sensor: demo/actuation.py drives the carriage to a hardcoded position, so read_sensors()
never reports ZEROING_FINISHED.

BEFORE AND DURING ZEROING, THE zero_position VARIABLES MUST EQUAL THE absolute_zero_position VARIABLES
"""
# External imports
import pigpio
import logging

# Internal imports (from src/)
from Enums.error_codes import ErrorCode
from moteus_thread import MoteusThread
from Enums.control_modes import ControlMode

# Global variables for shared instances
_pi: pigpio.pi
_motor_controller: MoteusThread

# Battery voltage threshold for low battery detection. TODO: Calibrate this value based on actual battery performance.
BATTERY_THRESHOLD_V: float = 21.6  # 6S LiPo battery, 3.6V per cell minimum, 4.2V per cell maximum. 6S = 21.6V minimum, 25.2V maximum.

# Global variables for sensor starting positions
rotary_absolute_zero_position: float = 0.0  # The absolute position of rotary encoder on startup

# Global variables for sensor zeroed positions
rotary_zero_position: float = 0.0  # The zeroed position of the rotary encoder after zeroing


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

    logging.info("Sensors initialized (demo: rotary encoder only)")
    return ErrorCode.NORMAL_OPERATION


def read_sensors(control_mode: ControlMode) -> ErrorCode:
    """No sensors are read. Zeroing finishes in demo/actuation.py once the carriage reaches its hardcoded position.

    Returns:
        ErrorCode: NORMAL_OPERATION, or ERROR_SENSOR_FAILURE on an invalid control mode
    """
    match control_mode:
        case ControlMode.ZEROING | ControlMode.COMPRESSIONS | ControlMode.PAUSE_RETRACT:
            pass
        case _:
            logging.error(f"Invalid control mode for sensor reading: {control_mode}")
            return ErrorCode.ERROR_SENSOR_FAILURE

    return ErrorCode.NORMAL_OPERATION

def zero_position() -> ErrorCode:
    """Set the zeroed position for the rotary encoder. This should be called after zeroing is complete, and the plunger is at the zeroed position.
    main.py only calls this once actuation.zeroing() reports ZEROING_FINISHED, so there is nothing left to check here.

    Returns:
        ErrorCode: Error code describing the success or failure of the operation. Normal if successful, ERROR_SENSOR_FAILURE if failed.
    """
    global rotary_zero_position

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
        return ErrorCode.ERROR_LOW_BATTERY
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
