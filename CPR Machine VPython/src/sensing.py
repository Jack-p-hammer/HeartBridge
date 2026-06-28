# sensing.py

import pigpio
import board
import busio
import logging
import adafruit_vl53l0x
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from error_codes import ErrorCode

# Global variables for shared sensor instances
_pi: pigpio.pi
_vl53: adafruit_vl53l0x.VL53L0X
_bno: BNO08X_I2C


def get_pi():
    """Returns the shared pigpio instance for use by hmi.py's button/LED/laser GPIO."""
    return _pi


def init_sensors() -> ErrorCode:
    """Initialize the shared GPIO and sensor hardware used by the system."""
    global _pi, _vl53, _bno

    # pigpio is only used here for the shared GPIO instance passed to hmi.py.
    # The sensors themselves use the Adafruit/Blinka I2C abstraction below.
    # The pigpio daemon must be running before this is called:
    #   sudo pigpiod
    _pi = pigpio.pi()
    if not _pi.connected:
        logging.error("Failed to connect to pigpio daemon")
        return ErrorCode.ERROR_INIT_FAILURE

    # board.SCL and board.SDA refer to the Pi's default I2C pins (GPIO 3 and 2).
    # Both sensors share this same I2C bus.
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        logging.error(f"Failed to initialize I2C bus: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # VL53L0X time-of-flight sensor
    try:
        _vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    except Exception as e:
        logging.error(f"VL53L0X initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # BNO085 IMU TODO: Check if this is the one we have
    try:
        _bno = BNO08X_I2C(i2c)
        # Enable the accelerometer report used for kneel detection
        _bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    except Exception as e:
        logging.error(f"BNO085 initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    logging.info("Sensors initialized")
    return ErrorCode.NORMAL_OPERATION


def battery_check() -> ErrorCode:
    """Ensure sufficient charge for operation

    Returns:
        ErrorCode: Normal if battery is sufficient, otherwise ERROR_LOW_BATTERY
    """
    # TODO: Implement actual battery level checking logic
    return ErrorCode.NORMAL_OPERATION


def read_sensors_fine_zeroing() -> ErrorCode:
    """Read and validate sensor readings for zeroing-appropriate setpoints

    Returns:
        ErrorCode: Normal if readings are valid, otherwise ERROR_SENSOR_FAILURE
    """
    return ErrorCode.NORMAL_OPERATION


def read_sensors_compressions() -> ErrorCode:
    """Read and validate sensor readings for zeroing-appropriate setpoints

    Returns:
        ErrorCode: Normal if readings are valid, otherwise ERROR_SENSOR_FAILURE
    """
    return ErrorCode.NORMAL_OPERATION


def check_sensor_error(sensor_setpoints: int) -> ErrorCode:
    """Determines if any sensor readings are out of intended range

    Args:
        sensor_setpoints (int): Sensor setpoints, in order [Rotary, ToF, IMU, Force]

    Returns:
        ErrorCode: Normal if readings are valid, ERROR_KNEEL_FAILURE if IMU error, ERROR_SENSOR_FAILURE otherwise
    """
    return ErrorCode.NORMAL_OPERATION
