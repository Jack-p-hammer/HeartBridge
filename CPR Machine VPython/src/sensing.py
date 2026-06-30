# sensing.py

import pigpio
import board
import busio
import logging
import adafruit_vl6180x
import adafruit_bno08x
from error_codes import ErrorCode
from adafruit_bno08x.i2c import BNO08X_I2C

# Global variables for shared sensor instances
_pi: pigpio.pi
_vl61: adafruit_vl6180x.VL6180X
_bno: BNO08X_I2C
_i2c: busio.I2C

# Futureproofing for a possible MUX on our I2C ADC
# Low - Force Sensor
# High - Battery Voltage
ADC_MUX_SIG_PIN = 19

# I2C address for the analog-to-digital converter. TODO: Use the I2C_scanner.py script to verify this!
ADC_ADDR = 0x48   # A0 variant: device code 1001 + address bits 000
ADC_VDD = 5  # 5V reference voltage for the ADC, used to convert the raw ADC reading to a voltage value


def get_pi():
    """Returns the shared pigpio instance for use by hmi.py's button/LED/laser GPIO."""
    return _pi


def init_sensors() -> ErrorCode:
    """Initialize the shared GPIO and sensor hardware used by the system."""
    global _pi, _vl61, _bno, _i2c

    # pigpio is only used here for the shared GPIO instance passed to hmi.py.
    # The sensors themselves use the Adafruit/Blinka I2C abstraction below.
    # The pigpio daemon must be running before this is called:
    #   sudo pigpiod
    _pi = pigpio.pi()
    if not _pi.connected:
        logging.error("Failed to connect to pigpio daemon")
        return ErrorCode.ERROR_INIT_FAILURE

    # Initialize MUX selector (does nothing for now)
    _pi.set_mode(ADC_MUX_SIG_PIN, pigpio.OUTPUT)
    _pi.set_pull_up_down(ADC_MUX_SIG_PIN, pigpio.PUD_DOWN)

    # board.SCL and board.SDA refer to the Pi's default I2C pins (GPIO 3 and 2).
    # Both sensors share this same I2C bus.
    try:
        _i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        logging.error(f"Failed to initialize I2C bus: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # VL6180X time-of-flight sensor
    try:
        _vl61 = adafruit_vl6180x.VL6180X(_i2c)
    except Exception as e:
        logging.error(f"VL6180X initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # BNO085 IMU TODO: Check if this is the one we have
    try:
        _bno = BNO08X_I2C(_i2c)
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


def read_sensors_zeroing() -> ErrorCode:
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


def read_force_sensor() -> float:
    """Read the force sensor and return the value in Newtons

    Returns:
        float: Force sensor reading in Newtons
    """
    raw_reading: int = read_ADC()
    raw_voltage: float = (raw_reading / 1023.0) * ADC_VDD

    # TODO: Calibrate force sensor
    # Placeholder conversion: 1V = 100N
    force_newtons: float = raw_voltage * 100.0

    return force_newtons


def read_ADC() -> int:
    """Read the analog-to-digital converter (ADC) and return the value in volts

    The chip sends 2 bytes:
      Byte 1: 0 0 0 0 0 0 D9 D8   (upper 6 bits are don't-care/zero)
      Byte 2: D7 D6 D5 D4 D3 D2 D1 D0

    So the 10-bit value is ((byte1 & 0x03) << 8) | byte2

    Returns:
        int: ADC reading in raw counts (0-1023)
    """

    result: bytearray = bytearray(2)

    # Select the ADC from MUX (currently does nothing)
    _pi.write(ADC_MUX_SIG_PIN, 0)

    # Read ADC
    _i2c.readfrom_into(ADC_ADDR, result)
    raw: int = ((result[0] & 0x03) << 8) | result[1]
    return raw


def read_ToF_sensor() -> int:
    """Read the time-of-flight sensor and return the value in millimeters

    Returns:
        int: ToF sensor reading in millimeters
    """
    return _vl61.range


def read_IMU() -> tuple[float, float, float] | None:
    """Read the IMU and return the values

    Returns:
        tuple [float, float, float]: IMU accelerometer readings in m/s^2 (x, y, z) or None
    """
    # TODO: Implement actual IMU reading logic
    accel_reading: tuple[float, float, float] | None = _bno.acceleration
    return accel_reading


def read_rotary_encoder() -> float:
    """Read the rotary encoder from the moteus-x1 and return the value in rotations

    Returns:
        float: Rotary position in rotations
    """
    # TODO: Implement actual rotary encoder reading logic
    return 0.0
