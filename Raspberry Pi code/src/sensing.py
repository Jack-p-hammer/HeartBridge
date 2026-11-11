# sensing.py
"""
Library of functions for reading and validating sensor readings, including the force sensor, rotary encoder, time-of-flight sensor, and IMU. These functions are non-blocking.

All functions that read sensors return a value assuming a successful read. All reading validation functions return an error code indicating whether the readings are valid or not. Therefore, try/except blocks are used when calling the read functions, all of which is done through the read_sensors() function, which is called by the main loop.

Zeroing detects the chest from a spike in the BLDC's q-axis (torque-producing) current, the same process as
demo/current_zeroing/current_zeroing.py, not from the force sensor. The force sensor is still validated against its limits.

BEFORE AND DURING ZEROING, THE zero_position VARIABLES MUST EQUAL THE absolute_zero_position VARIABLES
"""
# External imports
import dataclasses
import time
import pigpio
import board
import busio
import logging
import adafruit_vl53l0x
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

# Internal imports
from Enums.error_codes import ErrorCode
from moteus_thread import MoteusThread, MotorState, BLDC_ROTATIONS_PER_M, ZEROING_MAX_FORCE_N
from Enums.control_modes import ControlMode

# Global variables for shared sensor instances
_pi: pigpio.pi
_vl53: adafruit_vl53l0x.VL53L0X
_bno: BNO08X_I2C
_i2c: busio.I2C
_motor_controller: MoteusThread

# Futureproofing for a possible MUX on our I2C ADC
# Low - Force Sensor
# High - Battery Voltage
ADC_MUX_SIG_PIN = 19

# I2C address for the analog-to-digital converter. TODO: Use the I2C_scanner.py script to verify this!
ADC_ADDR = 0x48   # A0 variant: device code 1001 + address bits 000
ADC_VDD: float = 5.0  # 5V reference voltage for the ADC, used to convert the raw ADC reading to a voltage value
ADC_BITS: int = 10 

# Battery voltage threshold for low battery detection. TODO: Calibrate this value based on actual battery performance.
BATTERY_THRESHOLD_V: float = 21.6  # 6S LiPo battery, 3.6V per cell minimum, 4.2V per cell maximum. 6S = 21.6V minimum, 25.2V maximum.

COMPRESSION_FORCE_THRESHOLD_N: float = 500.0  # Newtons

# Current spike zeroing, same values as demo/current_zeroing/current_zeroing.py. TODO: Tune on hardware
# Samples are taken once per main loop tick (main.LOOP_TICK_SEC), not every moteus reply like the demo, so the
# baseline averages fewer samples and the consecutive samples span more time
ZEROING_SETTLE_TIME_S: float = 0.5  # seconds after zeroing starts before the current is trusted (acceleration transient)
ZEROING_BASELINE_TIME_S: float = 0.5  # seconds of free-running current averaged into the baseline, after ZEROING_SETTLE_TIME_S
ZEROING_SPIKE_THRESHOLD_A: float = 1.0  # amps above the baseline |q_current| that counts as chest contact
ZEROING_SPIKE_CONSECUTIVE_SAMPLES: int = 3  # samples in a row above threshold before zeroing finishes, rejects single-sample noise

POSITION_DISAGREE_THRESHOLD_MM: int = 2 # Millimeters

# Global variables for sensor starting positions
rotary_absolute_zero_position: float = 0.0  # The absolute position of rotary encoder on startup
ToF_absolute_zero_position: int = 0  # The absolute position of the ToF sensor on startup

# Global variables for sensor zeroed positions
rotary_zero_position: float = 0.0  # The zeroed position of the rotary encoder after zeroing
ToF_zero_position: int = 0  # The zeroed position of the ToF sensor after zeroing
force_zero_value: float = 0.0 # Newtons, force sensor reading under no load

# Global variables for current spike zeroing, reset by init_zeroing()
zeroing_start_time: float = 0.0  # time.monotonic() when zeroing started
zeroing_last_sample_time: float = 0.0  # MotorState.timestamp of the last motor sample used, so each sample is only counted once
zeroing_baseline_sum_a: float = 0.0  # sum of the free-running |q_current| samples, in amps
zeroing_baseline_count: int = 0  # number of samples in zeroing_baseline_sum_a
zeroing_spike_count: int = 0  # consecutive samples above the spike threshold
zeroing_contact_found: bool = False  # True once the current spike has been found, checked by zero_position()

# Define setpoint struct
# Not including position setpoints because those are validated against e/o
@dataclasses.dataclass()
class SensorLimits:
    force: float  # Force sensor reading in Newtons
    accel: tuple[float, float, float]  # IMU accelerometer readings in m/s^2 (x, y, z)

# Define setpoints for zeroing and compressions
zeroing_limits = SensorLimits(
    # The zeroing torque cap limits the plunger to ZEROING_MAX_FORCE_N, so a reading well above that is a sensor failure
    force = ZEROING_MAX_FORCE_N * 1.5, # TODO: Refine Multiplier
    accel = (0.0, 0.0, 9.81)  # TODO: Determine method to set this based on IMU orientation. Assuming the IMU is oriented such that gravity is along the z-axis
)

compression_limits = SensorLimits(
    force = COMPRESSION_FORCE_THRESHOLD_N,
    accel = (0.0, 0.0, 9.81) # TODO: Determine method to set this based on IMU orientation. Assuming the IMU is oriented such that gravity is along the z-axis
)


def init_sensors(motor_controller: MoteusThread, pi: pigpio.pi) -> ErrorCode:
    """Initialize the shared GPIO and sensor hardware used by the system."""
    global _pi, _vl53, _bno, _i2c
    global rotary_absolute_zero_position, ToF_absolute_zero_position, force_zero_value
    global rotary_zero_position, ToF_zero_position
    global _motor_controller, _pi
    
    logging.debug("Initializing sensors...")
    
    # Pull the shared moteus controller instance from actuation.py
    _motor_controller = motor_controller
    _pi = pi

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

    # VL53L0X time-of-flight sensor
    try:
        _vl53 = adafruit_vl53l0x.VL53L0X(_i2c)
    except Exception as e:
        logging.error(f"VL53L0X initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # BNO085 IMU TODO: Check if this is the one we have
    try:
        _bno = BNO08X_I2C(_i2c)
        # Enable the accelerometer report used for kneel detection
        _bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    except Exception as e:
        logging.error(f"BNO085 initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE
    
    # Zero the position sensors
    try:
        rotary_zero_position = read_rotary_encoder()
        rotary_absolute_zero_position = rotary_zero_position
    except Exception as e:
        logging.error(f"Rotary encoder absolute zeroing failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE
    
    try:
        ToF_zero_position = read_ToF_sensor()
        ToF_absolute_zero_position = ToF_zero_position
    except Exception as e:
        logging.error(f"ToF absolute zeroing failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE
    
    # Zero the force sensor
    try:
        force_zero_value = read_force_sensor()
    except Exception as e:
        logging.error(f"Force sensor zeroing failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE
    
    # TODO: Zero the IMU?

    logging.info("Sensors initialized")
    return ErrorCode.NORMAL_OPERATION


def read_sensors(control_mode: ControlMode) -> ErrorCode:
    """Read and validate sensor readings for control mode-appropriate setpoints

    Returns:
        ErrorCode: Normal if readings are valid, otherwise ERROR_SENSOR_FAILURE
    """
    global zeroing_limits, compression_limits
    sensor_limits: SensorLimits
    match control_mode:
        case ControlMode.ZEROING:
            sensor_limits = zeroing_limits
        case ControlMode.COMPRESSIONS:
            sensor_limits = compression_limits
        case ControlMode.PAUSE_RETRACT:
            sensor_limits = compression_limits
        case _:
            logging.error(f"Invalid control mode for sensor reading: {control_mode}")
            return ErrorCode.ERROR_SENSOR_FAILURE
        
    # TODO: Decide whether to store most recent sensor readings in a global variable for use by other functions, or just poll the sensors when needed. For now, just read and validate the sensors.
    try:
        current_force: float = read_force_sensor()
    except Exception as e:
        logging.error(f"Force sensor read failed: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    try:
        current_rotary: float = read_rotary_encoder()
    except Exception as e:
        logging.error(f"Rotary encoder read failed: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    try:
        current_ToF: int = read_ToF_sensor()
    except Exception as e:
        logging.error(f"ToF sensor read failed: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    try:
        current_accel: tuple[float, float, float] | None = read_IMU()
        if current_accel is None:
            logging.error("IMU read returned None")
            return ErrorCode.ERROR_SENSOR_FAILURE
    except Exception as e:
        logging.error(f"IMU read failed: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    # Defensive programming: default to error, but if all readings are valid return normal operation
    validation_error: ErrorCode = ErrorCode.ERROR_SENSOR_FAILURE
    current_readings: tuple = (current_force, current_rotary, current_ToF, current_accel)
    
    # Check readings against limits for the current control mode
    check_result: ErrorCode = check_sensor_error(sensor_limits, current_readings)
    if(check_result == ErrorCode.NORMAL_OPERATION):
        validation_error = ErrorCode.NORMAL_OPERATION
    elif(check_result == ErrorCode.ERROR_IMU_KNEEL_FAILURE):
        # Pass through IMU kneel failure error code
        validation_error = ErrorCode.ERROR_IMU_KNEEL_FAILURE
        
    # Zeroing Case: Check the motor current for chest contact, only once the other readings are known to be valid
    if (control_mode == ControlMode.ZEROING and
        validation_error == ErrorCode.NORMAL_OPERATION
    ):
        return check_zeroing_current()

    return validation_error

def init_zeroing() -> ErrorCode:
    """Reset the current spike detection for a new zeroing run. Non-blocking, call when zeroing starts.

    Returns:
        ErrorCode: Normal operation
    """
    global zeroing_start_time, zeroing_last_sample_time, zeroing_baseline_sum_a, zeroing_baseline_count
    global zeroing_spike_count, zeroing_contact_found

    zeroing_start_time = time.monotonic()
    zeroing_last_sample_time = 0.0  # Samples from before zeroing_start_time fall in the settle time and are ignored
    zeroing_baseline_sum_a = 0.0
    zeroing_baseline_count = 0
    zeroing_spike_count = 0
    zeroing_contact_found = False
    return ErrorCode.NORMAL_OPERATION

def zero_position() -> ErrorCode:
    """Set zeroed positions for the rotary encoder and ToF sensor. This should be called after zeroing is complete, and the plunger is at the zeroed position.

    Returns:
        ErrorCode: Error code describing the success or failure of the operation. Normal if successful, ERROR_SENSOR_FAILURE if failed.
    """
    global rotary_zero_position, ToF_zero_position, zeroing_contact_found

    # The spike that finished zeroing has already been counted, so check the result instead of reading the current again
    if not zeroing_contact_found:
        logging.error("Zeroing not finished, cannot set zeroed positions")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    if(zero_rotary_encoder() != ErrorCode.NORMAL_OPERATION or 
       zero_ToF_sensor() != ErrorCode.NORMAL_OPERATION):
        logging.error("Failed to set zeroed positions for sensors")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    logging.info(f"Zeroed positions set: Rotary {rotary_zero_position}, ToF {ToF_zero_position}")
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
    """Ensure sufficient charge for operation. 
    TODO: Split into startup battery check and continuous monitoring. Higher threshold on startup to prevent mid-operation shutdown, but still check battery voltage during operation.

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

# -------- PRIVATE FUNCTIONS --------

def check_zeroing_current() -> ErrorCode:
    """Check the newest motor sample for a q current spike above the free-running baseline, meaning the plunger has
    reached the chest. Same process as demo/current_zeroing/current_zeroing.py update_descent():
    1. Ignore the current for ZEROING_SETTLE_TIME_S while the motor accelerates
    2. Average the current for ZEROING_BASELINE_TIME_S to get the free-running baseline
    3. Report contact once |q_current| exceeds the baseline by ZEROING_SPIKE_THRESHOLD_A for
       ZEROING_SPIKE_CONSECUTIVE_SAMPLES samples in a row

    Returns:
        ErrorCode: ZEROING_FINISHED on a current spike, NORMAL_OPERATION otherwise, ERROR_SENSOR_FAILURE if the motor
                   state can't be read, ERROR_ZEROING_FAILURE if no baseline samples were collected
    """
    global _motor_controller, zeroing_start_time, zeroing_last_sample_time, zeroing_baseline_sum_a
    global zeroing_baseline_count, zeroing_spike_count, zeroing_contact_found

    try:
        state: MotorState = _motor_controller.get_state()
    except Exception as e:
        logging.error(f"Motor state read failed: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE

    # The moteus thread only updates at COMMAND_RATE_HZ, skip samples that were already counted (or no reply yet)
    if state.stale or state.timestamp == zeroing_last_sample_time:
        return ErrorCode.NORMAL_OPERATION
    zeroing_last_sample_time = state.timestamp

    sample_time_s: float = state.timestamp - zeroing_start_time
    current_a: float = abs(state.q_current)  # sign depends on motor direction, only the magnitude matters

    # Acceleration transient, current isn't meaningful yet
    if sample_time_s < ZEROING_SETTLE_TIME_S:
        return ErrorCode.NORMAL_OPERATION

    # Average the free-running current into the baseline
    if sample_time_s < ZEROING_SETTLE_TIME_S + ZEROING_BASELINE_TIME_S:
        zeroing_baseline_sum_a += current_a
        zeroing_baseline_count += 1
        return ErrorCode.NORMAL_OPERATION

    if zeroing_baseline_count == 0:
        logging.error("No samples collected for the zeroing current baseline")
        return ErrorCode.ERROR_ZEROING_FAILURE

    baseline_a: float = zeroing_baseline_sum_a / zeroing_baseline_count
    if current_a - baseline_a > ZEROING_SPIKE_THRESHOLD_A:
        zeroing_spike_count += 1
    else:
        zeroing_spike_count = 0

    if zeroing_spike_count >= ZEROING_SPIKE_CONSECUTIVE_SAMPLES:
        logging.info(f"Zeroing complete: Current {current_a} A exceeded baseline {baseline_a} A by more than {ZEROING_SPIKE_THRESHOLD_A} A")
        zeroing_contact_found = True
        return ErrorCode.ZEROING_FINISHED

    return ErrorCode.NORMAL_OPERATION


def check_sensor_error(sensor_limits: SensorLimits, sensor_readings: tuple) -> ErrorCode:
    """Determines if any sensor readings are out of intended range. 
    NOTE THAT THIS ASSUMES zero_position == absolute_zero_position FOR BOTH POS SENSORS BEFORE ZEROING IS COMPLETED

    Args:
        sensor_limits (SensorLimits): Sensor limits struct for the current operation 
        sensor_readings (tuple): Tuple containing the current readings from all sensors of format (force, rotary, ToF, accel)

    Returns:
        ErrorCode: Normal if readings are valid, ERROR_KNEEL_FAILURE if IMU error, ERROR_SENSOR_FAILURE otherwise
    """
    global rotary_zero_position, ToF_zero_position
    
    # Validation Method ---------------------------------------------------------------
    # If force or IMU readings exceed sensor limits, sensor failure
    # If ToF and rotary encoder disagree by POSITION_DISAGREE_THRESHOLD, sensor failure
    # ---------------------------------------------------------------------------------
    
    
    # Validate force, IMU
    if sensor_readings[0] > sensor_limits.force:
        logging.error(
            f"Force sensor reading out of range: Read {sensor_readings[0]}, Limit {sensor_limits.force}"
        )
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    # Loop over accel tuple to validate each
    for accel_tuple_index in range(3):
        # TODO: Check is abs is correct for all 3 cases
        if abs(sensor_readings[3][accel_tuple_index]) > abs(sensor_limits.accel[accel_tuple_index]): 
            logging.error(
                f"IMU reading out of range: Read {sensor_readings[3][accel_tuple_index]}, Limit {sensor_limits.accel[accel_tuple_index]} for direction {accel_tuple_index}"
            )
            return ErrorCode.ERROR_IMU_KNEEL_FAILURE
        
    # Validate position sensors
    # TODO: This is placeholder logic
    rotary_pos_m: float = (sensor_readings[1] - rotary_zero_position) / BLDC_ROTATIONS_PER_M # Plunger travel along the ball screw
    rotary_pos_mm: float = rotary_pos_m*1000
    ToF_pos_mm: int = sensor_readings[2] - ToF_zero_position
    
    if(abs(rotary_pos_mm-ToF_pos_mm) > POSITION_DISAGREE_THRESHOLD_MM):
        logging.error(f"Position sensor disagreement: Rotary {rotary_pos_mm}, ToF {ToF_pos_mm}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    
    return ErrorCode.NORMAL_OPERATION


def read_force_sensor() -> float:
    """Read the force sensor and return the value in Newtons

    Returns:
        float: Force sensor reading in Newtons
    """
    raw_reading: int = read_ADC()
    raw_voltage: float = (raw_reading / (2**ADC_BITS)) * ADC_VDD

    # TODO: Calibrate force sensor
    # Placeholder conversion: 1V = 100N
    force_newtons: float = raw_voltage * 100.0

    return force_newtons


def read_ADC() -> int:
    """Read the analog-to-digital converter (ADC) and return the raw value

    The chip sends 2 bytes:
      Byte 1: 0 0 0 0 0 0 D9 D8   (upper 6 bits are don't-care/zero)
      Byte 2: D7 D6 D5 D4 D3 D2 D1 D0

    So the 10-bit value is ((byte1 & 0x03) << 8) | byte2

    Returns:
        int: ADC reading in raw counts (0-1023)
    """
    global _pi, _i2c

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
    global _vl53
    return _vl53.range


def read_IMU() -> tuple[float, float, float] | None:
    """Read the IMU and return the values

    Returns:
        tuple [float, float, float]: IMU accelerometer readings in m/s^2 (x, y, z) or None
    """
    global _bno
    accel_reading: tuple[float, float, float] | None = _bno.acceleration
    return accel_reading


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
  
    
def zero_ToF_sensor() -> ErrorCode:
    """Zero the time-of-flight sensor by setting the current position as zero.

    Returns:
        ErrorCode: Normal if successful, ERROR_SENSOR_FAILURE if failed
    """
    global ToF_zero_position
    try:
        ToF_zero_position = read_ToF_sensor()
        return ErrorCode.NORMAL_OPERATION
    except Exception as e:
        logging.error(f"Failed to zero ToF sensor: {e}")
        return ErrorCode.ERROR_SENSOR_FAILURE
    

def read_rotary_encoder() -> float:
    """Read the rotary encoder from the moteus-x1 and return the value in rotations

    Returns:
        float: Rotary position in rotations
    """
    global _motor_controller
    return _motor_controller.get_rotary_position()