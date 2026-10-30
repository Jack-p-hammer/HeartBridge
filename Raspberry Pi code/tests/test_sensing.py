"""Tests for sensing.py: sensor init, reading/validation, zeroing, and battery check.

Most tests here pass a lightweight _FakeMotorController directly into
init_sensing() (conftest.py) instead of going through actuation.init_motor() and
the real MoteusThread. sensing.py only ever calls get_rotary_position()/
get_battery_voltage() on whatever object it's given, so this is equivalent
from sensing.py's point of view, and avoids the real MoteusThread's
background command-loop thread (and the small sleep needed for its state to
catch up) entirely. Exactly one test (test_init_sensors_integrates_with_real_
motor_controller) exercises the real actuation.init_motor() -> MoteusThread
path end to end, to prove the wiring genuinely works.
"""
from typing import TYPE_CHECKING, cast

import pytest

from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from conftest import init_sensing

if TYPE_CHECKING:
    # Only needed for the cast() below -- importing the real module here would
    # pull in the real `moteus` package at test-collection time for no reason.
    from moteus_thread import MoteusThread


class _FakeMotorController:
    """Minimal stand-in for a MoteusThread, for tests that only need
    get_rotary_position()/get_battery_voltage() and don't care about the
    real motor command loop.
    """

    def __init__(self, position=0.0, voltage=24.0, raise_on_battery=None, raise_on_position=None):
        self.position = position
        self.voltage = voltage
        self.raise_on_battery = raise_on_battery
        self.raise_on_position = raise_on_position

    def get_rotary_position(self):
        if self.raise_on_position is not None:
            raise self.raise_on_position
        return self.position

    def get_battery_voltage(self):
        if self.raise_on_battery is not None:
            raise self.raise_on_battery
        return self.voltage


def _fake_controller(position=0.0, voltage=24.0, raise_on_battery=None) -> "MoteusThread":
    """Build a _FakeMotorController, typed as a MoteusThread for callers.

    sensing.init_sensors() is annotated to take a real MoteusThread; a duck-
    typed fake satisfies it at runtime but not for a nominal type checker
    like pyright, so this cast documents that the mismatch is intentional
    rather than sprinkling `# type: ignore` across every call site.
    """
    return cast("MoteusThread", _FakeMotorController(position, voltage, raise_on_battery))


def _raw_adc_bytes_for_force(force_newtons: float) -> bytes:
    """Compute the 2 raw ADC bytes read_force_sensor() would need to see to
    report the given force, inverting sensing.py's raw -> voltage -> force
    conversion (10-bit ADC, 5V reference, 1V = 100N placeholder scale).
    """
    raw = round(force_newtons / 100.0 / 5.0 * 1024)
    return bytes([(raw >> 8) & 0x03, raw & 0xFF])


# -------------------- init_sensors: failure paths --------------------

def test_init_sensors_fails_when_i2c_unavailable(hardware):
    hardware.i2c_connect_error = RuntimeError("i2c bus unavailable")
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.ERROR_INIT_FAILURE


def test_init_sensors_fails_when_tof_sensor_unavailable(hardware):
    hardware.tof_connect_error = RuntimeError("ToF sensor not responding")
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.ERROR_INIT_FAILURE


def test_init_sensors_fails_when_imu_unavailable(hardware):
    hardware.imu_connect_error = RuntimeError("IMU not responding")
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.ERROR_INIT_FAILURE


def test_init_sensors_fails_when_rotary_encoder_unreadable(hardware, caplog):
    """The rotary encoder read for its absolute zero raising is an init failure, not an exception."""
    import sensing
    controller = _FakeMotorController(raise_on_position=RuntimeError("comm lost"))

    assert init_sensing(sensing, cast("MoteusThread", controller), hardware) == ErrorCode.ERROR_INIT_FAILURE
    assert "Rotary encoder absolute zeroing failed" in caplog.text


def test_init_sensors_fails_when_tof_sensor_unreadable(hardware, caplog):
    """The ToF constructs fine but its absolute zero read raises: an init failure, not an exception."""
    hardware.tof.raise_on_read = RuntimeError("device not found")
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.ERROR_INIT_FAILURE
    assert "ToF absolute zeroing failed" in caplog.text


def test_init_sensors_fails_when_force_sensor_unreadable(hardware, caplog):
    """The force sensor zero read raising (ADC on the I2C bus) is an init failure, not an exception."""
    hardware.i2c.raise_on_read = TimeoutError("i2c bus timeout")
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.ERROR_INIT_FAILURE
    assert "Force sensor zeroing failed" in caplog.text


# -------------------- init_sensors: happy path --------------------

def test_init_sensors_captures_absolute_zero_positions(hardware):
    """Startup should snapshot the rotary/ToF/force readings as the absolute zero."""
    hardware.tof.range = 15
    hardware.i2c.adc_bytes = bytes([0, 0])  # 0V -> 0N
    import sensing

    error = init_sensing(sensing, _fake_controller(position=0.25), hardware)

    assert error == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_absolute_zero_position == 0.25
    assert sensing.ToF_absolute_zero_position == 15
    assert sensing.force_zero_value == pytest.approx(0.0)


def test_init_sensors_integrates_with_real_motor_controller(hardware):
    """End-to-end check through the real actuation.init_motor() -> MoteusThread path.

    MoteusThread updates its state from a background command-loop thread, so
    a short sleep is needed after configuring the fake low-level controller
    for that state to actually propagate before init_sensors() reads it.
    """
    import time
    import actuation
    import sensing

    actuation.init_motor()
    hardware.moteus_controller.position = 0.5
    time.sleep(0.05)

    error = init_sensing(sensing, actuation.get_motor_controller(), hardware)

    assert error == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_absolute_zero_position == pytest.approx(0.5)


# -------------------- individual sensor reads --------------------

def test_read_force_sensor_converts_adc_reading(hardware):
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(250.0)
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_force_sensor() == pytest.approx(250.0)


def test_read_tof_sensor_returns_configured_range(hardware):
    hardware.tof.range = 42
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_ToF_sensor() == 42


def test_read_imu_returns_configured_acceleration(hardware):
    hardware.imu.acceleration = (1.0, 2.0, 3.0)
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_IMU() == (1.0, 2.0, 3.0)


# -------------------- read_sensors: validation --------------------

def test_read_sensors_normal_operation_within_limits(hardware):
    """Default harness state (0 force, 0 position, 9.81 accel) is within every limit."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.NORMAL_OPERATION


def test_read_sensors_detects_zeroing_finished(hardware):
    """Force past the zeroing threshold (but under the zeroing sensor limit) should
    report ZEROING_FINISHED during a zeroing read."""
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)  # > 35N threshold, < 52.5N limit
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_sensors(ControlMode.ZEROING) == ErrorCode.ZEROING_FINISHED


def test_read_sensors_accel_within_tolerance_is_normal_operation(hardware):
    """IMU shift detection: an orientation within the tolerance band is not a fault."""
    hardware.imu.acceleration = (0.0, 0.0, 9.0)  # under the 9.81 compression limit
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.NORMAL_OPERATION


def test_read_sensors_detects_accel_over_limit(hardware):
    """IMU shift detection: an orientation beyond the tolerance band is a kneel
    failure specifically, not the generic ERROR_SENSOR_FAILURE (contrast with
    test_read_sensors_detects_position_disagreement below, a non-IMU failure)."""
    hardware.imu.acceleration = (0.0, 0.0, 20.0)  # exceeds the 9.81 compression limit
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_read_sensors_detects_position_disagreement(hardware):
    """Rotary and ToF should agree on how far each has moved since zeroing; a
    large mismatch between the two is a sensor failure."""
    import sensing
    controller = _FakeMotorController(position=0.0)
    init_sensing(sensing, cast("MoteusThread", controller), hardware)  # zeroes both sensors to their readings at init
    controller.position = 1.0  # rotary now reports 10mm of ball screw travel since zeroing
    # hardware.tof.range is left unchanged, so ToF reports no travel at all --
    # a 10mm disagreement, far past the 2mm threshold

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_rejects_invalid_control_mode(hardware):
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.read_sensors(ControlMode.HOLD_POSITION) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_i2c_read_failure_mid_operation(hardware):
    """A transient I2C error on a read (after a previously successful init) is
    reported as a sensor failure rather than raising out of read_sensors()."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)
    hardware.i2c.raise_on_read = TimeoutError("i2c bus timeout")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_tof_disconnect_mid_operation(hardware):
    """The ToF sensor failing after a previously successful init is a sensor failure."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)
    hardware.tof.raise_on_read = RuntimeError("device not found")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_rotary_encoder_failure_mid_operation(hardware, caplog):
    """The rotary encoder read raising after a previously successful init is a sensor failure."""
    import sensing
    controller = _FakeMotorController()
    init_sensing(sensing, cast("MoteusThread", controller), hardware)
    controller.raise_on_position = RuntimeError("comm lost")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE
    assert "Rotary encoder read failed" in caplog.text


def test_read_sensors_detects_imu_disconnect_mid_operation(hardware):
    """The IMU failing after a previously successful init is a sensor failure."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)
    hardware.imu.raise_on_read = RuntimeError("device not found")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


# -------------------- zero_position --------------------

def test_zero_position_captures_current_position_once_zeroing_finishes(hardware):
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)  # past the zeroing threshold
    import sensing
    init_sensing(sensing, _fake_controller(position=1.0), hardware)
    hardware.tof.range = 10  # 1.0 rotation is 10mm of ball screw travel

    assert sensing.zero_position() == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_zero_position == pytest.approx(1.0)
    assert sensing.ToF_zero_position == 10
    assert sensing.get_rotary_zero_position() == pytest.approx(1.0)


def test_zero_position_fails_when_zeroing_not_finished(hardware):
    """With force still under the zeroing threshold, zeroing hasn't finished yet."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)

    assert sensing.zero_position() == ErrorCode.ERROR_SENSOR_FAILURE


def test_zero_rotary_encoder_reports_read_failure(hardware, caplog):
    """zero_rotary_encoder() reports a read that raises as a sensor failure rather than raising."""
    import sensing
    controller = _FakeMotorController()
    init_sensing(sensing, cast("MoteusThread", controller), hardware)
    controller.raise_on_position = RuntimeError("comm lost")

    assert sensing.zero_rotary_encoder() == ErrorCode.ERROR_SENSOR_FAILURE
    assert "Failed to zero rotary encoder" in caplog.text


def test_zero_tof_sensor_reports_read_failure(hardware, caplog):
    """zero_ToF_sensor() reports a read that raises as a sensor failure rather than raising."""
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)
    hardware.tof.raise_on_read = RuntimeError("device not found")

    assert sensing.zero_ToF_sensor() == ErrorCode.ERROR_SENSOR_FAILURE
    assert "Failed to zero ToF sensor" in caplog.text


# -------------------- battery_check --------------------

def test_battery_check_normal_operation(hardware):
    import sensing
    init_sensing(sensing, _fake_controller(voltage=24.0), hardware)

    assert sensing.battery_check() == ErrorCode.NORMAL_OPERATION


def test_battery_check_reports_low_battery(hardware):
    import sensing
    init_sensing(sensing, _fake_controller(voltage=20.0), hardware)  # under the 21.6V threshold

    assert sensing.battery_check() == ErrorCode.ERROR_LOW_BATTERY


def test_battery_check_reports_motor_failure_when_read_fails(hardware):
    import sensing
    init_sensing(sensing, _fake_controller(raise_on_battery=RuntimeError("comm lost")), hardware)

    assert sensing.battery_check() == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- check_lead_screw_position --------------------

def test_check_lead_screw_position_within_tolerance(hardware):
    import sensing

    assert sensing.check_lead_screw_position(0.010, 0.010, (0.0105, 0.0102)) == ErrorCode.NORMAL_OPERATION


def test_check_lead_screw_position_accepts_anywhere_between_position_and_setpoint(hardware):
    """While steps are playing out, the encoders can be anywhere between the position and setpoint, in either direction."""
    import sensing

    assert sensing.check_lead_screw_position(0.010, 0.020, (0.015, 0.015)) == ErrorCode.NORMAL_OPERATION
    assert sensing.check_lead_screw_position(0.020, 0.010, (0.015, 0.015)) == ErrorCode.NORMAL_OPERATION


def test_check_lead_screw_position_detects_missed_steps(hardware):
    """Both encoders lagging the position (stall/missed steps) is a failure, at rest or moving."""
    import sensing

    assert sensing.check_lead_screw_position(0.010, 0.010, (0.0, 0.0)) == ErrorCode.ERROR_MOTOR_FAILURE
    assert sensing.check_lead_screw_position(0.010, 0.050, (0.005, 0.005)) == ErrorCode.ERROR_MOTOR_FAILURE


def test_check_lead_screw_position_detects_screws_out_of_sync(hardware):
    """Each screw within tolerance of the command, but too far from each other (racking), is a failure."""
    import sensing
    offset = sensing.STEPPER_POSITION_TOLERANCE_M * 0.9

    assert sensing.check_lead_screw_position(0.010, 0.010, (0.010 - offset, 0.010 + offset)) == ErrorCode.ERROR_MOTOR_FAILURE


def test_check_lead_screw_position_reports_unusable_readings(hardware, caplog):
    """No encoder readings at all is a sensor failure, not an exception."""
    import sensing

    assert sensing.check_lead_screw_position(0.010, 0.010, ()) == ErrorCode.ERROR_SENSOR_FAILURE
    assert "Lead screw encoder spread could not be computed" in caplog.text
