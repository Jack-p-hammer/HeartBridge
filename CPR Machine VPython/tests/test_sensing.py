"""Tests for sensing.py: sensor init, reading/validation, zeroing, and battery check.

Most tests here pass a lightweight _FakeMotorController directly into
sensing.init_sensors(, hardware.pi) instead of going through actuation.init_motor() and
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

if TYPE_CHECKING:
    # Only needed for the cast() below -- importing the real module here would
    # pull in the real `moteus` package at test-collection time for no reason.
    from moteus_thread import MoteusThread


class _FakeMotorController:
    """Minimal stand-in for a MoteusThread, for tests that only need
    get_rotary_position()/get_battery_voltage() and don't care about the
    real motor command loop.
    """

    def __init__(self, position=0.0, voltage=24.0, raise_on_battery=None):
        self.position = position
        self.voltage = voltage
        self.raise_on_battery = raise_on_battery

    def get_rotary_position(self):
        return self.position

    def get_battery_voltage(self):
        if self.raise_on_battery is not None:
            raise self.raise_on_battery
        return self.voltage


def _fake_controller(position=0.0, voltage=24.0, raise_on_battery=None) -> "MoteusThread":
    """Build a _FakeMotorController, typed as a MoteusThread for callers.

    sensing.init_sensors(, hardware.pi) is annotated to take a real MoteusThread; a duck-
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

    assert sensing.init_sensors(_fake_controller(), hardware.pi) == ErrorCode.ERROR_INIT_FAILURE


def test_init_sensors_fails_when_tof_sensor_unavailable(hardware):
    hardware.tof_connect_error = RuntimeError("ToF sensor not responding")
    import sensing

    assert sensing.init_sensors(_fake_controller(), hardware.pi) == ErrorCode.ERROR_INIT_FAILURE


def test_init_sensors_fails_when_imu_unavailable(hardware):
    hardware.imu_connect_error = RuntimeError("IMU not responding")
    import sensing

    assert sensing.init_sensors(_fake_controller(), hardware.pi) == ErrorCode.ERROR_INIT_FAILURE


# -------------------- init_sensors: happy path --------------------

def test_init_sensors_captures_absolute_zero_positions(hardware):
    """Startup should snapshot the rotary/ToF/force readings as the absolute zero."""
    hardware.tof.range = 15
    hardware.i2c.adc_bytes = bytes([0, 0])  # 0V -> 0N
    import sensing

    error = sensing.init_sensors(_fake_controller(position=0.25), hardware.pi)

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

    error = sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)

    assert error == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_absolute_zero_position == pytest.approx(0.5)


# -------------------- individual sensor reads --------------------

def test_read_force_sensor_converts_adc_reading(hardware):
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(250.0)
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_force_sensor() == pytest.approx(250.0)


def test_read_tof_sensor_returns_configured_range(hardware):
    hardware.tof.range = 42
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_ToF_sensor() == 42


def test_read_imu_returns_configured_acceleration(hardware):
    hardware.imu.acceleration = (1.0, 2.0, 3.0)
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_IMU() == (1.0, 2.0, 3.0)


# -------------------- read_sensors: validation --------------------

def test_read_sensors_normal_operation_within_limits(hardware):
    """Default harness state (0 force, 0 position, 9.81 accel) is within every limit."""
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.NORMAL_OPERATION


def test_read_sensors_detects_zeroing_finished(hardware):
    """Force past the zeroing threshold (but under the zeroing sensor limit) should
    report ZEROING_FINISHED during a zeroing read."""
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)  # > 35N threshold, < 52.5N limit
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_sensors(ControlMode.ZEROING) == ErrorCode.ZEROING_FINISHED


def test_read_sensors_accel_within_tolerance_is_normal_operation(hardware):
    """IMU shift detection: an orientation within the tolerance band is not a fault."""
    hardware.imu.acceleration = (0.0, 0.0, 9.0)  # under the 9.81 compression limit
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.NORMAL_OPERATION


def test_read_sensors_detects_accel_over_limit(hardware):
    """IMU shift detection: an orientation beyond the tolerance band is a kneel
    failure specifically, not the generic ERROR_SENSOR_FAILURE (contrast with
    test_read_sensors_detects_position_disagreement below, a non-IMU failure)."""
    hardware.imu.acceleration = (0.0, 0.0, 20.0)  # exceeds the 9.81 compression limit
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_read_sensors_detects_position_disagreement(hardware):
    """Rotary and ToF should agree on how far each has moved since zeroing; a
    large mismatch between the two is a sensor failure."""
    import sensing
    controller = _FakeMotorController(position=0.0)
    sensing.init_sensors(cast("MoteusThread", controller), hardware.pi)  # zeroes both sensors to their readings at init
    controller.position = 1.0  # rotary now reports ~62.8mm of travel since zeroing
    # hardware.tof.range is left unchanged, so ToF reports no travel at all --
    # a ~63mm disagreement, far past the 2mm threshold

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_rejects_invalid_control_mode(hardware):
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.read_sensors(ControlMode.HOLD_POSITION) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_i2c_read_failure_mid_operation(hardware):
    """A transient I2C error on a read (after a previously successful init) is
    reported as a sensor failure rather than raising out of read_sensors()."""
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)
    hardware.i2c.raise_on_read = TimeoutError("i2c bus timeout")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_tof_disconnect_mid_operation(hardware):
    """The ToF sensor failing after a previously successful init is a sensor failure."""
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)
    hardware.tof.raise_on_read = RuntimeError("device not found")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


def test_read_sensors_detects_imu_disconnect_mid_operation(hardware):
    """The IMU failing after a previously successful init is a sensor failure."""
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)
    hardware.imu.raise_on_read = RuntimeError("device not found")

    assert sensing.read_sensors(ControlMode.COMPRESSIONS) == ErrorCode.ERROR_SENSOR_FAILURE


# -------------------- zero_position --------------------

def test_zero_position_captures_current_position_once_zeroing_finishes(hardware):
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)  # past the zeroing threshold
    import sensing
    sensing.init_sensors(_fake_controller(position=1.0), hardware.pi)
    hardware.tof.range = 63  # agrees with 1.0 rotation within the 2mm threshold

    assert sensing.zero_position() == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_zero_position == pytest.approx(1.0)
    assert sensing.ToF_zero_position == 63
    assert sensing.get_rotary_zero_position() == pytest.approx(1.0)


def test_zero_position_fails_when_zeroing_not_finished(hardware):
    """With force still under the zeroing threshold, zeroing hasn't finished yet."""
    import sensing
    sensing.init_sensors(_fake_controller(), hardware.pi)

    assert sensing.zero_position() == ErrorCode.ERROR_SENSOR_FAILURE


# -------------------- battery_check --------------------

def test_battery_check_normal_operation(hardware):
    import sensing
    sensing.init_sensors(_fake_controller(voltage=24.0), hardware.pi)

    assert sensing.battery_check() == ErrorCode.NORMAL_OPERATION


def test_battery_check_reports_low_battery(hardware):
    import sensing
    sensing.init_sensors(_fake_controller(voltage=20.0), hardware.pi)  # under the 21.6V threshold

    assert sensing.battery_check() == ErrorCode.ERROR_LOW_BATTERY


def test_battery_check_reports_motor_failure_when_read_fails(hardware):
    import sensing
    sensing.init_sensors(_fake_controller(raise_on_battery=RuntimeError("comm lost")), hardware.pi)

    assert sensing.battery_check() == ErrorCode.ERROR_MOTOR_FAILURE
