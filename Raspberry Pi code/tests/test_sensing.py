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
import types
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
        # Latest motor sample for zeroing's current spike check, see get_state()
        self.q_current = 0.0
        self.timestamp = 0.0
        self.stale = False

    def get_state(self):
        """Only the MotorState fields sensing.py reads"""
        return types.SimpleNamespace(q_current=self.q_current, timestamp=self.timestamp, stale=self.stale)

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


def test_init_sensors_stores_tof_sensor_for_reads(hardware):
    """Regression test: init_sensors() must construct the sensor as adafruit_vl53l0x.VL53L0X (the fake
    library only has that exact name) and store it in the `_vl53` global that read_ToF_sensor() reads.
    Both were once misspelled (`VL53l0X`, `_vl61`), which crashed init and every later ToF read."""
    hardware.tof.range = 42
    import sensing

    assert init_sensing(sensing, _fake_controller(), hardware) == ErrorCode.NORMAL_OPERATION
    assert getattr(sensing, "_vl53") is hardware.tof
    assert sensing.ToF_absolute_zero_position == 42
    assert sensing.read_ToF_sensor() == 42


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


def _start_zeroing(hardware) -> tuple:
    """Init sensing on a _FakeMotorController and start a zeroing run. Returns (sensing, controller)"""
    import sensing
    controller = _FakeMotorController()
    init_sensing(sensing, cast("MoteusThread", controller), hardware)
    sensing.init_zeroing()
    return sensing, controller


def _zeroing_read(sensing, controller, time_s: float, q_current: float) -> ErrorCode:
    """Feed one motor sample, time_s after zeroing started, through a zeroing sensor read"""
    controller.timestamp = sensing.zeroing_start_time + time_s
    controller.q_current = q_current
    return sensing.read_sensors(ControlMode.ZEROING)


def _feed_baseline(sensing, controller, baseline_a: float) -> None:
    """Samples through the settle and baseline windows at a steady free-running current"""
    start_s = sensing.ZEROING_SETTLE_TIME_S
    for i in range(5):
        time_s = start_s + sensing.ZEROING_BASELINE_TIME_S * i / 5
        assert _zeroing_read(sensing, controller, time_s, baseline_a) == ErrorCode.NORMAL_OPERATION


def test_read_sensors_detects_zeroing_current_spike(hardware):
    """A q current spike above the free-running baseline for ZEROING_SPIKE_CONSECUTIVE_SAMPLES samples in a row
    finishes zeroing, on the last of those samples. The current's sign doesn't matter."""
    sensing, controller = _start_zeroing(hardware)
    _feed_baseline(sensing, controller, -2.0)
    after_baseline_s = sensing.ZEROING_SETTLE_TIME_S + sensing.ZEROING_BASELINE_TIME_S
    spike_a = -(2.0 + sensing.ZEROING_SPIKE_THRESHOLD_A + 0.5)

    for i in range(sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES - 1):
        assert _zeroing_read(sensing, controller, after_baseline_s + 0.05 * i, spike_a) == ErrorCode.NORMAL_OPERATION
    assert _zeroing_read(sensing, controller, after_baseline_s + 1.0, spike_a) == ErrorCode.ZEROING_FINISHED
    assert sensing.zeroing_contact_found


def test_read_sensors_ignores_short_current_spike(hardware):
    """A spike shorter than ZEROING_SPIKE_CONSECUTIVE_SAMPLES resets once the current drops, rejecting noise."""
    sensing, controller = _start_zeroing(hardware)
    _feed_baseline(sensing, controller, 2.0)
    time_s = sensing.ZEROING_SETTLE_TIME_S + sensing.ZEROING_BASELINE_TIME_S
    spike_a = 2.0 + sensing.ZEROING_SPIKE_THRESHOLD_A + 0.5

    for q_current in [spike_a] * (sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES - 1) + [2.0] + [spike_a]:
        time_s += 0.05
        assert _zeroing_read(sensing, controller, time_s, q_current) == ErrorCode.NORMAL_OPERATION
    assert not sensing.zeroing_contact_found


def test_read_sensors_ignores_current_during_settle_time(hardware):
    """The acceleration transient right after zeroing starts isn't counted as contact or into the baseline."""
    sensing, controller = _start_zeroing(hardware)

    for i in range(sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES + 2):
        time_s = sensing.ZEROING_SETTLE_TIME_S * i / (sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES + 3)
        assert _zeroing_read(sensing, controller, time_s, 50.0) == ErrorCode.NORMAL_OPERATION
    assert sensing.zeroing_baseline_count == 0


def test_read_sensors_counts_each_motor_sample_once(hardware):
    """Reads between moteus replies see the same sample again, which is not counted twice."""
    sensing, controller = _start_zeroing(hardware)
    _feed_baseline(sensing, controller, 2.0)
    time_s = sensing.ZEROING_SETTLE_TIME_S + sensing.ZEROING_BASELINE_TIME_S + 0.05

    for _ in range(sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES + 2):
        assert _zeroing_read(sensing, controller, time_s, 10.0) == ErrorCode.NORMAL_OPERATION
    assert sensing.zeroing_spike_count == 1


def test_read_sensors_reports_missing_current_baseline(hardware, caplog):
    """No samples landing in the baseline window (e.g. replies stalled) is a zeroing failure, not a divide by zero."""
    sensing, controller = _start_zeroing(hardware)
    after_baseline_s = sensing.ZEROING_SETTLE_TIME_S + sensing.ZEROING_BASELINE_TIME_S

    assert _zeroing_read(sensing, controller, after_baseline_s + 0.1, 2.0) == ErrorCode.ERROR_ZEROING_FAILURE
    assert "No samples collected for the zeroing current baseline" in caplog.text


def test_read_sensors_zeroing_ignores_force_for_contact(hardware):
    """Force alone (under the zeroing force limit) no longer finishes zeroing, only the current spike does."""
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)
    sensing, controller = _start_zeroing(hardware)

    assert _zeroing_read(sensing, controller, 0.1, 0.0) == ErrorCode.NORMAL_OPERATION


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
    import sensing
    init_sensing(sensing, _fake_controller(position=1.0), hardware)
    hardware.tof.range = 10
    sensing.init_zeroing()
    sensing.zeroing_contact_found = True  # the current spike was found

    assert sensing.zero_position() == ErrorCode.NORMAL_OPERATION
    assert sensing.rotary_zero_position == pytest.approx(1.0)
    assert sensing.ToF_zero_position == 10
    assert sensing.get_rotary_zero_position() == pytest.approx(1.0)


def test_zero_position_fails_when_zeroing_not_finished(hardware):
    """With no current spike found yet, zeroing hasn't finished, even with force on the plunger."""
    hardware.i2c.adc_bytes = _raw_adc_bytes_for_force(40.0)
    import sensing
    init_sensing(sensing, _fake_controller(), hardware)
    sensing.init_zeroing()

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
