"""Tests for actuation.py: motor init, zeroing, compressions, pause, and abort.

Most tests here rely on the real actuation -> sensing -> moteus_thread chain
via the `hardware` fixture. Behavior that depends on the motor controller's
latest queried state (motor error, position, battery voltage) requires a
short sleep after configuring hardware.moteus_controller, since that state is
only updated once MoteusThread's background command-loop thread ticks -- see
the `hardware` fixture's docstring in conftest.py.
"""
import pytest

from Enums.error_codes import ErrorCode
from conftest import init_sensing


# -------------------- init_motor / get_motor_controller --------------------

def test_init_motor_success_creates_motor_controller(hardware):
    import actuation

    assert actuation.init_motor() == ErrorCode.NORMAL_OPERATION
    assert actuation.get_motor_controller() is not None


def test_init_motor_failure_when_moteus_construction_fails(hardware):
    hardware.moteus_connect_error = RuntimeError("no CAN adapter")
    import actuation

    assert actuation.init_motor() == ErrorCode.ERROR_INIT_FAILURE


@pytest.mark.parametrize("q_current", [-12.5, 0.0, 3.0, 40.0])
def test_motor_state_reports_q_current(hardware, q_current):
    """The q-axis current register is requested from the controller and passed through to MotorState."""
    import time
    import actuation

    actuation.init_motor()
    hardware.moteus_controller.q_current = q_current
    time.sleep(0.05)

    motor = actuation.get_motor_controller()
    assert hardware.moteus_controller.query_resolution.q_current == "f32"
    assert motor.get_state().q_current == pytest.approx(q_current)
    assert motor.get_motor_current() == pytest.approx(q_current)


def test_command_loop_without_controller_reports_init_failure(hardware, caplog):
    """The command loop's guard for a missing moteus controller reports an init failure and returns, rather than raising."""
    import asyncio
    from moteus_thread import MoteusThread
    thread = MoteusThread()  # never started, so no controller was constructed
    thread._last_error = ErrorCode.NORMAL_OPERATION  # so the check can't pass on the initial value alone

    asyncio.run(thread._command_loop())

    assert thread.get_last_error() == ErrorCode.ERROR_INIT_FAILURE
    assert "Moteus controller not initialized" in caplog.text


def test_get_motor_controller_returns_same_instance_after_init(hardware):
    import actuation
    actuation.init_motor()

    controller = actuation.get_motor_controller()
    assert actuation.get_motor_controller() is controller


# -------------------- init_zeroing / init_compressions --------------------

def test_init_zeroing_sets_start_time(hardware):
    import time
    import actuation

    before = time.monotonic()
    assert actuation.init_zeroing() == ErrorCode.NORMAL_OPERATION
    after = time.monotonic()

    assert before <= actuation.zeroing_start_time <= after


def test_init_compressions_sets_start_time(hardware):
    import time
    import actuation

    before = time.monotonic()
    assert actuation.init_compressions() == ErrorCode.NORMAL_OPERATION
    after = time.monotonic()

    assert before <= actuation.compression_start_time <= after


# -------------------- zeroing --------------------

def test_zeroing_fails_on_timeout(hardware):
    """The timeout check runs before touching the motor controller at all."""
    import actuation
    actuation.init_zeroing()
    actuation.zeroing_start_time -= 9999  # simulate far more than ZEROING_TIMEOUT_SEC elapsed

    assert actuation.zeroing() == ErrorCode.ERROR_ZEROING_FAILURE


def test_zeroing_fails_on_motor_error(hardware):
    import time
    import actuation
    actuation.init_motor()
    actuation.init_zeroing()

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)  # let the background command loop record the failure

    assert actuation.zeroing() == ErrorCode.ERROR_MOTOR_FAILURE


def test_zeroing_fails_on_max_extension(hardware):
    """The plunger reaching the end of the ball screw without finding the chest is a zeroing failure."""
    import time
    import actuation
    import sensing
    from moteus_thread import BLDC_ROTATIONS_PER_M
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)  # startup position 0.0 rev
    actuation.init_zeroing()

    hardware.moteus_controller.position = actuation.EXTENSION_STROKE_LIMIT_M * BLDC_ROTATIONS_PER_M
    time.sleep(0.05)  # let the background command loop pick up the new position

    assert actuation.zeroing() == ErrorCode.ERROR_ZEROING_FAILURE


def test_zeroing_drives_plunger_down_at_zeroing_velocity(hardware):
    """Zeroing commands a constant velocity down the ball screw (pure velocity control, no position target)."""
    import math
    import time
    import actuation
    import sensing
    from moteus_thread import (ZEROING_VELOCITY_MPS, BLDC_ROTATIONS_PER_M, BLDC_GEAR_RATIO, ZEROING_TORQUE_LIMIT_NM,
                               TORQUE_LIMIT_NM)
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    actuation.init_zeroing()
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.NORMAL_OPERATION

    target = actuation.get_motor_controller()._target
    assert math.isnan(target.position)
    assert target.velocity == pytest.approx(ZEROING_VELOCITY_MPS * BLDC_ROTATIONS_PER_M)
    assert math.copysign(1.0, target.velocity) == math.copysign(1.0, BLDC_GEAR_RATIO)  # down, through the gear train
    # Reduced torque cap, so the q current spike stays below where the current saturates
    assert target.maximum_torque == pytest.approx(ZEROING_TORQUE_LIMIT_NM)
    assert 0 < ZEROING_TORQUE_LIMIT_NM < TORQUE_LIMIT_NM


def test_zeroing_fails_on_moteus_fault(hardware, caplog):
    """A latched controller fault stops zeroing with a motor failure, since the current no longer means anything."""
    import time
    import actuation
    import sensing
    from moteus_thread import MOTEUS_FAULT_MODE
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    actuation.init_zeroing()
    hardware.moteus_controller.mode = MOTEUS_FAULT_MODE
    hardware.moteus_controller.fault = 39
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.ERROR_MOTOR_FAILURE
    assert "moteus fault 39" in caplog.text


def test_zeroing_succeeds_when_sensors_healthy(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    actuation.init_zeroing()

    # zeroing() checks get_last_error(), which only reflects NORMAL_OPERATION
    # once the background command loop has completed its first tick.
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.NORMAL_OPERATION


def test_zeroing_stops_plunger_when_chest_found(hardware):
    """A q current spike finishes zeroing, stops the plunger where it is, and lets zero_position() record it."""
    import math
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    actuation.init_zeroing()

    # Past the settle and baseline windows with a 0 A baseline, one spike sample short of finishing
    sensing.zeroing_start_time -= sensing.ZEROING_SETTLE_TIME_S + sensing.ZEROING_BASELINE_TIME_S
    sensing.zeroing_baseline_count = 1
    sensing.zeroing_spike_count = sensing.ZEROING_SPIKE_CONSECUTIVE_SAMPLES - 1
    hardware.moteus_controller.q_current = sensing.ZEROING_SPIKE_THRESHOLD_A + 0.5
    time.sleep(0.05)  # let the background command loop pick up the new current

    assert actuation.zeroing() == ErrorCode.ZEROING_FINISHED
    assert sensing.zero_position() == ErrorCode.NORMAL_OPERATION

    target = actuation.get_motor_controller()._target
    assert math.isnan(target.position)
    assert target.velocity == 0.0


def test_retract_plunger_reports_home_once_at_startup_position(hardware):
    """retract_plunger() commands the plunger back to the startup position, and only reports home once it is there."""
    import time
    import actuation
    import sensing
    from moteus_thread import BLDC_ROTATIONS_PER_M
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)  # startup position 0.0 rev

    hardware.moteus_controller.position = 0.05 * BLDC_ROTATIONS_PER_M  # 5 cm down the ball screw
    time.sleep(0.05)
    assert actuation.retract_plunger() == (ErrorCode.NORMAL_OPERATION, False)
    assert actuation.get_motor_controller()._target.position == pytest.approx(0.0)

    hardware.moteus_controller.position = 0.0
    time.sleep(0.05)
    assert actuation.retract_plunger() == (ErrorCode.NORMAL_OPERATION, True)


def test_retract_plunger_reports_motor_failure(hardware):
    """A motor communication failure while retracting surfaces as a motor failure rather than raising."""
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)

    assert actuation.retract_plunger() == (ErrorCode.ERROR_MOTOR_FAILURE, False)


# -------------------- motor direction --------------------

def test_gear_train_reverses_bldc(hardware):
    """The 1:1 gear train reverses rotation (matching scripts/slow_jog.py), so the gear ratio is negative. Every other
    direction is derived from this sign; update this test only if the gear train changes."""
    import moteus_thread

    assert moteus_thread.BLDC_GEAR_RATIO == -1.0
    assert moteus_thread.BLDC_ROTATIONS_PER_M < 0  # down on the ball screw is negative motor rotation
    assert moteus_thread.TORQUE_LIMIT_NM > 0


def test_bldc_targets_follow_gear_ratio_sign(hardware):
    """Plunger travel down converts to motor rotation in the direction BLDC_GEAR_RATIO sets, while velocity
    limits stay positive magnitudes whichever way the gear train turns."""
    import math
    from Enums.control_modes import ControlMode
    from moteus_thread import MoteusThread, BLDC_GEAR_RATIO, BLDC_ROTATIONS_PER_M
    thread = MoteusThread()  # never started, set_target() only stores the target
    abs_zero = 3.0
    zero = abs_zero + 0.04 * BLDC_ROTATIONS_PER_M  # chest found 4 cm down the ball screw

    assert thread.set_target(ControlMode.ZEROING, abs_zero, abs_zero) == ErrorCode.NORMAL_OPERATION
    assert math.copysign(1.0, thread._target.velocity) == math.copysign(1.0, BLDC_GEAR_RATIO)

    assert thread.set_target(ControlMode.COMPRESSIONS, zero, abs_zero, 0.07) == ErrorCode.NORMAL_OPERATION
    assert math.copysign(1.0, thread._target.position - zero) == math.copysign(1.0, BLDC_GEAR_RATIO)

    for mode in (ControlMode.PAUSE_RETRACT, ControlMode.ABORT_RETRACT):
        assert thread.set_target(mode, zero, abs_zero) == ErrorCode.NORMAL_OPERATION
        assert thread._target.velocity_limit > 0

    # Above the top of travel (the wrong way for the gear train) is still rejected
    assert thread.set_target(ControlMode.COMPRESSIONS, abs_zero, abs_zero, -0.01) == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- computeCompressionSetpoint (pure) --------------------

def test_compute_compression_setpoint_piecewise_profile(hardware, monkeypatch):
    """The trapezoidal waveform should stay flat, ramp up, plateau, ramp down, and repeat."""
    import actuation

    fake_time = {"t": 0.0}
    monkeypatch.setattr(actuation.time, "monotonic", lambda: fake_time["t"])
    actuation.init_compressions()  # compression_start_time = 0.0
    peak = actuation.COMPRESSION_DEPTH_CM / 100.0

    fake_time["t"] = 0.05  # 0.00-0.12s: still at the top
    assert actuation.computeCompressionSetpoint() == pytest.approx(0.0)

    fake_time["t"] = 0.18  # 0.12-0.24s: ramping down toward full compression
    assert 0.0 < actuation.computeCompressionSetpoint() < peak

    fake_time["t"] = 0.28  # 0.24-0.323s: fully compressed
    assert actuation.computeCompressionSetpoint() == pytest.approx(peak)

    fake_time["t"] = 0.50  # 0.323-0.56s: ramping back up
    assert 0.0 < actuation.computeCompressionSetpoint() < peak

    fake_time["t"] = 0.56 + 0.05  # cycle repeats every 0.56s
    assert actuation.computeCompressionSetpoint() == pytest.approx(0.0)


def test_compute_compression_setpoint_exact_boundary_values(hardware, monkeypatch):
    """The test above checks interior points of each segment; this pins down the
    exact transition instants so a boundary can't silently fall into the wrong
    branch (e.g. an off-by-one in a `<` vs `<=` comparison)."""
    import actuation

    fake_time = {"t": 0.0}
    monkeypatch.setattr(actuation.time, "monotonic", lambda: fake_time["t"])
    actuation.init_compressions()
    peak = actuation.COMPRESSION_DEPTH_CM / 100.0

    fake_time["t"] = 0.12  # flat -> ramp boundary: ramp is just starting, still 0
    assert actuation.computeCompressionSetpoint() == pytest.approx(0.0)

    fake_time["t"] = 0.24  # ramp -> plateau boundary: ramp just finished, at peak
    assert actuation.computeCompressionSetpoint() == pytest.approx(peak)

    fake_time["t"] = 0.323  # plateau -> ramp-down boundary: still at peak
    assert actuation.computeCompressionSetpoint() == pytest.approx(peak)


# -------------------- compressions --------------------

def test_compressions_normal_operation(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)

    # compressions() checks get_last_error(), which only reflects NORMAL_OPERATION
    # once the background command loop has completed its first tick.
    time.sleep(0.05)

    assert actuation.compressions() == ErrorCode.NORMAL_OPERATION


def test_compressions_fails_on_sensor_error(hardware):
    """A non-IMU sensor failure (here, rotary/ToF position disagreement) should
    convert to the generic ERROR_SENSOR_FAILURE -- contrast with
    test_compressions_propagates_imu_kneel_failure below, where an IMU-specific
    failure is passed through unchanged instead."""
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    hardware.moteus_controller.position = 1.0  # 10mm of ball screw travel
    hardware.tof.range = 0  # ToF disagrees by 10mm, far past the 2mm threshold
    time.sleep(0.05)  # let the background command loop pick up the new position

    assert actuation.compressions() == ErrorCode.ERROR_SENSOR_FAILURE


def test_compressions_propagates_imu_kneel_failure(hardware):
    """ERROR_IMU_KNEEL_FAILURE from read_sensors() must pass through as-is, unlike
    every other non-normal result, which compressions() converts to
    ERROR_SENSOR_FAILURE (see test_compressions_fails_on_sensor_error above)."""
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    hardware.imu.acceleration = (0.0, 0.0, 20.0)  # exceeds the compression accel limit

    assert actuation.compressions() == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_compressions_fails_on_motor_error(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)

    assert actuation.compressions() == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- pause_compressions --------------------

def test_pause_compressions_normal_operation(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)

    # pause_compressions() checks get_last_error(), which only reflects
    # NORMAL_OPERATION once the background command loop has completed its
    # first tick.
    time.sleep(0.05)

    assert actuation.pause_compressions() == ErrorCode.NORMAL_OPERATION


def test_pause_compressions_fails_on_sensor_error(hardware):
    """Same distinction as test_compressions_fails_on_sensor_error above, for
    pause_compressions()."""
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    hardware.moteus_controller.position = 1.0
    hardware.tof.range = 0
    time.sleep(0.05)  # let the background command loop pick up the new position

    assert actuation.pause_compressions() == ErrorCode.ERROR_SENSOR_FAILURE


def test_pause_compressions_propagates_imu_kneel_failure(hardware):
    """Same distinction as test_compressions_propagates_imu_kneel_failure above,
    for pause_compressions()."""
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    hardware.imu.acceleration = (0.0, 0.0, 20.0)

    assert actuation.pause_compressions() == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_pause_compressions_fails_on_motor_error(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)

    assert actuation.pause_compressions() == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- abort_compressions --------------------

def test_abort_compressions_does_not_require_sensing_init(hardware):
    """Abort intentionally skips sensor reads to return to zero as fast as possible."""
    import actuation
    actuation.init_motor()
    # Deliberately skip init_sensing() -- abort must not depend on it.

    assert actuation.abort_compressions() == ErrorCode.NORMAL_OPERATION
