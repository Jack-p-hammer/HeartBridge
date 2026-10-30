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


def test_command_loop_without_controller_reports_init_failure(hardware, caplog):
    """The command loop's guard for a missing moteus controller reports an init failure and returns, rather than raising."""
    import asyncio
    from moteus_thread import MoteusThread
    thread = MoteusThread()  # never started, so no controller was constructed
    thread._last_error = ErrorCode.NORMAL_OPERATION  # so the check can't pass on the initial value alone

    asyncio.run(thread._command_loop())

    assert thread.get_last_error() == ErrorCode.ERROR_INIT_FAILURE
    assert "Moteus controller not initialized" in caplog.text


def _daemon_gone(*args, **kwargs):
    """Stand-in for a pigpio call that raises, e.g. the daemon or its socket going away."""
    raise RuntimeError("daemon gone")


@pytest.mark.parametrize("pigpio_method, pin_name", [
    ("set_mode", None),
    ("write", "STEPPER_STEP_PIN"),
    ("write", "STEPPER_DIR_PIN"),
    ("write", "STEPPER_ENABLE_PIN"),
    ("wave_clear", None),
])
def test_init_steppers_reports_pigpio_failure(hardware, monkeypatch, caplog, pigpio_method, pin_name):
    """Each pigpio call while setting up the steppers that raises is an init failure, not an exception.
    pin_name picks out which of the write() calls fails."""
    import actuation
    import stepper

    if pin_name is None:
        monkeypatch.setattr(hardware.pi, pigpio_method, _daemon_gone)
    else:
        failing_pin = getattr(stepper, pin_name)
        real_write = hardware.pi.write
        monkeypatch.setattr(hardware.pi, "write",
                            lambda pin, value: _daemon_gone() if pin == failing_pin else real_write(pin, value))

    assert actuation.init_steppers(hardware.pi) == ErrorCode.ERROR_INIT_FAILURE
    assert "Stepper GPIO initialization failed" in caplog.text


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
    import time
    import actuation
    import stepper
    actuation.init_motor()
    actuation.init_steppers(hardware.pi)
    actuation.init_zeroing()

    # Carriage reached the end of the lead screws without finding the chest
    stepper.position_steps = round(actuation.EXTENSION_STROKE_LIMIT_M * stepper.STEPS_PER_M)
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.ERROR_ZEROING_FAILURE


def test_zeroing_lowers_carriage_with_steppers(hardware):
    import time
    import actuation
    actuation.init_motor()
    actuation.init_steppers(hardware.pi)
    actuation.init_zeroing()
    time.sleep(0.05)

    for _ in range(5):
        actuation.zeroing()

    assert actuation.stepper.get_position() > 0.0


@pytest.mark.parametrize("position_m, velocity_mps, log_text", [
    (-0.01, 0.04, "Invalid stepper target"),                        # above home
    (1e6, 0.04, "Invalid stepper target"),                          # past the end of the lead screws
    (float("nan"), 0.04, "Invalid stepper target"),
    (0.05, 0.0, "Invalid stepper target"),
    (0.05, float("nan"), "Invalid stepper target"),
    ("0.05", 0.04, "Stepper target could not be checked"),          # wrong type raises in the range check
])
def test_stepper_set_target_rejects_invalid_targets(hardware, caplog, position_m, velocity_mps, log_text):
    """Invalid stepper targets report a motor failure instead of raising or being accepted."""
    import stepper
    stepper.init_steppers(hardware.pi)

    assert stepper.set_target(position_m, velocity_mps) == ErrorCode.ERROR_MOTOR_FAILURE
    assert log_text in caplog.text
    assert stepper.get_setpoint() == 0.0  # previous setpoint kept


def test_zeroing_reports_stepper_failure_and_halts(hardware, caplog):
    """A pigpio failure while lowering the carriage is a motor failure, and the steppers stop."""
    import time
    import actuation
    import stepper
    actuation.init_motor()
    actuation.init_steppers(hardware.pi)
    actuation.init_zeroing()
    time.sleep(0.05)
    hardware.pi.raise_on_write = RuntimeError("daemon gone")

    assert actuation.zeroing() == ErrorCode.ERROR_MOTOR_FAILURE
    assert stepper.get_setpoint() == stepper.get_position()
    assert "Failed to build stepper waveform" in caplog.text


def test_stepper_update_reports_rejected_waveform(hardware, monkeypatch, caplog):
    """pigpio rejecting a waveform (negative wave_create status) is a motor failure, not an exception."""
    import stepper
    stepper.init_steppers(hardware.pi)
    stepper.set_target(0.05, stepper.ZEROING_VELOCITY_MPS)
    monkeypatch.setattr(hardware.pi, "wave_create", lambda: -1)

    assert stepper.update() == ErrorCode.ERROR_MOTOR_FAILURE
    assert "wave_create failed" in caplog.text


@pytest.mark.parametrize("pigpio_method, log_text", [
    ("wave_tx_at", "Failed to read the playing stepper waveform"),
    ("wave_delete", "Failed to delete stepper waveform"),
    ("write", "Failed to build stepper waveform"),
    ("wave_add_generic", "Failed to build stepper waveform"),
    ("wave_create", "Failed to build stepper waveform"),
    ("wave_send_using_mode", "Failed to send stepper waveform"),
])
def test_stepper_update_reports_pigpio_failure(hardware, monkeypatch, caplog, pigpio_method, log_text):
    """Each pigpio call in update() that raises is a motor failure, not an exception."""
    import stepper
    stepper.init_steppers(hardware.pi)
    stepper.chunk_queue = [(0, 1)]  # a finished chunk to retire (fake daemon reports nothing playing)
    stepper.set_target(0.05, stepper.ZEROING_VELOCITY_MPS)  # and more steps to queue
    monkeypatch.setattr(hardware.pi, pigpio_method, _daemon_gone)

    assert stepper.update() == ErrorCode.ERROR_MOTOR_FAILURE
    assert log_text in caplog.text


@pytest.mark.parametrize("actuation_fn_name", ["compressions", "pause_compressions"])
def test_compressions_and_pause_report_stepper_failure(hardware, monkeypatch, actuation_fn_name, caplog):
    """A pigpio failure while the steppers finish their last queued chunk is a motor failure in compressions/pause."""
    import time
    import actuation
    import sensing
    import stepper
    actuation.init_motor()
    actuation.init_steppers(hardware.pi)
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    stepper.chunk_queue = [(0, 1)]  # a chunk still playing out from zeroing

    def daemon_gone():
        raise RuntimeError("daemon gone")

    monkeypatch.setattr(hardware.pi, "wave_tx_at", daemon_gone)
    time.sleep(0.05)  # let the background command loop report a healthy motor first

    assert getattr(actuation, actuation_fn_name)() == ErrorCode.ERROR_MOTOR_FAILURE
    assert "Failed to read the playing stepper waveform" in caplog.text


def test_retract_carriage_reports_pigpio_failure(hardware):
    """A pigpio failure while retracting surfaces as a motor failure rather than raising."""
    import actuation
    import stepper
    actuation.init_steppers(hardware.pi)
    stepper.position_steps = round(0.05 * stepper.STEPS_PER_M)  # carriage lowered, so retracting needs new steps
    hardware.pi.raise_on_write = RuntimeError("daemon gone")

    assert actuation.retract_carriage() == ErrorCode.ERROR_MOTOR_FAILURE


def test_zeroing_succeeds_when_sensors_healthy(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    actuation.init_steppers(hardware.pi)
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    actuation.init_zeroing()

    # zeroing() checks get_last_error(), which only reflects NORMAL_OPERATION
    # once the background command loop has completed its first tick.
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.NORMAL_OPERATION


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
