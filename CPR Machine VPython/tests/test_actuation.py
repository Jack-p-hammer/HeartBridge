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


# -------------------- init_motor / get_motor_controller --------------------

def test_init_motor_success_creates_motor_controller(hardware):
    import actuation

    assert actuation.init_motor() == ErrorCode.NORMAL_OPERATION
    assert actuation.get_motor_controller() is not None


def test_init_motor_failure_when_moteus_construction_fails(hardware):
    hardware.moteus_connect_error = RuntimeError("no CAN adapter")
    import actuation

    assert actuation.init_motor() == ErrorCode.ERROR_INIT_FAILURE


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
    actuation.init_motor()
    actuation.init_zeroing()

    hardware.moteus_controller.position = actuation.EXTENSION_STROKE_LIMIT_M + 1.0
    time.sleep(0.05)

    assert actuation.zeroing() == ErrorCode.ERROR_ZEROING_FAILURE


def test_zeroing_succeeds_when_sensors_healthy(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)
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
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)

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
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)
    hardware.moteus_controller.position = 1.0  # ~62.8mm of rotary travel
    hardware.tof.range = 0  # ToF disagrees by ~63mm, far past the 2mm threshold
    time.sleep(0.05)  # let the background command loop pick up the new position

    assert actuation.compressions() == ErrorCode.ERROR_SENSOR_FAILURE


def test_compressions_propagates_imu_kneel_failure(hardware):
    """ERROR_IMU_KNEEL_FAILURE from read_sensors() must pass through as-is, unlike
    every other non-normal result, which compressions() converts to
    ERROR_SENSOR_FAILURE (see test_compressions_fails_on_sensor_error above)."""
    import actuation
    import sensing
    actuation.init_motor()
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)
    hardware.imu.acceleration = (0.0, 0.0, 20.0)  # exceeds the compression accel limit

    assert actuation.compressions() == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_compressions_fails_on_motor_error(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)

    assert actuation.compressions() == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- pause_compressions --------------------

def test_pause_compressions_normal_operation(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)

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
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)
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
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)
    hardware.imu.acceleration = (0.0, 0.0, 20.0)

    assert actuation.pause_compressions() == ErrorCode.ERROR_IMU_KNEEL_FAILURE


def test_pause_compressions_fails_on_motor_error(hardware):
    import time
    import actuation
    import sensing
    actuation.init_motor()
    sensing.init_sensors(actuation.get_motor_controller(), hardware.pi)

    hardware.moteus_controller.raise_on_set_position = RuntimeError("comm lost")
    time.sleep(0.05)

    assert actuation.pause_compressions() == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- abort_compressions --------------------

def test_abort_compressions_does_not_require_sensing_init(hardware):
    """Abort intentionally skips sensor reads to return to zero as fast as possible."""
    import actuation
    actuation.init_motor()
    # Deliberately skip sensing.init_sensors(, hardware.pi) -- abort must not depend on it.

    assert actuation.abort_compressions() == ErrorCode.NORMAL_OPERATION
