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


def test_lost_frame_is_only_a_motor_failure_after_tolerance(hardware):
    """A lost frame shortly after a good reply is tolerated; with no good reply for LOST_REPLY_TOLERANCE_S, or none
    yet at all, it's reported."""
    import time
    import dataclasses
    from moteus_thread import MoteusThread, LOST_REPLY_TOLERANCE_S
    thread = MoteusThread()  # never started, state set by hand
    thread._last_error = ErrorCode.ERROR_MOTOR_FAILURE
    thread._last_error_from_lost_frame = True

    # No reply yet: reported straight away
    assert thread.get_last_error() == ErrorCode.ERROR_MOTOR_FAILURE

    # Good reply just now: the lost frame is tolerated
    thread._state = dataclasses.replace(thread._state, stale=False, timestamp=time.monotonic())
    assert thread.get_last_error() == ErrorCode.NORMAL_OPERATION

    # No good reply for longer than the tolerance: reported
    thread._state = dataclasses.replace(thread._state, timestamp=time.monotonic() - 2.0 * LOST_REPLY_TOLERANCE_S)
    assert thread.get_last_error() == ErrorCode.ERROR_MOTOR_FAILURE


def test_other_motor_errors_are_reported_straight_away(hardware):
    """Only lost frames are tolerated: any other failed cycle is reported even right after a good reply."""
    import time
    import dataclasses
    from moteus_thread import MoteusThread
    thread = MoteusThread()  # never started, state set by hand
    thread._state = dataclasses.replace(thread._state, stale=False, timestamp=time.monotonic())
    thread._last_error = ErrorCode.ERROR_MOTOR_FAILURE
    thread._last_error_from_lost_frame = False

    assert thread.get_last_error() == ErrorCode.ERROR_MOTOR_FAILURE


@pytest.mark.parametrize("failure, from_lost_frame", [
    (RuntimeError("Error: b'ERR unknown flag'"), True),   # adapter rejecting a command joined onto a lost one
    (RuntimeError("comm lost"), False),                   # anything else
])
def test_command_loop_marks_only_lost_frame_errors(hardware, failure, from_lost_frame):
    """The command loop marks the adapter's "unknown flag" error as a lost frame, and nothing else."""
    import time
    import actuation
    actuation.init_motor()
    time.sleep(0.05)  # commanding normally
    hardware.moteus_controller.raise_on_set_position = failure
    time.sleep(0.05)

    motor = actuation.get_motor_controller()
    assert motor._last_error == ErrorCode.ERROR_MOTOR_FAILURE
    assert motor._last_error_from_lost_frame == from_lost_frame


def test_position_timeout_mode_is_cleared_with_a_stop(hardware, caplog):
    """A reply in position timeout mode (the moteus ignoring commands) makes the command loop send a stop to clear it."""
    import time
    import actuation
    from moteus_thread import MODE_POSITION_TIMEOUT
    actuation.init_motor()
    time.sleep(0.05)  # startup stop done, commanding normally
    stops_before = hardware.moteus_controller.stop_count

    hardware.moteus_controller.mode = MODE_POSITION_TIMEOUT
    time.sleep(0.05)

    assert hardware.moteus_controller.stop_count == stops_before + 1  # one stop clears it (the fake goes to mode 0)
    assert hardware.moteus_controller.mode == 0
    assert "position timeout" in caplog.text


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
    stepper.extended_steps = stepper.position_steps  # as if those steps were commanded down
    hardware.pi.raise_on_write = RuntimeError("daemon gone")

    assert actuation.retract_carriage() == (ErrorCode.ERROR_MOTOR_FAILURE, False)


def _run_steppers_to(stepper, position_m: float, velocity_mps: float) -> None:
    """Drive the steppers to a position on the fake pigpio, whose waves finish instantly"""
    assert stepper.set_target(position_m, velocity_mps) == ErrorCode.NORMAL_OPERATION
    for _ in range(10_000):
        assert stepper.update() == ErrorCode.NORMAL_OPERATION
        if stepper.at_target():
            return
    raise AssertionError("Steppers never reached the target")


def test_stepper_counts_only_extending_steps(hardware):
    """Every step commanded down adds to the extending count, and steps commanded up don't reduce it."""
    import stepper
    stepper.init_steppers(hardware.pi)

    _run_steppers_to(stepper, 0.05, stepper.ZEROING_VELOCITY_MPS)
    assert stepper.extended_steps == round(0.05 * stepper.STEPS_PER_M)

    _run_steppers_to(stepper, 0.03, stepper.RETRACT_VELOCITY_MPS)
    assert stepper.extended_steps == round(0.05 * stepper.STEPS_PER_M)
    assert stepper.get_extended_distance() == pytest.approx(0.05)

    stepper.reset_extended_steps()
    assert stepper.extended_steps == 0


def test_retract_carriage_retracts_exactly_the_extended_steps(hardware):
    """retract_carriage() moves the carriage up by exactly the commanded extending steps, reports home once there,
    and resets the count so a second extend and retract cycle only undoes the second extension."""
    import actuation
    import stepper
    actuation.init_steppers(hardware.pi)

    for depth_m in (0.05, 0.02):
        _run_steppers_to(stepper, depth_m, stepper.ZEROING_VELOCITY_MPS)
        assert stepper.extended_steps == round(depth_m * stepper.STEPS_PER_M)

        for _ in range(10_000):
            error, home = actuation.retract_carriage()
            assert error == ErrorCode.NORMAL_OPERATION
            if home:
                break
        else:
            raise AssertionError("Carriage never reported home")

        assert stepper.position_steps == 0
        assert stepper.extended_steps == 0
        assert actuation.retract_target_m is None


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


# -------------------- motor directions --------------------

def test_gear_trains_reverse_both_motors(hardware):
    """Both 1:1 gear trains reverse rotation (matching scripts/slow_jog.py), so both gear ratios are negative and
    the steppers drive down with DIR low. Every other direction is derived from these signs; update this test
    only if the gear trains change."""
    import moteus_thread
    import stepper

    assert moteus_thread.BLDC_GEAR_RATIO == -1.0
    assert moteus_thread.BLDC_ROTATIONS_PER_M < 0  # down on the ball screw is negative motor rotation
    assert moteus_thread.TORQUE_LIMIT_NM > 0
    assert stepper.STEPPER_GEAR_RATIO == -1.0
    assert stepper.STEPPER_DIR_DOWN_LEVEL == 0
    assert stepper.STEPS_PER_M > 0  # step counts stay positive down, DIR carries the direction


def test_bldc_targets_follow_gear_ratio_sign(hardware):
    """Plunger travel down converts to motor rotation in the direction BLDC_GEAR_RATIO sets, while velocity
    limits stay positive magnitudes whichever way the gear train turns."""
    import math
    from Enums.control_modes import ControlMode
    from moteus_thread import MoteusThread, BLDC_GEAR_RATIO, BLDC_ROTATIONS_PER_M, BALL_SCREW_TOP_CLEARANCE_M
    thread = MoteusThread()  # never started, set_target() only stores the target
    abs_zero = 3.0
    zero = abs_zero + BALL_SCREW_TOP_CLEARANCE_M * BLDC_ROTATIONS_PER_M  # where zeroing leaves the plunger

    assert thread.set_target(ControlMode.ZEROING, abs_zero, abs_zero) == ErrorCode.NORMAL_OPERATION
    assert math.copysign(1.0, thread._target.position - abs_zero) == math.copysign(1.0, BLDC_GEAR_RATIO)
    assert thread._target.velocity_limit > 0

    assert thread.set_target(ControlMode.COMPRESSIONS, zero, abs_zero, 0.07,
                             compressionVelocityLimit=0.5, compressionAccelLimit=5.0) == ErrorCode.NORMAL_OPERATION
    assert math.copysign(1.0, thread._target.position - zero) == math.copysign(1.0, BLDC_GEAR_RATIO)
    assert thread._target.velocity_limit > 0
    assert thread._target.accel_limit > 0

    # Plunger moving down (positive velocity) turns the motor the same way as moving the position down
    assert thread.set_target(ControlMode.COMPRESSIONS_STREAMED_DEPRECATED, zero, abs_zero, 0.03,
                             compressionVelocity=0.5) == ErrorCode.NORMAL_OPERATION
    assert math.copysign(1.0, thread._target.velocity) == math.copysign(1.0, BLDC_GEAR_RATIO)

    for mode in (ControlMode.PAUSE_RETRACT, ControlMode.ABORT_RETRACT):
        assert thread.set_target(mode, zero, abs_zero) == ErrorCode.NORMAL_OPERATION
        assert thread._target.velocity_limit > 0

    # Above the top of travel (the wrong way for the gear train) is still rejected
    assert thread.set_target(ControlMode.COMPRESSIONS, abs_zero, abs_zero, -0.01,
                             compressionVelocityLimit=0.5, compressionAccelLimit=5.0) == ErrorCode.ERROR_MOTOR_FAILURE


def test_stepper_dir_pin_follows_gear_ratio_sign(hardware):
    """Lowering the carriage writes STEPPER_DIR_DOWN_LEVEL to the DIR pin, and raising it writes the other level."""
    import stepper
    stepper.init_steppers(hardware.pi)

    def last_dir_write() -> int:
        return [value for pin, value in hardware.pi.writes if pin == stepper.STEPPER_DIR_PIN][-1]

    stepper.set_target(0.05, stepper.ZEROING_VELOCITY_MPS)
    assert stepper.update() == ErrorCode.NORMAL_OPERATION
    assert last_dir_write() == stepper.STEPPER_DIR_DOWN_LEVEL

    stepper.init_steppers(hardware.pi)
    stepper.position_steps = round(0.05 * stepper.STEPS_PER_M)  # carriage lowered, so retracting moves up
    stepper.set_target(0.0, stepper.RETRACT_VELOCITY_MPS)
    assert stepper.update() == ErrorCode.NORMAL_OPERATION
    assert last_dir_write() == 1 - stepper.STEPPER_DIR_DOWN_LEVEL


# -------------------- computeCompressionSetpoint (pure) --------------------

def test_compute_compression_setpoint_flips_between_stroke_ends(hardware, monkeypatch):
    """The setpoint is the extreme of the current stroke: the bottom for the first half of each cycle, the top for the second."""
    import actuation

    fake_time = {"t": 0.0}
    monkeypatch.setattr(actuation.time, "monotonic", lambda: fake_time["t"])
    actuation.init_compressions()  # compression_start_time = 0.0
    depth = actuation.COMPRESSION_DEPTH_CM / 100.0
    stroke = actuation.STROKE_SEC

    expected = [
        (0.0, depth),                 # down stroke starts straight away
        (0.5 * stroke, depth),
        (0.999 * stroke, depth),
        (stroke, 0.0),                # bottom reached, head back up
        (1.5 * stroke, 0.0),
        (2.0 * stroke, depth),        # cycle repeats
        (2.5 * stroke, depth),
    ]
    for t, position in expected:
        fake_time["t"] = t
        assert actuation.computeCompressionSetpoint() == pytest.approx(position), t


def test_compression_caps_make_each_stroke_take_stroke_sec(hardware):
    """Speeding up and slowing down at the acceleration cap, cruising at the speed cap, covers the depth from rest to rest in exactly STROKE_SEC, a third each."""
    import actuation

    depth = actuation.COMPRESSION_DEPTH_CM / 100.0
    accel_time = actuation.COMPRESSION_SPEED_MPS / actuation.COMPRESSION_ACCEL_MPS2
    assert accel_time == pytest.approx(actuation.STROKE_SEC / 3.0)
    ramps_m = actuation.COMPRESSION_SPEED_MPS * accel_time  # both ramps together
    cruise_time = (depth - ramps_m) / actuation.COMPRESSION_SPEED_MPS
    assert 2.0 * accel_time + cruise_time == pytest.approx(actuation.STROKE_SEC)


def test_compressions_target_rejected_without_caps(hardware):
    """COMPRESSIONS without both the speed and acceleration caps would jump to each extreme, so it's refused."""
    import math
    from Enums.control_modes import ControlMode
    from moteus_thread import MoteusThread

    thread = MoteusThread()  # never started, set_target() only stores the target
    for velocity_limit, accel_limit in ((math.nan, 5.0), (0.5, math.nan), (0.0, 5.0), (0.5, -1.0)):
        assert thread.set_target(ControlMode.COMPRESSIONS, 3.0, 3.0, 0.03,
                                 compressionVelocityLimit=velocity_limit,
                                 compressionAccelLimit=accel_limit) == ErrorCode.ERROR_MOTOR_FAILURE


# -------------------- computeCompressionSetpoint_streamed_deprecated (pure) --------------------

def test_streamed_setpoint_stroke_profile(hardware, monkeypatch):
    """Down stroke then up stroke, each a thirds trapezoid (quarter, half, quarter of the depth), repeating,
    with the matching velocity: zero at each end, the cruise speed in the middle, negative on the way up."""
    import actuation

    fake_time = {"t": 0.0}
    monkeypatch.setattr(actuation.time, "monotonic", lambda: fake_time["t"])
    actuation.init_compressions()  # compression_start_time = 0.0
    depth = actuation.COMPRESSION_DEPTH_CM / 100.0
    stroke = actuation.STROKE_SEC
    speed = actuation.COMPRESSION_SPEED_MPS

    expected = [
        (0.0, 0.0, 0.0),                            # top, at rest, starting the down stroke
        (stroke / 3.0, 0.25 * depth, speed),        # end of speeding up
        (stroke / 2.0, 0.5 * depth, speed),         # middle of the cruise
        (2.0 * stroke / 3.0, 0.75 * depth, speed),  # start of slowing down
        (stroke, depth, 0.0),                       # bottom, at rest, starting the up stroke
        (1.5 * stroke, 0.5 * depth, -speed),        # middle of the up stroke
        (2.0 * stroke, 0.0, 0.0),                   # back at the top, cycle repeats
        (2.5 * stroke, 0.5 * depth, speed),         # middle of the next down stroke
    ]
    for t, position, velocity in expected:
        fake_time["t"] = t
        actual_position, actual_velocity = actuation.computeCompressionSetpoint_streamed_deprecated()
        assert actual_position == pytest.approx(position, abs=1e-9), t
        assert actual_velocity == pytest.approx(velocity, abs=1e-9), t


def test_streamed_setpoint_is_continuous_and_velocity_matches(hardware, monkeypatch):
    """No jumps anywhere in the cycle, never outside 0 to depth, and the velocity matches how fast the position changes."""
    import actuation

    fake_time = {"t": 0.0}
    monkeypatch.setattr(actuation.time, "monotonic", lambda: fake_time["t"])
    actuation.init_compressions()
    depth = actuation.COMPRESSION_DEPTH_CM / 100.0
    dt = 1e-5
    # Fastest possible change per step is the cruise speed
    max_step = actuation.COMPRESSION_SPEED_MPS * dt * 1.01
    # Velocity changes by at most COMPRESSION_ACCEL_MPS2 * dt within a step, so the finite difference is that close to it
    velocity_tolerance = actuation.COMPRESSION_ACCEL_MPS2 * dt * 1.01

    previous, _ = actuation.computeCompressionSetpoint_streamed_deprecated()
    for i in range(1, int(2.0 * actuation.STROKE_SEC / dt) + 2):
        fake_time["t"] = i * dt
        position, velocity = actuation.computeCompressionSetpoint_streamed_deprecated()
        assert -1e-12 <= position <= depth + 1e-12
        assert abs(position - previous) <= max_step
        assert abs((position - previous) / dt - velocity) <= velocity_tolerance
        previous = position


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


def test_compressions_sends_stroke_extreme_with_caps(hardware):
    """compressions() commands the bottom of the first stroke with velocity 0 and both speed and acceleration caps set."""
    import time
    import actuation
    import sensing
    from moteus_thread import BLDC_ROTATIONS_PER_M
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    time.sleep(0.05)  # let the background command loop complete its first tick

    actuation.init_compressions()
    assert actuation.compressions() == ErrorCode.NORMAL_OPERATION
    target = actuation.get_motor_controller()._target
    bottom = sensing.get_rotary_zero_position() + actuation.COMPRESSION_DEPTH_CM / 100.0 * BLDC_ROTATIONS_PER_M
    assert target.position == pytest.approx(bottom)
    assert target.velocity == 0.0
    assert target.velocity_limit == pytest.approx(actuation.COMPRESSION_SPEED_MPS * abs(BLDC_ROTATIONS_PER_M))
    assert target.accel_limit == pytest.approx(actuation.COMPRESSION_ACCEL_MPS2 * abs(BLDC_ROTATIONS_PER_M))


def test_compressions_streamed_deprecated_sends_position_and_velocity(hardware):
    """The deprecated streamed mode still runs, sending a position and velocity with no speed or acceleration caps."""
    import math
    import time
    import actuation
    import sensing
    actuation.init_motor()
    init_sensing(sensing, actuation.get_motor_controller(), hardware)
    time.sleep(0.05)  # let the background command loop complete its first tick

    actuation.init_compressions()
    assert actuation.compressions_streamed_deprecated() == ErrorCode.NORMAL_OPERATION
    target = actuation.get_motor_controller()._target
    assert not math.isnan(target.position)
    assert not math.isnan(target.velocity)
    assert math.isnan(target.velocity_limit)
    assert math.isnan(target.accel_limit)


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
