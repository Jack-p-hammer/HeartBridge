# jog_compressions.py (demo)
"""
Bench-test compressions for the demo, modeled on the cycling stage of scripts/slow_jog.py: from wherever the plunger
is when COMPRESSION starts, move down STROKE_IN inches and back up at a constant SPEED_IN_PER_S, over and over.

Used by demo/main.py in place of actuation.compressions(). There is no CPR waveform and no dependence on zeroing, and
the torque is capped low like slow_jog. Each stroke is planned by the moteus itself: it is sent the stroke's end
position with velocity 0 and a velocity_limit, so the motion stays smooth even though main.py only updates every
LOOP_TICK_SEC. Pause and abort still go through actuation.pause_compressions() / abort_compressions().
"""
# External imports
import logging
import time

# Internal imports (sensing resolves to demo/sensing.py, everything else to src/)
import sensing
import stepper
from Enums.error_codes import ErrorCode
from moteus_thread import MoteusThread, MotorTarget, BLDC_ROTATIONS_PER_M, BALL_SCREW_TRAVEL_M

STROKE_IN: float = 2.0  # inches down from the starting position, then back up
SPEED_IN_PER_S: float = 1.0  # constant plunger speed in both directions. slow_jog.py defaults to 0.1
MAX_TORQUE_NM: float = 0.4  # bench limit, same as slow_jog.py (the full build allows TORQUE_LIMIT_NM)
START_TOLERANCE_M: float = 0.005  # how far above the startup (absolute zero) position the start may read, for encoder noise

M_PER_IN: float = 0.0254
# Motor rotations per inch of plunger travel, signed like BLDC_ROTATIONS_PER_M (negative: the gear train reverses rotation)
ROTATIONS_PER_IN: float = M_PER_IN * BLDC_ROTATIONS_PER_M
SPEED_RPS: float = abs(SPEED_IN_PER_S * ROTATIONS_PER_IN)  # velocity_limit is a magnitude
STROKE_SEC: float = STROKE_IN / SPEED_IN_PER_S  # time for one stroke in one direction

_top_rev: float = 0.0  # motor position the cycle starts from and returns to, in rotations
_bottom_rev: float = 0.0  # motor position STROKE_IN below _top_rev, in rotations
_cycle_start: float = 0.0  # time.monotonic() when the current run of cycling started
_going_down: bool | None = None  # direction of the current stroke, None before the first one, for logging


def init_jog_compressions(motor_controller: MoteusThread) -> ErrorCode:
    """Record where the plunger is now as the top of the stroke and start the cycle with a down stroke.
    Call once each time COMPRESSION is entered, so resuming after a pause starts from wherever the plunger is then.

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance

    Returns:
        ErrorCode: NORMAL_OPERATION, or ERROR_MOTOR_FAILURE if the moteus hasn't replied yet or the stroke would leave
            the ball screw travel
    """
    global _top_rev, _bottom_rev, _cycle_start, _going_down

    state = motor_controller.get_state()
    if state.stale:
        logging.error("Jog compressions: no reply from the moteus yet, plunger position unknown")
        return ErrorCode.ERROR_MOTOR_FAILURE

    top_rev: float = state.position
    bottom_rev: float = top_rev + STROKE_IN * ROTATIONS_PER_IN

    # Never command the plunger past either end of the ball screw (set_raw_target() doesn't check this)
    top_travel_m: float = (top_rev - sensing.get_rotary_absolute_zero_position()) / BLDC_ROTATIONS_PER_M
    bottom_travel_m: float = top_travel_m + STROKE_IN * M_PER_IN
    if top_travel_m < -START_TOLERANCE_M or bottom_travel_m > BALL_SCREW_TRAVEL_M:
        logging.error(f"Jog compressions: {STROKE_IN} in stroke from {top_travel_m:.4f} m would leave the ball screw "
                      f"travel (0 to {BALL_SCREW_TRAVEL_M} m)")
        return ErrorCode.ERROR_MOTOR_FAILURE

    _top_rev, _bottom_rev = top_rev, bottom_rev
    _cycle_start = time.monotonic()
    _going_down = None
    logging.info(f"Jog compressions: cycling {STROKE_IN} in down and up at {SPEED_IN_PER_S} in/s, "
                 f"between {_top_rev:.3f} and {_bottom_rev:.3f} rev")
    return ErrorCode.NORMAL_OPERATION


def jog_compressions(motor_controller: MoteusThread) -> ErrorCode:
    """Keep the stroke going. Non-blocking, call every tick while in COMPRESSION, after init_jog_compressions().

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance

    Returns:
        ErrorCode: NORMAL_OPERATION while cycling, ERROR_MOTOR_FAILURE on a moteus or stepper failure
    """
    global _going_down

    error = motor_controller.get_last_error()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Jog compressions failed on motor: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # Same as actuation.compressions(): the carriage holds, but play out any steps still queued
    error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Jog compressions failed on stepper: {error}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # First half of each cycle heads for the bottom, second half back to the top. The moteus plans each stroke at
    # SPEED_RPS, so it arrives just as the target flips, the same constant-speed triangle as slow_jog.py
    going_down: bool = (time.monotonic() - _cycle_start) % (2.0 * STROKE_SEC) < STROKE_SEC
    if going_down != _going_down:
        logging.debug(f"Jog compressions: stroke {'down' if going_down else 'up'}")
        _going_down = going_down

    motor_controller.set_raw_target(MotorTarget(
        position=_bottom_rev if going_down else _top_rev,
        velocity=0.0,  # stop at the end of the stroke
        velocity_limit=SPEED_RPS,
        maximum_torque=MAX_TORQUE_NM,
    ))
    return ErrorCode.NORMAL_OPERATION
