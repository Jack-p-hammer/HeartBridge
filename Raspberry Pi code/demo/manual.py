# manual.py (demo)
"""
Manual control for the demo, in case the hardcoded positions drift: drive the carriage (steppers) and the plunger
(moteus) up and down from the keyboard. Used by demo/main.py's MANUAL state, entered with M from WAIT or PAUSE.

Keys (letters only: in the terminal, arrow keys arrive as Esc sequences, and Esc exits the demo):
    W / S   carriage (steppers) up / down
    I / K   plunger (moteus) up / down
    Z       use where the carriage is as the zeroing position, for every zeroing until the demo exits
    M       leave manual mode (handled in main.py, which calls exit_manual())

Each key press moves a fixed step. A terminal (e.g. over SSH) sends no key releases, only repeats while a key is held,
so holding a key keeps it moving. After letting go, the plunger stops within about a step, and the carriage within
about 2 mm (a step plus the steps already queued to the drivers).
Neither moves past either end of its travel. Positions are in meters, positive down, like the rest of the code.
"""
# External imports
import logging

import pygame

# Internal imports (sensing and actuation resolve to demo/, everything else to src/)
import actuation
import sensing
import stepper
from Enums.error_codes import ErrorCode
from moteus_thread import MoteusThread, MotorTarget, BLDC_ROTATIONS_PER_M, BALL_SCREW_TRAVEL_M, TORQUE_LIMIT_NM

CARRIAGE_UP_KEY: int = pygame.K_w
CARRIAGE_DOWN_KEY: int = pygame.K_s
PLUNGER_UP_KEY: int = pygame.K_i
PLUNGER_DOWN_KEY: int = pygame.K_k
SAVE_ZERO_KEY: int = pygame.K_z

CARRIAGE_STEP_M: float = 0.001  # carriage travel per key press
CARRIAGE_VELOCITY_MPS: float = stepper.ZEROING_VELOCITY_MPS  # speed cap, same as zeroing
PLUNGER_STEP_M: float = 0.001  # plunger travel per key press
PLUNGER_VELOCITY_MPS: float = 0.02  # speed cap, a quarter of the moteus's retract speed
PLUNGER_ACCEL_MPS2: float = 0.5  # acceleration cap, reaches the speed cap in 40 ms

_plunger_target_rev: float = 0.0  # motor position the plunger is held at, in rotations
_carriage_moving: bool = False  # whether the carriage was moving last tick, to log where it stops


def init_manual(motor_controller: MoteusThread) -> ErrorCode:
    """Start manual mode: stop the carriage where its queued steps end and hold the plunger where it is.
    Call once each time MANUAL is entered.

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance

    Returns:
        ErrorCode: NORMAL_OPERATION, or ERROR_MOTOR_FAILURE if the moteus hasn't replied yet (plunger position unknown)
    """
    global _plunger_target_rev, _carriage_moving

    state = motor_controller.get_state()
    if state.stale:
        logging.error("Manual mode: no reply from the moteus yet, plunger position unknown")
        return ErrorCode.ERROR_MOTOR_FAILURE

    stepper.halt()
    _carriage_moving = False
    _plunger_target_rev = state.position
    _send_plunger_target(motor_controller)

    logging.info(f"Manual mode: carriage at {stepper.get_setpoint() * 1000:.1f} mm, plunger at "
                 f"{_plunger_travel_m(state.position) * 1000:.1f} mm, zeroing position "
                 f"{actuation.get_zero_position() * 1000:.1f} mm")
    logging.info("Manual mode keys: W/S carriage up/down, I/K plunger up/down, Z save carriage position as zeroing "
                 "position, M leave manual mode")
    return ErrorCode.NORMAL_OPERATION


def manual(motor_controller: MoteusThread, keys: list[int]) -> ErrorCode:
    """Act on the keys pressed this tick and keep the carriage moving. Non-blocking, call every tick while in MANUAL,
    after init_manual(). The plunger needs nothing between key presses: the moteus thread keeps sending its target.

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance
        keys (list[int]): pygame key codes pressed this tick

    Returns:
        ErrorCode: NORMAL_OPERATION, or ERROR_MOTOR_FAILURE on a stepper failure
    """
    global _carriage_moving

    for key in keys:
        if key == CARRIAGE_UP_KEY:
            _jog_carriage(-1)
        elif key == CARRIAGE_DOWN_KEY:
            _jog_carriage(1)
        elif key == PLUNGER_UP_KEY:
            _jog_plunger(motor_controller, -1)
        elif key == PLUNGER_DOWN_KEY:
            _jog_plunger(motor_controller, 1)
        elif key == SAVE_ZERO_KEY:
            # Save where the carriage stops: halting ends the move where its queued steps end. At the bottom of travel,
            # rounding to a whole step can leave the setpoint a fraction of a step past LEAD_SCREW_TRAVEL_M, which
            # set_target() (and so zeroing) would reject; the clamped value rounds to the same step
            stepper.halt()
            actuation.set_zero_position(min(stepper.get_setpoint(), stepper.LEAD_SCREW_TRAVEL_M))

    # The steppers only move while being updated
    error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Manual mode failed on stepper: {error}")
        stepper.halt()
        return ErrorCode.ERROR_MOTOR_FAILURE
    # Moving up as well as down, so keep the extended step count matching the position. Then ABORT and RESET_DEMO
    # (actuation.retract_carriage()) bring the carriage home from wherever manual mode leaves it, at any time
    stepper.sync_extended_steps()

    if _carriage_moving and stepper.at_target():
        logging.info(f"Manual mode: carriage stopped at {stepper.get_position() * 1000:.1f} mm "
                     f"({stepper.get_position()} m)")
    _carriage_moving = not stepper.at_target()

    return ErrorCode.NORMAL_OPERATION


def exit_manual(motor_controller: MoteusThread) -> None:
    """Leave manual mode: stop the carriage where its queued steps end. The plunger holds where it is until the next
    state commands it. Call when leaving MANUAL.

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance
    """
    stepper.halt()
    stepper.sync_extended_steps()
    logging.info(f"Leaving manual mode: carriage at {stepper.get_setpoint() * 1000:.1f} mm, plunger at "
                 f"{_plunger_travel_m(motor_controller.get_state().position) * 1000:.1f} mm, zeroing position "
                 f"{actuation.get_zero_position() * 1000:.1f} mm")


# -------- PRIVATE FUNCTIONS --------

def _jog_carriage(direction: int) -> None:
    """Move the carriage one step from where its queued steps end, within the lead screw travel.
    Measured from the queued end, not get_position(), so the target is never behind steps already queued in the same
    direction (a target behind them would make it stop and come back). Holding a key moves it continuously, and it
    runs on at most a step plus the queued steps (about 2 mm at full speed) after letting go.

    Args:
        direction (int): 1 down, -1 up
    """
    target_m: float = stepper.get_queued_end() + direction * CARRIAGE_STEP_M
    target_m = min(max(target_m, 0.0), stepper.LEAD_SCREW_TRAVEL_M)
    # Clamped above, so only fails on a bad velocity
    error = stepper.set_target(target_m, CARRIAGE_VELOCITY_MPS)
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Manual mode: carriage target {target_m} m rejected: {error}")
        return
    logging.debug(f"Manual mode: carriage {'down' if direction > 0 else 'up'} to {target_m * 1000:.1f} mm")


def _jog_plunger(motor_controller: MoteusThread, direction: int) -> None:
    """Move the plunger one step from where it is now, within the ball screw travel.
    Measured from the plunger's position, not the last target, so holding a key moves it continuously, it can't run on
    more than a step after letting go, and pushing against something can't build up more than a step of position error.

    Args:
        motor_controller (MoteusThread): The shared moteus controller instance
        direction (int): 1 down, -1 up
    """
    global _plunger_target_rev

    abs_zero_rev: float = sensing.get_rotary_absolute_zero_position()
    target_m: float = _plunger_travel_m(motor_controller.get_state().position) + direction * PLUNGER_STEP_M
    target_m = min(max(target_m, 0.0), BALL_SCREW_TRAVEL_M)
    _plunger_target_rev = abs_zero_rev + target_m * BLDC_ROTATIONS_PER_M
    _send_plunger_target(motor_controller)
    logging.debug(f"Manual mode: plunger {'down' if direction > 0 else 'up'} to {target_m * 1000:.1f} mm")


def _plunger_travel_m(position_rev: float) -> float:
    """Returns a motor position as plunger travel below the top of the ball screw (absolute zero), in meters"""
    return (position_rev - sensing.get_rotary_absolute_zero_position()) / BLDC_ROTATIONS_PER_M + 0.0  # + 0.0 turns -0.0 into 0.0 for logs


def _send_plunger_target(motor_controller: MoteusThread) -> None:
    """Command the plunger to _plunger_target_rev at the manual speed and acceleration caps, stopping there.
    set_raw_target() doesn't check the ball screw travel, so callers keep the target inside it"""
    motor_controller.set_raw_target(MotorTarget(
        position=_plunger_target_rev,
        velocity=0.0,  # stop at the target
        velocity_limit=PLUNGER_VELOCITY_MPS * abs(BLDC_ROTATIONS_PER_M),  # magnitudes, in rotations
        accel_limit=PLUNGER_ACCEL_MPS2 * abs(BLDC_ROTATIONS_PER_M),
        maximum_torque=TORQUE_LIMIT_NM,
    ))
