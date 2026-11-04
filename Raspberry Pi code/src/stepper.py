# stepper.py
"""Functions that drive the two lead-screw steppers that raise and lower the plunger carriage (the BLDC + ball screw assembly). These functions are non-blocking.

Both stepper drivers share a single STEP/DIR input, so they are commanded as one axis.

update() must be called once per main loop tick (actuation.py does this). Each call tops up a short queue of step chunks as pigpio waveforms, which the pigpio daemon plays back to back by DMA in the background, and returns immediately. Steps are only counted once their waveform has finished, so the commanded position is always known exactly in steps.

While chunks are playing, the carriage is somewhere between get_position() and get_setpoint(), since queued steps never go past the setpoint. That range is what the lead screw encoders are validated against (see sensing.check_lead_screw_position()).

NOTE: ALL PUBLIC POSITIONS ARE IN METERS OF CARRIAGE TRAVEL FROM THE STARTUP (HOME) POSITION, POSITIVE IS DOWN (TOWARDS THE PATIENT), MATCHING THE BLDC CONVENTION
"""
# External imports
import pigpio
import math
import logging

# Internal imports
from Enums.error_codes import ErrorCode

# TODO: Set actual GPIO pin numbers to match the hardware hat
STEPPER_STEP_PIN: int = 20
STEPPER_DIR_PIN: int = 21
STEPPER_ENABLE_PIN: int = 16
STEPPER_ENABLE_ACTIVE_LOW: bool = True  # Most stepper drivers (A4988, DRV8825, TMC2209) enable on a LOW EN pin
STEPPER_DIR_DOWN_LEVEL: int = 1  # DIR pin level that drives the carriage down. TODO: Verify against wiring

# --- Mechanical constants (PLACEHOLDERS, TODO: replace with real hardware values) ---
LEAD_SCREW_PITCH_M: float = 0.008  # meters of carriage travel per lead screw revolution (the lead; equals pitch only for single-start screws)
STEPPER_GEAR_RATIO: float = 1.0  # stepper revolutions per lead screw revolution, 1:1 gear train
STEPPER_FULL_STEPS_PER_REV: int = 200  # 1.8 degree steppers
STEPPER_MICROSTEPS: int = 8  # Microstepping set on the drivers
STEPS_PER_M: float = STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO / LEAD_SCREW_PITCH_M

LEAD_SCREW_TRAVEL_M: float = 0.0254 * 8  # meters, usable carriage travel from home (startup) position, 8" placeholder

# --- Motion constants ---
CHUNK_DURATION_S: float = 0.04  # Duration of each queued chunk of steps
CHUNK_QUEUE_DEPTH: int = 2  # Chunks kept queued, CHUNK_DURATION_S * CHUNK_QUEUE_DEPTH must exceed main.py LOOP_TICK_SEC plus jitter or motion stutters
ZEROING_VELOCITY_MPS: float = 0.04  # meters per second, carriage velocity while seeking the chest during zeroing
RETRACT_VELOCITY_MPS: float = 2*ZEROING_VELOCITY_MPS  # meters per second, carriage velocity when retracting to home
STEPPER_ACCEL_MPS2: float = 0.2  # meters per second^2, ramp limit so the steppers don't stall. TODO: Tune
STEP_PULSE_HIGH_US: int = 5  # STEP high time, most drivers need 1-2us

# See sensing.py for lead screw encoder tolerance constants

# Global variables for the shared pigpio instance
_pi: pigpio.pi

# Global variables for stepper state
# Startup position is assumed to be home (fully retracted), same assumption as the BLDC absolute zero
position_steps: int = 0  # Steps that have actually been sent
target_steps: int = 0
velocity_limit_steps_per_s: float = 0.0
rate_steps_per_s: float = 0.0  # Signed step rate of the last chunk, used for the accel ramp
chunk_queue: list[tuple[int, int]] = []  # Chunks queued on the pigpio daemon in play order as (wave_id, signed steps)


def init_steppers(pi: pigpio.pi) -> ErrorCode:
    """Configure the stepper GPIO and enable the drivers.

    Args:
        pi (pigpio.pi): The shared pigpio instance from HMI.init_Pi()

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_INIT_FAILURE if failed
    """
    global _pi, position_steps, target_steps, rate_steps_per_s, chunk_queue
    logging.debug("Initializing steppers...")

    _pi = pi
    position_steps = 0
    target_steps = 0
    rate_steps_per_s = 0.0
    chunk_queue = []

    try:
        # STEP/DIR/EN as outputs, STEP low, DIR down
        _pi.set_mode(STEPPER_STEP_PIN, pigpio.OUTPUT)
        _pi.set_mode(STEPPER_DIR_PIN, pigpio.OUTPUT)
        _pi.set_mode(STEPPER_ENABLE_PIN, pigpio.OUTPUT)
        _pi.write(STEPPER_STEP_PIN, 0)
        _pi.write(STEPPER_DIR_PIN, STEPPER_DIR_DOWN_LEVEL)
        # Drivers stay enabled for the whole run: holding torque keeps the carriage from back-driving under compression load
        _pi.write(STEPPER_ENABLE_PIN, 0 if STEPPER_ENABLE_ACTIVE_LOW else 1)
        _pi.wave_clear()
    except Exception as e:
        logging.error(f"Stepper GPIO initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    logging.info("Steppers initialized")
    return ErrorCode.NORMAL_OPERATION


def update() -> ErrorCode:
    """Queue the next chunks of steps towards the target. Non-blocking, call once per main loop tick.

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_MOTOR_FAILURE on a pigpio failure
    """
    global target_steps, rate_steps_per_s, chunk_queue

    error: ErrorCode = retire_finished_chunks()
    if error != ErrorCode.NORMAL_OPERATION:
        return error

    while len(chunk_queue) < CHUNK_QUEUE_DEPTH and queued_end_steps() != target_steps:
        error = queue_next_chunk()
        if error != ErrorCode.NORMAL_OPERATION:
            return error

    if queued_end_steps() == target_steps:
        rate_steps_per_s = 0.0

    return ErrorCode.NORMAL_OPERATION


def set_target(position_m: float, velocity_limit_mps: float) -> ErrorCode:
    """Set the carriage position setpoint, picked up on the next update().

    Args:
        position_m (float): Target carriage position in meters from home, positive is down
        velocity_limit_mps (float): Maximum carriage speed in meters per second

    Returns:
        ErrorCode: ERROR_MOTOR_FAILURE if the target is invalid or outside the lead screw travel, NORMAL_OPERATION otherwise
    """
    global target_steps, velocity_limit_steps_per_s

    # Written so NaN fails both checks
    try:
        target_valid: bool = (0.0 <= position_m <= LEAD_SCREW_TRAVEL_M) and (velocity_limit_mps > 0.0)
    except Exception as e:
        # e.g. not a number
        logging.error(f"Stepper target could not be checked: {position_m} m at {velocity_limit_mps} m/s: {e}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    if not target_valid:
        logging.error(f"Invalid stepper target: {position_m} m at {velocity_limit_mps} m/s")
        return ErrorCode.ERROR_MOTOR_FAILURE

    target_steps = round(position_m * STEPS_PER_M)
    velocity_limit_steps_per_s = velocity_limit_mps * STEPS_PER_M
    return ErrorCode.NORMAL_OPERATION


def halt() -> None:
    """Stop where the already-queued chunks end (at most CHUNK_DURATION_S * CHUNK_QUEUE_DEPTH of travel away).
    NOTE: This stops without a deceleration ramp, which is fine at zeroing speed."""
    global target_steps, rate_steps_per_s
    target_steps = queued_end_steps()
    rate_steps_per_s = 0.0


def get_position() -> float:
    """Returns the current carriage position (steps finished playing), in meters from home, positive is down"""
    global position_steps
    return position_steps / STEPS_PER_M


def get_setpoint() -> float:
    """Returns the carriage position setpoint, in meters from home, positive is down"""
    global target_steps
    return target_steps / STEPS_PER_M


def at_target() -> bool:
    """Returns True once all steps to the target have been sent and played out"""
    global chunk_queue, position_steps, target_steps
    return not chunk_queue and position_steps == target_steps


# -------- PRIVATE FUNCTIONS --------

def queued_end_steps() -> int:
    """Returns the position the carriage will be at once every queued chunk has played"""
    global position_steps, chunk_queue
    return position_steps + sum(steps for _, steps in chunk_queue)


def retire_finished_chunks() -> ErrorCode:
    """Count every chunk that has finished playing into the position and free its waveform

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_MOTOR_FAILURE on a pigpio failure
    """
    global _pi, position_steps, chunk_queue
    if not chunk_queue:
        return ErrorCode.NORMAL_OPERATION

    try:
        playing_id: int = _pi.wave_tx_at()  # Returns a non-wave id when nothing is playing
    except Exception as e:
        logging.error(f"Failed to read the playing stepper waveform: {e}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    queued_ids: list[int] = [wave_id for wave_id, _ in chunk_queue]
    num_finished: int = queued_ids.index(playing_id) if playing_id in queued_ids else len(queued_ids)
    finished_chunks: list[tuple[int, int]] = chunk_queue[:num_finished]
    del chunk_queue[:num_finished]

    # Finished chunks have played, so count them before freeing their waveforms
    for wave_id, steps in finished_chunks:
        position_steps += steps

    for wave_id, _ in finished_chunks:
        try:
            _pi.wave_delete(wave_id)
        except Exception as e:
            logging.error(f"Failed to delete stepper waveform {wave_id}: {e}")
            return ErrorCode.ERROR_MOTOR_FAILURE

    return ErrorCode.NORMAL_OPERATION


def queue_next_chunk() -> ErrorCode:
    """Queue one chunk of steps towards the target, limited by velocity and acceleration. It starts as soon as the
    previous chunk ends. Only call when the queue does not already end at the target.

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_MOTOR_FAILURE on a pigpio failure
    """
    global _pi, target_steps, velocity_limit_steps_per_s, rate_steps_per_s, chunk_queue
    remaining_steps: int = target_steps - queued_end_steps()
    direction: int = 1 if remaining_steps > 0 else -1

    # Speed is limited by the velocity limit, the speed we can still stop from before the target, and how much we
    # can speed up this chunk. Reversing direction needs the current rate brought to zero first.
    accel_steps_per_s2: float = STEPPER_ACCEL_MPS2 * STEPS_PER_M
    prev_rate: float = max(0.0, rate_steps_per_s * direction)
    rate: float = min(
        velocity_limit_steps_per_s,
        math.sqrt(2 * accel_steps_per_s2 * abs(remaining_steps)),
        prev_rate + accel_steps_per_s2 * CHUNK_DURATION_S,
    )
    # Always take at least one step so the ramp can start from rest
    num_steps: int = min(abs(remaining_steps), max(1, int(rate * CHUNK_DURATION_S)))
    rate = max(rate, 1.0 / CHUNK_DURATION_S)
    step_period_us: int = max(2 * STEP_PULSE_HIGH_US, int(1e6 / rate))

    step_mask: int = 1 << STEPPER_STEP_PIN
    pulses: list = []
    for _ in range(num_steps):
        pulses.append(pigpio.pulse(step_mask, 0, STEP_PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, step_mask, step_period_us - STEP_PULSE_HIGH_US))

    try:
        _pi.write(STEPPER_DIR_PIN, STEPPER_DIR_DOWN_LEVEL if direction > 0 else 1 - STEPPER_DIR_DOWN_LEVEL)
        _pi.wave_add_generic(pulses)
        wave_id: int = _pi.wave_create()
    except Exception as e:
        logging.error(f"Failed to build stepper waveform: {e}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    if wave_id < 0:
        logging.error(f"pigpio wave_create failed: {wave_id}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    try:
        _pi.wave_send_using_mode(wave_id, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
    except Exception as e:
        logging.error(f"Failed to send stepper waveform {wave_id}: {e}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    chunk_queue.append((wave_id, direction * num_steps))
    rate_steps_per_s = direction * rate
    return ErrorCode.NORMAL_OPERATION
