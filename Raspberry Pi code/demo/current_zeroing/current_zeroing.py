# current_zeroing.py (demo)
"""
Current-spike zeroing test. Drives the plunger down the ball screw at a constant velocity with the moteus, and stops when
the q-axis (torque-producing) current spikes above its free-running baseline, or when the extension limit is reached.

Uses src/moteus_thread.py unaltered for all moteus communication and its ball screw constants for unit conversion.
The lead screw steppers and the sensor suite are not touched, so this runs without pigpio, I2C, or the force sensor.

Sequence:
    1. Record the startup motor position as the top of ball screw travel (same as sensing.py)
    2. Descend at DESCENT_VELOCITY_MPS. Ignore the current for SETTLE_TIME_S while the motor accelerates, then average
       the current for BASELINE_TIME_S to get the free-running baseline
    3. Stop when |q_current| exceeds the baseline by SPIKE_THRESHOLD_A for SPIKE_CONSECUTIVE_SAMPLES samples in a row
       (chest found), or when the plunger reaches EXTENSION_LIMIT_M or DESCENT_TIMEOUT_S passes (not found)
    4. Hold for HOLD_TIME_S, then retract to the startup position

Log messages go to the console and Logs/current_zeroing_<timestamp>.log, same setup as src/main.py. Every motor sample
is written to a matching .csv in Logs/ so the thresholds can be tuned from real data.
Ctrl+C stops the motor at any point.

Usage (from the repo root, on the Pi):
    python demo/current_zeroing/current_zeroing.py
"""

# External imports
import sys
import csv
import math
import time
import logging
from datetime import datetime
from pathlib import Path
from typing import TextIO, Callable

REPO_DIR: Path = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_DIR / "src"))

# Internal imports (from src/, unaltered)
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode
from moteus_thread import (MoteusThread, MotorTarget, MotorState, CONTROLLER_ID, BLDC_ROTATIONS_PER_M,
                           BALL_SCREW_LEAD_M, BLDC_GEAR_RATIO, BALL_SCREW_TRAVEL_M)

# --- Tunable constants (all TODO: tune on hardware) ---------------------------------------------------------
DESCENT_VELOCITY_MPS: float = 0.005  # meters per second, plunger speed down the ball screw while searching
# Force cap while descending. Kept well under moteus_thread.MAX_PLUNGER_FORCE_N, since contact is what we're looking for.
# The q current saturates at the torque limit, so the spike threshold must stay below the current this cap allows
DESCENT_MAX_FORCE_N: float = 50.0
# Stop short of the end of travel, since the plunger coasts a little past the limit after the stop is commanded
EXTENSION_LIMIT_M: float = BALL_SCREW_TRAVEL_M - 0.005  # meters below the startup position
DESCENT_TIMEOUT_S: float = 1.5 * EXTENSION_LIMIT_M / DESCENT_VELOCITY_MPS  # 1.5x buffer for the accel ramp

SETTLE_TIME_S: float = 0.5  # seconds after the descent starts before the current is trusted (acceleration transient)
BASELINE_TIME_S: float = 0.5  # seconds of free-running current averaged into the baseline, after SETTLE_TIME_S
SPIKE_THRESHOLD_A: float = 1.0  # amps above the baseline |q_current| that counts as contact
SPIKE_CONSECUTIVE_SAMPLES: int = 3  # samples in a row above threshold before stopping, rejects single-sample noise

HOLD_TIME_S: float = 2.0  # seconds to hold at the stop position before retracting
RETRACT_TOLERANCE_M: float = 0.001  # meters from the startup position that counts as retracted
RETRACT_TIMEOUT_S: float = 10.0  # seconds before giving up on the retract

LOOP_PERIOD_S: float = 0.002  # main loop tick, faster than moteus_thread.COMMAND_RATE_HZ so no sample is missed
STARTUP_TIMEOUT_S: float = 2.0  # seconds to wait for the first reply from the moteus

# Same log location and naming as src/main.py. The CSV shares the run timestamp so the two files pair up
LOG_DIR: Path = REPO_DIR / "Logs"
RUN_TIMESTAMP: str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE: Path = LOG_DIR / f"current_zeroing_{RUN_TIMESTAMP}.log"
CSV_FILE: Path = LOG_DIR / f"current_zeroing_{RUN_TIMESTAMP}.csv"

# Same ideal ball screw conversion as moteus_thread.TORQUE_LIMIT_NM
DESCENT_MAX_TORQUE_NM: float = DESCENT_MAX_FORCE_N * BALL_SCREW_LEAD_M / (2 * math.pi * abs(BLDC_GEAR_RATIO))

# --- Module state -------------------------------------------------------------------------------------------
_motor: MoteusThread
_abs_zero_position: float = 0.0  # motor rotations at startup, top of ball screw travel
_descent_start_time: float = 0.0  # time.monotonic() when the descent started
_last_sample_time: float = 0.0  # MotorState.timestamp of the last sample processed, so each sample is only used once
_baseline_sum_a: float = 0.0
_baseline_count: int = 0
_spike_count: int = 0  # consecutive samples above threshold
_csv_file: TextIO


def configure_logging() -> None:
    """Configure console and file logging, same format and level as src/main.py."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)  # Ensure logs directory exists
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),            # console
            logging.FileHandler(LOG_FILE),      # log file
        ]
    )


def init() -> ErrorCode:
    """Start the moteus thread, wait for its first reply, and record the startup position.

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_INIT_FAILURE otherwise
    """
    global _motor, _abs_zero_position, _csv_file

    _motor = MoteusThread(controller_id=CONTROLLER_ID)
    if _motor.start() != ErrorCode.NORMAL_OPERATION:
        logging.error("Moteus thread failed to start")
        return ErrorCode.ERROR_INIT_FAILURE

    # Hold still while waiting, so the first command sent isn't a leftover target
    _motor.set_target(ControlMode.HOLD_POSITION, 0.0, 0.0)
    deadline: float = time.monotonic() + STARTUP_TIMEOUT_S
    while _motor.get_state().stale:
        if time.monotonic() > deadline:
            logging.error(f"No reply from moteus: {_motor.get_last_error()}")
            return ErrorCode.ERROR_INIT_FAILURE
        time.sleep(LOOP_PERIOD_S)

    state: MotorState = _motor.get_state()
    if state.mode == 1:
        logging.error(f"Moteus is faulted (fault {state.fault}), clear it before running")
        return ErrorCode.ERROR_INIT_FAILURE
    _abs_zero_position = state.position

    _csv_file = open(CSV_FILE, "w", newline="")
    csv.writer(_csv_file).writerow(["phase", "time_s", "travel_m", "velocity_mps", "q_current_a", "baseline_a", "mode", "fault"])

    logging.info(f"Startup position {_abs_zero_position:.3f} rev, logging to {LOG_FILE} and {CSV_FILE}")
    logging.info(f"Descending at {DESCENT_VELOCITY_MPS} m/s, limit {EXTENSION_LIMIT_M} m, "
                 f"torque cap {DESCENT_MAX_TORQUE_NM:.3f} Nm, spike threshold {SPIKE_THRESHOLD_A} A over baseline")
    return ErrorCode.NORMAL_OPERATION


def get_travel_m(state: MotorState) -> float:
    """Plunger travel below the startup position, in meters, positive down"""
    return (state.position - _abs_zero_position) / BLDC_ROTATIONS_PER_M


def log_sample(phase: str, state: MotorState, baseline_a: float) -> None:
    """Write one motor sample to the CSV. Flushed every row, so a power loss or hard kill keeps the samples around contact"""
    csv.writer(_csv_file).writerow([phase, f"{state.timestamp - _descent_start_time:.4f}", f"{get_travel_m(state):.5f}",
                          f"{state.velocity / BLDC_ROTATIONS_PER_M:.5f}", f"{state.q_current:.4f}",
                          f"{baseline_a:.4f}", state.mode, state.fault])
    _csv_file.flush()


def init_descent() -> None:
    """Start the constant velocity descent. Non-blocking."""
    global _descent_start_time, _last_sample_time

    _descent_start_time = time.monotonic()
    _last_sample_time = _motor.get_state().timestamp

    # MoteusThread.set_target() has no constant velocity mode, and src/ must stay unaltered, so the target is written
    # directly. Position NaN puts the moteus in pure velocity control. The thread sends this every tick until replaced
    setattr(_motor, "_target", MotorTarget(
        position=math.nan,
        velocity=DESCENT_VELOCITY_MPS * BLDC_ROTATIONS_PER_M,  # rotations per second, signed by the gear train
        kp_scale=1.0,
        kd_scale=1.0,
        feedforward_torque=0.0,
        velocity_limit=math.nan,
        maximum_torque=DESCENT_MAX_TORQUE_NM,
    ))


def update_descent() -> ErrorCode:
    """Check the newest motor sample for a current spike or the extension limit. Non-blocking, call every tick.

    Returns:
        ErrorCode: NORMAL_OPERATION while descending, ZEROING_FINISHED on a current spike,
                   ERROR_ZEROING_FAILURE at the extension limit or timeout, ERROR_MOTOR_FAILURE on a moteus error
    """
    global _last_sample_time, _baseline_sum_a, _baseline_count, _spike_count

    elapsed_s: float = time.monotonic() - _descent_start_time
    if elapsed_s > DESCENT_TIMEOUT_S:
        logging.error(f"Timed out after {elapsed_s:.1f} s without a current spike")
        return ErrorCode.ERROR_ZEROING_FAILURE

    if _motor.get_last_error() != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Moteus communication error: {_motor.get_last_error()}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    # The moteus thread only updates at COMMAND_RATE_HZ, skip ticks with no new sample
    state: MotorState = _motor.get_state()
    if state.timestamp == _last_sample_time:
        return ErrorCode.NORMAL_OPERATION
    _last_sample_time = state.timestamp

    if state.mode == 1:
        logging.error(f"Moteus fault {state.fault}")
        return ErrorCode.ERROR_MOTOR_FAILURE

    baseline_a: float = _baseline_sum_a / _baseline_count if _baseline_count > 0 else math.nan
    log_sample("descent", state, baseline_a)

    travel_m: float = get_travel_m(state)
    if travel_m >= EXTENSION_LIMIT_M:
        logging.error(f"Extension limit reached at {travel_m:.4f} m without a current spike")
        return ErrorCode.ERROR_ZEROING_FAILURE

    sample_time_s: float = state.timestamp - _descent_start_time
    current_a: float = abs(state.q_current)  # sign depends on motor direction, only the magnitude matters

    # Acceleration transient, current isn't meaningful yet
    if sample_time_s < SETTLE_TIME_S:
        return ErrorCode.NORMAL_OPERATION

    # Average the free-running current into the baseline
    if sample_time_s < SETTLE_TIME_S + BASELINE_TIME_S:
        _baseline_sum_a += current_a
        _baseline_count += 1
        return ErrorCode.NORMAL_OPERATION

    if _baseline_count == 0:
        logging.error("No samples collected for the current baseline")
        return ErrorCode.ERROR_MOTOR_FAILURE

    if current_a - baseline_a > SPIKE_THRESHOLD_A:
        _spike_count += 1
    else:
        _spike_count = 0

    if _spike_count >= SPIKE_CONSECUTIVE_SAMPLES:
        logging.info(f"Current spike at {travel_m:.4f} m: {current_a:.2f} A vs baseline {baseline_a:.2f} A")
        return ErrorCode.ZEROING_FINISHED

    return ErrorCode.NORMAL_OPERATION


def hold() -> ErrorCode:
    """Hold the plunger where it is. Non-blocking, call every tick."""
    log_new_sample("hold")
    return _motor.set_target(ControlMode.HOLD_POSITION, _abs_zero_position, _abs_zero_position)


def retract() -> ErrorCode:
    """Retract the plunger to the startup position. Non-blocking, call every tick.

    Returns:
        ErrorCode: NORMAL_OPERATION while retracting, ZEROING_FINISHED once retracted, ERROR_MOTOR_FAILURE on failure
    """
    log_new_sample("retract")
    error: ErrorCode = _motor.set_target(ControlMode.ABORT_RETRACT, _abs_zero_position, _abs_zero_position)
    if error != ErrorCode.NORMAL_OPERATION:
        return error
    if abs(get_travel_m(_motor.get_state())) < RETRACT_TOLERANCE_M:
        return ErrorCode.ZEROING_FINISHED
    return ErrorCode.NORMAL_OPERATION


def log_new_sample(phase: str) -> None:
    """Log the newest motor sample, if one has arrived since the last call"""
    global _last_sample_time
    state: MotorState = _motor.get_state()
    if state.timestamp != _last_sample_time:
        _last_sample_time = state.timestamp
        baseline_a: float = _baseline_sum_a / _baseline_count if _baseline_count > 0 else math.nan
        log_sample(phase, state, baseline_a)


def run_for(phase_fn: Callable[[], ErrorCode], duration_s: float) -> ErrorCode:
    """Call phase_fn (hold or retract) every tick until it finishes, fails, or duration_s passes.

    Returns:
        ErrorCode: ZEROING_FINISHED if the phase finished, NORMAL_OPERATION if duration_s passed, otherwise the failure
    """
    end_time: float = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        error: ErrorCode = phase_fn()
        if error != ErrorCode.NORMAL_OPERATION:
            return error
        time.sleep(LOOP_PERIOD_S)
    return ErrorCode.NORMAL_OPERATION


def main() -> None:
    configure_logging()

    if init() != ErrorCode.NORMAL_OPERATION:
        sys.exit(1)

    try:
        input("Press Enter to start the descent (Ctrl+C stops the motor at any time)...")
        init_descent()
        result: ErrorCode = ErrorCode.NORMAL_OPERATION
        while result == ErrorCode.NORMAL_OPERATION:
            result = update_descent()
            time.sleep(LOOP_PERIOD_S)

        if result == ErrorCode.ERROR_MOTOR_FAILURE:
            return

        if result == ErrorCode.ZEROING_FINISHED:
            logging.info("Chest detected")
        logging.info(f"Holding for {HOLD_TIME_S} s")
        if run_for(hold, HOLD_TIME_S) != ErrorCode.NORMAL_OPERATION:
            logging.error("Hold failed")
            return

        logging.info("Retracting")
        if run_for(retract, RETRACT_TIMEOUT_S) == ErrorCode.ZEROING_FINISHED:
            logging.info("Retracted to the startup position")
        else:
            logging.error(f"Retract did not finish within {RETRACT_TIMEOUT_S} s")
    except KeyboardInterrupt:
        logging.info("Interrupted")
    finally:
        # Command zero velocity, then stop sending. The moteus watchdog stops the motor once commands stop
        _motor.set_target(ControlMode.HOLD_POSITION, _abs_zero_position, _abs_zero_position)
        time.sleep(0.05)
        _motor.stop()
        _csv_file.close()


if __name__ == "__main__":
    main()
