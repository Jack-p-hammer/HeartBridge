# main.py

# External imports
import time
import sys
import logging
from datetime import datetime
from pathlib import Path

# Internal imports
import sensing
import actuation
import stepper
import HMI
from Enums.error_codes import ErrorCode
from Enums.states import CPRState

# TODO: Optimize loop speed
LOOP_TICK_SEC = 0.05  # 50ms per tick, 20 Hz

# Frames cycled during the unfold step, and how long each one stays up
UNFOLD_FRAMES = [HMI.Image.UNFOLD_1, HMI.Image.UNFOLD_2, HMI.Image.UNFOLD_3, HMI.Image.UNFOLD_4]
UNFOLD_FRAME_SEC = 0.5  # 2 frames per second

# Initialize log directory and file
# 2x parent to get to project root
LOG_DIR = Path(__file__).resolve().parent.parent / "Logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)  # Ensure logs directory exists
RUN_TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = LOG_DIR / f"heartBridge_{RUN_TIMESTAMP}.log"

# Maps error codes to the state they should force the machine into
ERROR_STATE_MAP = {
    ErrorCode.ERROR_INIT_FAILURE:           CPRState.ABORT,
    ErrorCode.ERROR_SENSOR_FAILURE:         CPRState.ABORT,
    ErrorCode.ERROR_IMU_KNEEL_FAILURE:      CPRState.KNEEL_FAILURE,
    ErrorCode.ERROR_ZEROING_FAILURE:        CPRState.ABORT,
    ErrorCode.ERROR_LOW_BATTERY:            CPRState.ABORT,
    ErrorCode.ERROR_MOTOR_FAILURE:          CPRState.ABORT,
    ErrorCode.ERROR_PYGAME_FAILURE:         CPRState.ABORT,
}

FATAL_ERRORS = {
    ErrorCode.EXIT_UNKNOWN, 
    ErrorCode.ERROR_PYGAME_INIT_FAILURE,
    ErrorCode.ERROR_PI_DAEMON_FAILURE,
    ErrorCode.ERROR_UNKNOWN_IMAGE,
}


def configure_logging():
    """Configure console and file logging for the CPR machine runtime."""
    # Write to both console and log file simultaneously.
    # Each line is prefixed with a timestamp and severity level, e.g.:
    #   2026-05-10 12:00:00 - INFO - Sensors initialized
    # Severity levels: DEBUG < WARNING < ERROR < CRITICAL
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),            # console
            logging.FileHandler(LOG_FILE),      # log file
        ]
    )


def main():
    """Run the main state-machine loop for the CPR machine."""
    configure_logging()

    # Start by defining initial state and error code. The state machine will update these as it runs.
    state = CPRState.STARTUP
    prev_state = None
    error = ErrorCode.NORMAL_OPERATION

    # Total time spent in COMPRESSION across all runs. Pausing (or a kneel failure) freezes it and
    # resuming carries on from the same total, so it reads total CPR time.
    cpr_elapsed_sec: float = 0.0
    compression_start: float | None = None  # time.monotonic() when the current run of compressions started, None while not compressing
    shown_sec: int | None = None  # Whole seconds currently drawn on the timer, None to force a redraw

    # Unfold animation position, reset when UNFOLD_CUT_CLOTHES is entered
    frame_idx: int = 0
    last_frame_time: float = 0.0  # time.monotonic() when the current frame was shown

    while error not in FATAL_ERRORS:
        # Record the start time of each loop iteration to maintain a consistent tick rate
        loop_start = time.monotonic()

        # If an error occurred, override the current state
        # Whenever an error occurs, system must be forced back to this point via continue
        if error in ERROR_STATE_MAP:
            logging.warning(f"Error occurred: {error}, forcing state to {ERROR_STATE_MAP[error]}")
            state = ERROR_STATE_MAP[error]
            error = ErrorCode.NORMAL_OPERATION
            
        # Must run every tick regardless of state (after HMI.init_HMI has run on
        # the first tick), or the OS can mark the window as unresponsive
        # Neither error code returned by pump_events will cause an actual error, so no check
        error = HMI.pump_events()
        

        # --- State machine ---
        # Each state checks (state != prev_state) to detect first entry, and runs
        # one-time setup (screen, audio, button enables) only on that first tick.
        # All other logic runs every tick until the exit condition is met.
        
        # Capture current state to avoid issues with state changing mid-loop
        current_state = state
        match(current_state):
            case(CPRState.STARTUP):
                # Setup
                if state != prev_state:
                    # continue jumps back to the top of the while loop, so if any
                    # init step fails its error code is caught on the next iteration
                    error = HMI.init_Pi()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = actuation.init_motor()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = actuation.init_steppers(HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.init_HMI(HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = sensing.init_sensors(actuation.get_motor_controller(), HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = sensing.battery_check()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    
                    error = HMI.set_image_audio(HMI.Image.STARTUP, HMI.AudioPrompt.STARTUP)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                if HMI.audio_finished():
                    state = CPRState.UNFOLD_CUT_CLOTHES

            case(CPRState.UNFOLD_CUT_CLOTHES):
                # Setup
                if state != prev_state:
                    frame_idx = 0
                    last_frame_time = time.monotonic()
                    error = HMI.set_image_audio(UNFOLD_FRAMES[frame_idx], HMI.AudioPrompt.UNFOLD)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                # Advance the animation. set_image only, so the audio prompt isn't restarted
                if time.monotonic() - last_frame_time >= UNFOLD_FRAME_SEC:
                    frame_idx = (frame_idx + 1) % len(UNFOLD_FRAMES)
                    last_frame_time = time.monotonic()
                    error = HMI.set_image(UNFOLD_FRAMES[frame_idx])
                    if error != ErrorCode.NORMAL_OPERATION: continue

                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.ALIGNMENT

            case(CPRState.ALIGNMENT):
                # Setup
                if state != prev_state:
                    error = HMI.enable_lasers()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.ALIGNMENT, HMI.AudioPrompt.ALIGNMENT)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    error = HMI.disable_lasers()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    state = CPRState.KNEELING

            case(CPRState.KNEELING):
                # Setup
                if state != prev_state:
                    error = HMI.set_image_audio(HMI.Image.KNEELING, HMI.AudioPrompt.KNEELING)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.ZEROING

            case(CPRState.ZEROING):
                # Setup
                if state != prev_state:
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.ZEROING, HMI.AudioPrompt.ZEROING)
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    error = actuation.init_zeroing()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                # Proceed with zeroing until it returns zeroing success code
                error = actuation.zeroing()
                if error not in (ErrorCode.NORMAL_OPERATION, ErrorCode.ZEROING_FINISHED): continue

                if error == ErrorCode.ZEROING_FINISHED:
                    error = sensing.zero_position()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    state = CPRState.COMPRESSION

            case(CPRState.COMPRESSION_PREP):
                # !!!!!!!!!! DEPRICATED !!!!!!!!!!!!!!!!
                # Setup
                if state != prev_state:
                    logging.warning("COMPRESSION_PREP state is deprecated and should not be used. why are we here?")
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    error = actuation.init_compressions()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    
                    error = HMI.set_image_audio(HMI.Image.COMPRESSION_PREP, HMI.AudioPrompt.COMPRESSION_PREP)
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                if HMI.audio_finished():
                    state = CPRState.COMPRESSION

            case(CPRState.COMPRESSION):
                # Setup
                if state != prev_state:
                    error = HMI.enable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.COMPRESSION, HMI.AudioPrompt.COMPRESSION)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    # Start of this run of compressions; the timer is drawn on the first loop tick
                    compression_start = time.monotonic()
                    shown_sec = None

                # Loop
                error = actuation.compressions()
                if error != ErrorCode.NORMAL_OPERATION: continue

                # Setup always sets this before the loop runs; this only narrows the type for the timer math below
                if compression_start is None:
                    compression_start = time.monotonic()

                # Only redraw when the displayed second changes, not every tick
                run_sec = time.monotonic() - compression_start
                total_sec = int(cpr_elapsed_sec + run_sec)
                if total_sec != shown_sec:
                    error = HMI.draw_timer(total_sec)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    shown_sec = total_sec

                error, pause_pressed = HMI.pause_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if pause_pressed:
                    # Bank this run into the total so the timer resumes from here
                    cpr_elapsed_sec += time.monotonic() - compression_start
                    compression_start = None
                    logging.info(f"CPR paused at {cpr_elapsed_sec:.1f} s total")
                    state = CPRState.PAUSE

            case(CPRState.PAUSE):
                # Setup
                if state != prev_state:
                    error = actuation.pause_compressions()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.PAUSE, HMI.AudioPrompt.PAUSE)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    
                    logging.debug("Device Paused")

                # Loop
                # TODO: Must track kneel failure during pause as well
                error = actuation.abort_compressions()
                if error != ErrorCode.NORMAL_OPERATION: continue
                
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.COMPRESSION

            case(CPRState.KNEEL_FAILURE):
                # Setup
                if state != prev_state:
                    logging.warning(f"Kneel failure tripped: {error}")
                    # Bank the interrupted run into the CPR timer, same as a pause. Skipped if compressions weren't running
                    if compression_start is not None:
                        cpr_elapsed_sec += time.monotonic() - compression_start
                        compression_start = None
                        logging.info(f"CPR stopped by kneel failure at {cpr_elapsed_sec:.1f} s total")

                    error = actuation.pause_compressions()
                    if error not in (ErrorCode.NORMAL_OPERATION, ErrorCode.ERROR_IMU_KNEEL_FAILURE): continue

                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.KNEEL_FAILURE, HMI.AudioPrompt.KNEEL_FAILURE)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.ALIGNMENT  # must re-confirm position before resuming

            case(CPRState.ABORT):
                # Setup
                if state != prev_state:
                    # We still care about error codes, in case of fatal error, but only functions that can return those
                    error = actuation.abort_compressions()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.ABORT, HMI.AudioPrompt.ABORT)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    logging.error(f"Abort state tripped: {error}")

                # Loop
                # The steppers only move while being updated, so keep retracting the carriage every tick
                # No continue on failure: the error is still routed back to ABORT next tick, but prev_state is
                # recorded first so ABORT's setup doesn't re-run, and the tick still sleeps
                error, _ = actuation.retract_carriage()
                # Halt — only a power cycle exits this state

            case(_):
                error = ErrorCode.EXIT_UNKNOWN

        # Record the previous state and sleep for the remainder of the tick duration
        prev_state = current_state
        elapsed = time.monotonic() - loop_start
        time.sleep(max(0, LOOP_TICK_SEC - elapsed))

    logging.critical(f"Fatal error: {error}")

    # The motor thread is a daemon thread with no independent safety stop:
    # if the process exits without this, it dies silently mid-command. Every
    # FATAL_ERRORS member can only occur after actuation.init_motor() has
    # already succeeded, so the motor controller is guaranteed to be live here.
    error = actuation.abort_compressions()
    if error == ErrorCode.NORMAL_OPERATION:
        logging.debug("Motor successfully aborted")
    else:
        logging.error("Motor abort failed")
    sys.exit(1)


if __name__ == "__main__":
    main()