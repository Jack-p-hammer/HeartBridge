# main.py (demo)

# Copy of the state machine in src/main.py, modified for the demo. Differences from src/main.py:
#   - demo/sensing.py is used in place of src/sensing.py
#   - demo/Enums/states.py is used in place of src/Enums/states.py
#   - Log files are named heartBridge_demo_<timestamp>.log
#   - Esc or Q (or closing the window) stops the motor and steppers and exits back to the terminal from any state
# Every other module (actuation, stepper, moteus_thread, HMI, Enums.error_codes, Enums.control_modes)
# is imported from src/ unaltered.

# External imports
import time
import sys
import logging
from datetime import datetime
from pathlib import Path

DEMO_DIR = Path(__file__).resolve().parent
SRC_DIR = DEMO_DIR.parent / "src"

# demo/ goes first so `import sensing` finds demo/sensing.py and `Enums.states` finds demo/Enums/states.py;
# everything else is only in src/.
# Enums has no __init__.py in either folder, so Python merges demo/Enums and src/Enums into one package:
# states comes from demo/Enums, error_codes and control_modes from src/Enums.
# Adding an __init__.py to either Enums folder breaks this merge.
sys.path[:0] = [str(DEMO_DIR), str(SRC_DIR)]

import pygame

# Internal imports
import sensing
import actuation
import stepper
import HMI
from Enums.error_codes import ErrorCode
from Enums.states import CPRState  # demo/Enums/states.py

# TODO: Optimize loop speed
LOOP_TICK_SEC = 0.05  # 50ms per tick, 20 Hz

# Frames cycled during the unfold step, and how long each one stays up
UNFOLD_FRAMES = [HMI.Image.UNFOLD_1, HMI.Image.UNFOLD_2, HMI.Image.UNFOLD_3, HMI.Image.UNFOLD_4]
UNFOLD_FRAME_SEC = 0.5  # 2 frames per second

# The battery voltage comes from the moteus, so it reads 0 V until the first reply. Wait this long for one at startup
MOTOR_FIRST_REPLY_TIMEOUT_SEC = 5.0

# Initialize log directory and file
# 2x parent to get to project root
LOG_DIR = Path(__file__).resolve().parent.parent / "Logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)  # Ensure logs directory exists
RUN_TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = LOG_DIR / f"heartBridge_demo_{RUN_TIMESTAMP}.log"

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
    #   2026-05-10 12:00:00 - DEBUG - Sensors initialized
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


def read_keys() -> tuple[ErrorCode, list[int]]:
    """Return the keys pressed since the last tick, and exit the demo if the window was closed. Non-blocking.

    HMI.pump_events() keeps the window alive but never reads keys. This takes the key
    presses off pygame's queue, so it must be called exactly once per tick, and every
    key check in the loop reads its result.

    Returns:
        tuple[ErrorCode, list[int]]: NORMAL_OPERATION and the pygame key codes pressed, or ERROR_PYGAME_FAILURE and
            no keys if the event queue couldn't be read
    """
    # The display doesn't exist until HMI.init_HMI() runs in INIT_STARTUP
    if not pygame.display.get_init():
        return ErrorCode.NORMAL_OPERATION, []

    try:
        events = pygame.event.get((pygame.QUIT, pygame.KEYDOWN))
    except Exception as e:
        logging.error(f"Failed to read keyboard events: {e}")
        return ErrorCode.ERROR_PYGAME_FAILURE, []

    keys_pressed: list[int] = []
    for event in events:
        if event.type == pygame.QUIT:
            shutdown_demo()
        keys_pressed.append(event.key)
    return ErrorCode.NORMAL_OPERATION, keys_pressed


def shutdown_demo() -> None:
    """Stop the motor and steppers, turn off lasers and button LEDs, close the window and exit cleanly.

    Only called once a key press or window close has been read, which needs HMI.init_HMI() to have run.
    INIT_STARTUP only gets there after init_motor() and init_steppers() succeed, so both are initialized here.
    Failures are logged and the shutdown carries on, since the demo is exiting either way.
    """
    logging.info("Exit requested, shutting down demo")

    # Same motor stop main() does on a fatal error
    error = actuation.abort_compressions()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor abort on exit failed: {error}")
    stepper.halt()

    error = HMI.disable_lasers()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Disabling lasers on exit failed: {error}")
    error = HMI.disable_next_button()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Disabling Next button on exit failed: {error}")
    error = HMI.disable_pause_button()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Disabling Pause button on exit failed: {error}")

    pygame.quit()
    sys.exit(0)


def main():
    """Run the main state-machine loop for the CPR machine."""
    configure_logging()

    # Start by defining initial state and error code. The state machine will update these as it runs.
    state = CPRState.INIT_STARTUP
    prev_state = None
    error = ErrorCode.NORMAL_OPERATION

    # Total time spent in COMPRESSION across all runs. Pausing (or a kneel failure) freezes it and
    # resuming carries on from the same total, so it reads total CPR time.
    cpr_elapsed_sec: float = 0.0
    compression_start: float | None = None  # time.monotonic() when the current run of compressions started, None while not compressing
    shown_sec: int | None = None  # Whole seconds currently drawn on the timer, None to force a redraw
    abort_carriage_home: bool = False  # True once ABORT has finished retracting the carriage, so it stops re-retracting
    init_done_time: float = 0.0  # time.monotonic() when INIT_STARTUP's setup finished, to time out waiting for the moteus

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
        # Demo only: Esc or Q (or closing the window) exits back to the terminal from any state
        error, keys_pressed = read_keys()
        if error != ErrorCode.NORMAL_OPERATION: continue
        if pygame.K_ESCAPE in keys_pressed or pygame.K_q in keys_pressed:
            shutdown_demo()
        

        # --- State machine ---
        # Each state checks (state != prev_state) to detect first entry, and runs
        # one-time setup (screen, audio, button enables) only on that first tick.
        # All other logic runs every tick until the exit condition is met.
        
        # Capture current state to avoid issues with state changing mid-loop
        current_state = state
        match(current_state):
            case(CPRState.INIT_STARTUP):
                # Power cycle Setup
                if state != prev_state:
                    error = HMI.init_Pi()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = actuation.init_motor()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    time.sleep(0.5)  # Wait for the motor controller to be ready
                    error = actuation.init_steppers(HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.init_HMI(HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = sensing.init_sensors(actuation.get_motor_controller(), HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    init_done_time = time.monotonic()

                # Loop
                # Until the moteus first replies its state is stale and the battery reads 0 V, so only check the
                # battery once a reply has arrived. No reply within the timeout is a motor failure
                if actuation.get_motor_controller().get_state().stale:
                    if time.monotonic() - init_done_time > MOTOR_FIRST_REPLY_TIMEOUT_SEC:
                        logging.error(f"No reply from the moteus within {MOTOR_FIRST_REPLY_TIMEOUT_SEC} s of startup")
                        error = ErrorCode.ERROR_MOTOR_FAILURE
                else:
                    error = sensing.battery_check()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    # Go to the WAIT state, which is only used in the demo
                    state = CPRState.WAIT
                    
            case(CPRState.WAIT):
                # Wait for a keyboard input to start the demo. This state is only used in the demo, not in src/main.py.
                if state != prev_state:
                    # B starts the demo
                    logging.debug(f"Press B to start the demo, Esc or Q to exit")
                    # No images yet, TODO: Add a black screen or sm
                    pass

                # Loop

                if pygame.K_b in keys_pressed:
                    logging.info("B pressed, starting demo")
                    state = CPRState.STARTUP

            case(CPRState.STARTUP):
                # Setup
                if state != prev_state:
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
                    state = CPRState.COMPRESSION

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
                    logging.debug("Press R to reset the demo")

                # Loop
                # TODO: Must track kneel failure during pause as well
                error = actuation.abort_compressions()
                if error != ErrorCode.NORMAL_OPERATION: continue
                
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.COMPRESSION
                    
                if pygame.K_r in keys_pressed:
                    logging.info("R pressed, resetting demo")
                    state = CPRState.RESET_DEMO

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

            case(CPRState.RESET_DEMO):
                # Setup
                if state != prev_state:
                    logging.info("Resetting demo to STARTUP")
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = actuation.abort_compressions()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    
                    # Reset CPR timer
                    cpr_elapsed_sec = 0.0

                # Return carriage position to home
                error, retreat_success = actuation.retract_carriage()
                if error != ErrorCode.NORMAL_OPERATION: continue
                
                if retreat_success:
                    state = CPRState.WAIT  # Return to the demo's WAIT state
                
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
                    abort_carriage_home = False

                # Loop
                # The steppers only move while being updated, so keep retracting the carriage every tick until home.
                # Once home, stop calling retract_carriage(): it resets its target when done, so each further call
                # would start (and log) a fresh zero-length retraction
                # No continue on failure: the error is still routed back to ABORT next tick, but prev_state is
                # recorded first so ABORT's setup doesn't re-run, and the tick still sleeps
                if not abort_carriage_home:
                    error, abort_carriage_home = actuation.retract_carriage()
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