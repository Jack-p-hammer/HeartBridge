# main.py

# UI-only demo of the CPR state machine. Shows the same images, plays the same
# audio prompts and uses the same buttons as src/main.py, but all sensing and
# actuation (motor, sensors, battery check, lasers) has been removed. Button
# presses only advance to the next image and audio prompt.

# External imports
import time
import sys
import logging
from datetime import datetime
from pathlib import Path

# Internal imports
import HMI
from Enums.error_codes import ErrorCode
from Enums.states import CPRState

LOOP_TICK_SEC = 0.05  # 50ms per tick, 20 Hz

# Initialize log directory and file
# 2x parent to get to project root
LOG_DIR = Path(__file__).resolve().parent.parent / "Logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)  # Ensure logs directory exists
RUN_TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = LOG_DIR / f"heartBridge_demo_{RUN_TIMESTAMP}.log"

# Maps error codes to the state they should force the machine into
ERROR_STATE_MAP = {
    ErrorCode.ERROR_INIT_FAILURE:           CPRState.ABORT,
    ErrorCode.ERROR_PYGAME_FAILURE:         CPRState.ABORT,
}

FATAL_ERRORS = {
    ErrorCode.EXIT_UNKNOWN,
    ErrorCode.ERROR_PYGAME_INIT_FAILURE,
    ErrorCode.ERROR_PI_DAEMON_FAILURE,
    ErrorCode.ERROR_UNKNOWN_IMAGE,
}


def configure_logging():
    """Configure console and file logging for the demo runtime."""
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
    """Run the UI-only state-machine loop."""
    configure_logging()

    # Start by defining initial state and error code. The state machine will update these as it runs.
    state = CPRState.STARTUP
    prev_state = None
    error = ErrorCode.NORMAL_OPERATION

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
                    error = HMI.init_HMI(HMI.get_pi())
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    error = HMI.set_image_audio(HMI.Image.STARTUP, HMI.AudioPrompt.STARTUP)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.UNFOLD_CUT_CLOTHES

            case(CPRState.UNFOLD_CUT_CLOTHES):
                # Setup
                if state != prev_state:
                    error = HMI.set_image_audio(HMI.Image.UNFOLD, HMI.AudioPrompt.UNFOLD)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.ALIGNMENT

            case(CPRState.ALIGNMENT):
                # Setup
                if state != prev_state:
                    error = HMI.set_image_audio(HMI.Image.ALIGNMENT, HMI.AudioPrompt.ALIGNMENT)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.ZEROING_PREP

            case(CPRState.ZEROING_PREP):
                # Setup
                if state != prev_state:
                    error = HMI.set_image_audio(HMI.Image.ZEROING_PREP, HMI.AudioPrompt.ZEROING_PREP)
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

                # Loop
                # No motor to zero; move on once the prompt has played through once
                if HMI.audio_finished():
                    state = CPRState.COMPRESSION_PREP

            case(CPRState.COMPRESSION_PREP):
                # Setup
                if state != prev_state:
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_pause_button()
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

                # Loop
                error, pause_pressed = HMI.pause_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if pause_pressed:
                    state = CPRState.PAUSE

            case(CPRState.PAUSE):
                # Setup
                if state != prev_state:
                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.enable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.PAUSE, HMI.AudioPrompt.PAUSE)
                    if error != ErrorCode.NORMAL_OPERATION: continue

                    logging.debug("Device Paused")

                # Loop
                error, next_pressed = HMI.next_button_pressed()
                if error != ErrorCode.NORMAL_OPERATION: continue
                if next_pressed:
                    state = CPRState.COMPRESSION_PREP

            case(CPRState.ABORT):
                # Setup
                if state != prev_state:
                    # We still care about error codes, in case of fatal error, but only functions that can return those
                    error = HMI.disable_next_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.disable_pause_button()
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    error = HMI.set_image_audio(HMI.Image.ABORT, HMI.AudioPrompt.ABORT)
                    if error != ErrorCode.NORMAL_OPERATION: continue
                    logging.error(f"Abort state tripped: {error}")
                # Halt — only a power cycle exits this state

            case(_):
                error = ErrorCode.EXIT_UNKNOWN

        # Record the previous state and sleep for the remainder of the tick duration
        prev_state = current_state
        elapsed = time.monotonic() - loop_start
        time.sleep(max(0, LOOP_TICK_SEC - elapsed))

    logging.critical(f"Fatal error: {error}")
    sys.exit(1)


if __name__ == "__main__":
    main()
