# zeroing_key_test.py (demo)
"""
Bench test for the demo's carriage moves, stepped through with key presses in the terminal. Steppers only: the moteus
isn't started or commanded, so it doesn't need to be connected or powered.

Sequence:
    1. Lower the carriage STARTUP_LOWER_M (1 inch) from home, same as the demo's STARTUP state
    2. Press any key: lower the carriage to DEMO_ZERO_POSITION_M, where the demo's zeroing stops it
    3. Press any key: raise the carriage back to STARTUP_LOWER_M
    4. Back to step 2, so the move can be repeated
Keys pressed while the carriage is moving are ignored.

Press q (or Ctrl+C) at any time to stop: the carriage returns home and the script exits.

STARTUP_LOWER_M and DEMO_ZERO_POSITION_M come from demo/actuation.py, so the moves match the demo. The carriage moves at
the same speeds the demo uses: zeroing speed going down, retract speed going up. Unlike the demo's zeroing(), the
plunger isn't held, since the motor is never started.

Assumes the carriage starts fully retracted (home), since stepper positions are measured from where it starts.
The pigpio daemon must be running (sudo pigpiod).

The console shows INFO and above; the log file in Logs/ also has DEBUG.

Usage (from the Raspberry Pi code folder, on the Pi, in a terminal):
    python demo/scripts/zeroing_key_test.py
"""

# External imports
import os
import sys
import time
import tty
import select
import logging
import termios
from datetime import datetime
from pathlib import Path

CODE_DIR: Path = Path(__file__).resolve().parent.parent.parent  # the Raspberry Pi code folder
# Same import order as demo/main.py: actuation comes from demo/, everything else from src/
sys.path[:0] = [str(CODE_DIR / "demo"), str(CODE_DIR / "src")]

import pigpio

# Internal imports. actuation is only used for its carriage positions and lower_carriage(), which only drives the
# steppers; importing it doesn't start or talk to the moteus
import actuation  # demo/actuation.py
import stepper
from stepper import ZEROING_VELOCITY_MPS, RETRACT_VELOCITY_MPS
from Enums.error_codes import ErrorCode

LOOP_TICK_SEC: float = 0.05  # same tick as demo/main.py, which stepper.py's chunk queue is sized for
MOVE_TIMEOUT_SEC: float = 10.0  # seconds before giving up on any one carriage move

LOG_DIR: Path = CODE_DIR / "Logs"
RUN_TIMESTAMP: str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE: Path = LOG_DIR / f"zeroing_key_test_{RUN_TIMESTAMP}.log"


class Keyboard:
    """Reads key presses from the terminal without waiting for Enter. Non-blocking.
    Use as a context manager, which restores the terminal on exit."""

    def __init__(self):
        self._fd: int = sys.stdin.fileno()
        self._old_settings: list | None = None

    def is_terminal(self) -> bool:
        return os.isatty(self._fd)

    def __enter__(self) -> "Keyboard":
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)  # keys arrive one at a time and aren't echoed; Ctrl+C still raises KeyboardInterrupt
        return self

    def __exit__(self, *exc) -> None:
        if self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read(self) -> str:
        """Return every key pressed since the last call, or "" if none. Multi-byte keys (arrows) count as one press."""
        if not select.select([self._fd], [], [], 0)[0]:
            return ""
        return os.read(self._fd, 64).decode(errors="ignore")


def configure_logging() -> None:
    """Log INFO and above to the console, and everything to the log file, in the same format as demo/main.py."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[console, logging.FileHandler(LOG_FILE)],
    )


def mm(position_m: float) -> str:
    return f"{position_m * 1000:.1f} mm"


def move_carriage(target_m: float, velocity_mps: float) -> tuple[ErrorCode, bool]:
    """Drive the carriage to an absolute position. Non-blocking, call every tick until it reports arrived.

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, the stepper's error otherwise
        bool: True once the carriage is at target_m
    """
    error = stepper.set_target(target_m, velocity_mps)
    if error == ErrorCode.NORMAL_OPERATION:
        error = stepper.update()
    if error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Carriage move to {mm(target_m)} failed: {error}")
        stepper.halt()
        return error, False
    return ErrorCode.NORMAL_OPERATION, stepper.at_target()


def run(keyboard: Keyboard) -> int:
    """Run the test sequence until q is pressed or something fails.

    Returns:
        int: Exit code, 0 when stopped with q, 1 on a failure
    """
    phase: str = "LOWER"
    phase_start: float = time.monotonic()
    logging.info(f"Lowering carriage to {mm(actuation.STARTUP_LOWER_M)}")

    while True:
        tick_start: float = time.monotonic()

        # Read every tick, so keys pressed while moving are drained and ignored rather than queued up
        keys: str = keyboard.read()
        if "q" in keys.lower():
            logging.info("q pressed, stopping")
            return 0

        # Every move gets the same timeout, in case the carriage never reports arriving
        if phase in ("LOWER", "ZEROING", "RAISE") and time.monotonic() - phase_start > MOVE_TIMEOUT_SEC:
            logging.error(f"Carriage move timed out ({phase})")
            return 1

        match phase:
            case "LOWER":
                error, arrived = actuation.lower_carriage()
                if error != ErrorCode.NORMAL_OPERATION:
                    return 1
                if arrived:
                    logging.info(f"Carriage at {mm(stepper.get_position())}. Press any key to zero, q to quit")
                    phase = "WAIT_ZERO"

            case "WAIT_ZERO":
                if keys:
                    logging.info(f"Lowering carriage to zero position {mm(actuation.DEMO_ZERO_POSITION_M)}")
                    phase, phase_start = "ZEROING", time.monotonic()

            case "ZEROING":
                error, arrived = move_carriage(actuation.DEMO_ZERO_POSITION_M, ZEROING_VELOCITY_MPS)
                if error != ErrorCode.NORMAL_OPERATION:
                    return 1
                if arrived:
                    logging.info(f"Zeroed at {mm(stepper.get_position())} in {time.monotonic() - phase_start:.2f} s. "
                                 f"Press any key to raise back to {mm(actuation.STARTUP_LOWER_M)}, q to quit")
                    phase = "WAIT_RAISE"

            case "WAIT_RAISE":
                if keys:
                    logging.info(f"Raising carriage to {mm(actuation.STARTUP_LOWER_M)}")
                    phase, phase_start = "RAISE", time.monotonic()

            case "RAISE":
                error, arrived = move_carriage(actuation.STARTUP_LOWER_M, RETRACT_VELOCITY_MPS)
                if error != ErrorCode.NORMAL_OPERATION:
                    return 1
                if arrived:
                    logging.info(f"Carriage at {mm(stepper.get_position())}. Press any key to zero again, q to quit")
                    phase = "WAIT_ZERO"

        time.sleep(max(0.0, LOOP_TICK_SEC - (time.monotonic() - tick_start)))


def shutdown() -> None:
    """Return the carriage home.

    Homes to absolute position 0 rather than with actuation.retract_carriage(), which only undoes downward steps:
    the raise in this script moves up, so retract_carriage() would overshoot past home and fail.
    """
    logging.info("Returning carriage home")
    deadline: float = time.monotonic() + MOVE_TIMEOUT_SEC
    while True:
        error, home = move_carriage(0.0, RETRACT_VELOCITY_MPS)
        if error != ErrorCode.NORMAL_OPERATION or home:
            break
        if time.monotonic() > deadline:
            logging.error("Returning carriage home timed out")
            stepper.halt()
            break
        time.sleep(LOOP_TICK_SEC)
    logging.info(f"Carriage at {mm(stepper.get_position())}")


def main() -> None:
    configure_logging()
    logging.info(f"Logging to {LOG_FILE}")
    logging.info(f"Startup lower {mm(actuation.STARTUP_LOWER_M)}, zero at {mm(actuation.DEMO_ZERO_POSITION_M)}. "
                 f"The carriage must start fully retracted")

    keyboard = Keyboard()
    if not keyboard.is_terminal():
        logging.error("Run this from a terminal, it reads key presses")
        sys.exit(1)

    pi = pigpio.pi()
    if not pi.connected:
        logging.error("Can't connect to the pigpio daemon, start it with: sudo pigpiod")
        sys.exit(1)

    if actuation.init_steppers(pi) != ErrorCode.NORMAL_OPERATION:
        logging.error("Stepper initialization failed, nothing was moved")
        pi.stop()
        sys.exit(1)

    exit_code: int = 1
    try:
        with keyboard:
            exit_code = run(keyboard)
    except KeyboardInterrupt:
        logging.info("Ctrl+C, stopping")
        exit_code = 0
    finally:
        shutdown()
        pi.stop()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
