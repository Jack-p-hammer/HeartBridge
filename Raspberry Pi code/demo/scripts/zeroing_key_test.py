# zeroing_key_test.py (demo)
"""
Bench test for the demo's hardcoded zeroing, stepped through with key presses in the terminal.

Sequence:
    1. Lower the carriage STARTUP_LOWER_M (1 inch) from home, same as the demo's STARTUP state
    2. Press any key: zero, lowering the carriage to DEMO_ZERO_POSITION_M with demo/actuation.py's zeroing()
    3. Press any key: raise the carriage back to STARTUP_LOWER_M
    4. Back to step 2, so zeroing can be repeated
Keys pressed while the carriage is moving are ignored.

Press q (or Ctrl+C) at any time to stop: the plunger returns to the top of its ball screw travel, the carriage returns
home, and the script exits.

Uses demo/actuation.py and demo/sensing.py the same way demo/main.py does, so it runs the same zeroing code. The screen,
audio, buttons and lasers aren't used, so it runs over SSH with no display. STARTUP_LOWER_M and DEMO_ZERO_POSITION_M
are set in demo/actuation.py.

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
# Same import order as demo/main.py: actuation and sensing come from demo/, everything else from src/
sys.path[:0] = [str(CODE_DIR / "demo"), str(CODE_DIR / "src")]

import pigpio

# Internal imports
import actuation  # demo/actuation.py
import sensing  # demo/sensing.py
import stepper
from stepper import RETRACT_VELOCITY_MPS
from Enums.error_codes import ErrorCode

LOOP_TICK_SEC: float = 0.05  # same tick as demo/main.py, which stepper.py's chunk queue is sized for
STARTUP_TIMEOUT_SEC: float = 2.0  # seconds to wait for the first reply from the moteus
MOVE_TIMEOUT_SEC: float = 10.0  # seconds before giving up on raising or homing the carriage
PLUNGER_SETTLE_SEC: float = 0.5  # seconds for the plunger to reach the top of its travel before commands stop

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


def init(pi: pigpio.pi) -> ErrorCode:
    """Start the motor, wait for its first reply, then initialize the steppers and sensors.

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_INIT_FAILURE otherwise
    """
    if actuation.init_motor() != ErrorCode.NORMAL_OPERATION:
        return ErrorCode.ERROR_INIT_FAILURE

    # The motor reports position 0 until its first reply, so wait for one before init_sensors() records the
    # plunger's startup position
    motor = actuation.get_motor_controller()
    deadline: float = time.monotonic() + STARTUP_TIMEOUT_SEC
    while motor.get_state().stale:
        if time.monotonic() > deadline:
            logging.error(f"No reply from moteus: {motor.get_last_error()}")
            return ErrorCode.ERROR_INIT_FAILURE
        time.sleep(0.01)
    if motor.get_state().mode == 1:
        logging.error(f"Moteus is faulted (fault {motor.get_state().fault}), clear it before running")
        return ErrorCode.ERROR_INIT_FAILURE

    if actuation.init_steppers(pi) != ErrorCode.NORMAL_OPERATION:
        return ErrorCode.ERROR_INIT_FAILURE
    if sensing.init_sensors(motor, pi) != ErrorCode.NORMAL_OPERATION:
        return ErrorCode.ERROR_INIT_FAILURE
    return ErrorCode.NORMAL_OPERATION


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
                    actuation.init_zeroing()
                    logging.info(f"Zeroing to {mm(actuation.DEMO_ZERO_POSITION_M)}")
                    phase, phase_start = "ZEROING", time.monotonic()

            case "ZEROING":
                result = actuation.zeroing()
                if result == ErrorCode.ZEROING_FINISHED:
                    if sensing.zero_position() != ErrorCode.NORMAL_OPERATION:
                        return 1
                    logging.info(f"Zeroed at {mm(stepper.get_position())} in {time.monotonic() - phase_start:.2f} s. "
                                 f"Press any key to raise back to {mm(actuation.STARTUP_LOWER_M)}, q to quit")
                    phase = "WAIT_RAISE"
                elif result != ErrorCode.NORMAL_OPERATION:
                    logging.error(f"Zeroing failed: {result}")
                    return 1

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
                elif time.monotonic() - phase_start > MOVE_TIMEOUT_SEC:
                    logging.error("Raising carriage timed out")
                    return 1

        time.sleep(max(0.0, LOOP_TICK_SEC - (time.monotonic() - tick_start)))


def shutdown() -> None:
    """Return the plunger to the top of its ball screw travel and the carriage home, then stop commanding the motor.

    Homes to absolute position 0 rather than with actuation.retract_carriage(), which only undoes downward steps:
    the raise in this script moves up, so retract_carriage() would overshoot past home and fail.
    """
    if actuation.abort_compressions() != ErrorCode.NORMAL_OPERATION:
        logging.error("Plunger retract command failed")

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

    # Commands stop with the thread, after which the moteus watchdog stops the motor
    time.sleep(PLUNGER_SETTLE_SEC)
    actuation.get_motor_controller().stop()


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

    if init(pi) != ErrorCode.NORMAL_OPERATION:
        logging.error("Initialization failed, nothing was moved")
        try:
            actuation.get_motor_controller().stop()
        except NameError:
            pass  # the motor thread was never created
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
