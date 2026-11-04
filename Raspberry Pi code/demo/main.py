# main.py (demo)

# Runs the real state machine in src/main.py, with demo/sensing.py in place of src/sensing.py.
# Every other module (actuation, stepper, moteus_thread, HMI, Enums) is imported from src/ unaltered,
# so any bug found with the demo is a bug in the main machine.

# External imports
import sys
import logging
import importlib.util
from pathlib import Path

DEMO_DIR = Path(__file__).resolve().parent
SRC_DIR = DEMO_DIR.parent / "src"

# demo/ goes first so `import sensing` finds demo/sensing.py; everything else is only in src/
sys.path[:0] = [str(DEMO_DIR), str(SRC_DIR)]

# Load src/main.py under its own name, since this file is also called main.py
spec = importlib.util.spec_from_file_location("heartbridge_main", SRC_DIR / "main.py")
if spec is None or spec.loader is None:
    sys.exit(f"Could not load {SRC_DIR / 'main.py'}")
heartbridge_main = importlib.util.module_from_spec(spec)
spec.loader.exec_module(heartbridge_main)

# Mark demo runs in the log file name. setattr since the loaded module's attributes aren't known to the type checker
setattr(heartbridge_main, "LOG_FILE", heartbridge_main.LOG_DIR / f"heartBridge_demo_{heartbridge_main.RUN_TIMESTAMP}.log")

# These are the same module objects src/main.py uses
import pygame
import HMI
import actuation
import stepper
from Enums.error_codes import ErrorCode

# Demo-only exit key. src/main.py calls HMI.pump_events() every tick, so the key check rides along with it
_pump_events = HMI.pump_events


def quit_requested() -> bool:
    """Check whether the user pressed Esc or Q, or closed the window.

    HMI.pump_events() keeps the window alive but never reads keys, so the
    fullscreen window would otherwise have no way to exit back to the terminal.
    """
    # The display doesn't exist until HMI.init_HMI() runs on the first tick
    if not pygame.display.get_init():
        return False
    for event in pygame.event.get((pygame.QUIT, pygame.KEYDOWN)):
        if event.type == pygame.QUIT:
            return True
        if event.key in (pygame.K_ESCAPE, pygame.K_q):
            return True
    return False


def shutdown_demo() -> None:
    """Stop the motor and steppers, turn off lasers and button LEDs, close the window and exit cleanly."""
    logging.info("Exit requested, shutting down demo")
    # Same motor stop src/main.py does on a fatal error. Fails if Esc/Q is pressed before the motor is initialized
    try:
        actuation.abort_compressions()
        stepper.halt()
    except Exception as e:
        logging.error(f"Motor stop on exit failed: {e}")
    HMI.disable_lasers()
    HMI.disable_next_button()
    HMI.disable_pause_button()
    pygame.quit()
    sys.exit(0)


def pump_events_with_quit() -> ErrorCode:
    """HMI.pump_events(), plus Esc or Q (or closing the window) exits back to the terminal from any state."""
    error: ErrorCode = _pump_events()
    if quit_requested():
        shutdown_demo()
    return error


HMI.pump_events = pump_events_with_quit


if __name__ == "__main__":
    heartbridge_main.main()
