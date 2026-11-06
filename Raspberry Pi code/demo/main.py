# main.py (demo)

# Runs the real state machine in src/main.py, with demo/sensing.py in place of src/sensing.py.
# Every other module (actuation, stepper, moteus_thread, HMI, Enums) is imported from src/ unaltered,
# so any bug found with the demo is a bug in the main machine.
# Hardware needed: the Next and Pause buttons, the steppers and their drivers, the moteus, the screen and the speaker.
# No sensors, lasers or keyboard: PAUSE_PRESS_TOUR below reaches KNEEL_FAILURE and ABORT from the Pause button.

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
import sensing  # demo/sensing.py, since demo/ is first on the path
from Enums.error_codes import ErrorCode

# Demo-only keys. src/main.py calls HMI.pump_events() every tick, so the key check rides along with it
_pump_events = HMI.pump_events


def handle_demo_keys() -> bool:
    """Handle the demo-only keys, and return whether the user pressed Esc or Q, or closed the window.

    HMI.pump_events() keeps the window alive but never reads keys, so the fullscreen window would otherwise have no
    way to exit back to the terminal, and with the sensor suite gone nothing could reach the states it triggers:
        K: simulated kneel failure -> KNEEL_FAILURE
        A: simulated sensor failure -> ABORT
    Both take effect the next time the sensors are read (zeroing, compressions or pause).
    """
    # The display doesn't exist until HMI.init_HMI() runs on the first tick
    if not pygame.display.get_init():
        return False
    quit_pressed: bool = False
    for event in pygame.event.get((pygame.QUIT, pygame.KEYDOWN)):
        if event.type == pygame.QUIT or event.key in (pygame.K_ESCAPE, pygame.K_q):
            quit_pressed = True
        elif event.key == pygame.K_k:
            sensing.simulate_fault(ErrorCode.ERROR_IMU_KNEEL_FAILURE)
        elif event.key == pygame.K_a:
            sensing.simulate_fault(ErrorCode.ERROR_SENSOR_FAILURE)
    return quit_pressed


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


def pump_events_with_demo_keys() -> ErrorCode:
    """HMI.pump_events(), plus the demo keys. Esc or Q (or closing the window) exits back to the terminal from any state."""
    error: ErrorCode = _pump_events()
    if handle_demo_keys():
        shutdown_demo()
    return error


HMI.pump_events = pump_events_with_demo_keys


# Button-only tour of the states the sensor suite would trigger, so the demo needs no keyboard. Each entry is what the
# next Pause press during compressions does: None pauses as normal, an ErrorCode is reported by the next sensor read
# instead (see sensing.simulate_fault()), sending the machine where that sensor fault would. Once the list runs out,
# Pause presses pause as normal. ABORT ends the tour, since only a restart leaves it. Set to [] to turn the tour off
PAUSE_PRESS_TOUR: list[ErrorCode | None] = [
    None,                               # 1st press: PAUSE, then Next resumes compressions
    ErrorCode.ERROR_IMU_KNEEL_FAILURE,  # 2nd press: KNEEL_FAILURE, then Next goes back to ALIGNMENT
    ErrorCode.ERROR_SENSOR_FAILURE,     # 3rd press: ABORT
]
_pause_presses: int = 0  # Pause presses accepted so far
_pause_button_pressed = HMI.pause_button_pressed


def pause_button_pressed_with_tour() -> tuple[ErrorCode, bool]:
    """HMI.pause_button_pressed(), with each accepted press following PAUSE_PRESS_TOUR.
    src/main.py only reads the Pause button during compressions, so only those presses are counted."""
    global _pause_presses
    error, pressed = _pause_button_pressed()
    if error != ErrorCode.NORMAL_OPERATION or not pressed:
        return error, pressed

    _pause_presses += 1
    if _pause_presses > len(PAUSE_PRESS_TOUR):
        return error, pressed

    fault: ErrorCode | None = PAUSE_PRESS_TOUR[_pause_presses - 1]
    if fault is None:
        logging.info(f"Demo tour: Pause press {_pause_presses}/{len(PAUSE_PRESS_TOUR)} pauses as normal")
        return error, pressed
    logging.info(f"Demo tour: Pause press {_pause_presses}/{len(PAUSE_PRESS_TOUR)} simulates {fault.name}")
    sensing.simulate_fault(fault)
    # Not a pause: the fault, reported by the next sensor read, decides the next state
    return error, False


HMI.pause_button_pressed = pause_button_pressed_with_tour


if __name__ == "__main__":
    try:
        heartbridge_main.main()
    except KeyboardInterrupt:
        # Ctrl+C in the terminal (e.g. over SSH) gets the same motor stop as Esc or Q
        shutdown_demo()
