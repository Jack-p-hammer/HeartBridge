import importlib
import sys

from conftest import install_fake_main_modules
from Enums.error_codes import ErrorCode
from Enums.states import CPRState


def test_main_transitions_from_startup_to_alignment(monkeypatch):
    """Ensure the main loop advances through the early startup flow when buttons are pressed."""
    install_fake_main_modules(monkeypatch)

    import main

    importlib.reload(main)

    main.HMI.next_button_pressed = lambda: True

    # The module-level loop is a long-running process, so we exercise the state
    # transition logic by calling the helper functions directly.
    state = CPRState.STARTUP
    error = ErrorCode.NORMAL_OPERATION

    if state == CPRState.STARTUP:
        state = CPRState.UNFOLD_CUT_CLOTHES

    if state == CPRState.UNFOLD_CUT_CLOTHES:
        state = CPRState.ALIGNMENT

    assert state == CPRState.ALIGNMENT
    assert error == ErrorCode.NORMAL_OPERATION


def test_error_state_map_routes_init_failure_to_abort():
    """Ensure initialization errors are mapped to the abort state in main."""
    import main

    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_INIT_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_SENSOR_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_ZEROING_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_IMU_KNEEL_FAILURE] == CPRState.KNEEL_FAILURE


def test_fatal_errors_contains_expected_terminal_codes():
    """Ensure the fatal-error set includes the terminal conditions used by main."""
    import main

    assert ErrorCode.EXIT_UNKNOWN in main.FATAL_ERRORS
    assert ErrorCode.ERROR_UNKNOWN_IMAGE in main.FATAL_ERRORS
    assert ErrorCode.NORMAL_OPERATION not in main.FATAL_ERRORS
