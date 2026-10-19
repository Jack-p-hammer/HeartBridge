"""Tests for main.py: the top-level CPR-machine state machine.

main.main() only returns via sys.exit() when it hits a FATAL_ERRORS code; any
other error just re-routes the state machine and the loop continues forever
by design (see CPRState.ABORT: "Halt -- only a power cycle exits this
state"). So tests that want to observe non-fatal routing use
_run_main_for_ticks() below to bail out after a bounded number of
iterations, instead of letting main() run forever.

Note: calling main.main() runs its real configure_logging(), which creates a
new (small, mostly empty) log file under <repo root>/Logs/ each time -- a
pre-existing side effect of importing/running main.py, not something these
tests add.
"""
import pytest

from conftest import install_fake_main_modules
from Enums.error_codes import ErrorCode
from Enums.states import CPRState


class _StopMainLoop(Exception):
    """Sentinel raised to break out of main()'s otherwise-infinite loop."""


def _run_main_for_ticks(monkeypatch, main, max_ticks):
    """Run main.main() for at most max_ticks loop iterations, then return.

    Patches time.sleep (main's per-tick pacing call) to count calls and raise
    _StopMainLoop once max_ticks is reached, so a test can inspect what the
    fakes recorded without needing main() to actually exit.
    """
    ticks = {"count": 0}

    def fake_sleep(seconds):
        ticks["count"] += 1
        if ticks["count"] >= max_ticks:
            raise _StopMainLoop()

    monkeypatch.setattr(main.time, "sleep", fake_sleep)
    try:
        main.main()
    except _StopMainLoop:
        pass


class _Recorder:
    """Callable fake that records each call's args and returns a fixed value.

    A plain function with a `.calls` list bolted onto it would work at
    runtime but isn't something a type checker accepts (function objects
    don't declare arbitrary attributes) -- a small class with a real,
    declared `calls` attribute avoids that friction.
    """

    def __init__(self, return_value=ErrorCode.NORMAL_OPERATION):
        self.return_value = return_value
        self.calls: list[tuple] = []

    def __call__(self, *args, **kwargs):
        self.calls.append(args)
        return self.return_value


def _fail_n_times(n, failure_result, then_result=ErrorCode.NORMAL_OPERATION):
    """Build a zero-arg fake that returns `failure_result` for its first `n`
    calls, then `then_result` after. Use for a function called from two
    different states (e.g. actuation.abort_compressions(), which both PAUSE's
    loop and ABORT's own setup call) when only a bounded number of failures
    should happen -- failing it forever can make the second call site's setup
    never complete either, an infinite busy-loop that never reaches
    time.sleep() (see conversation, same issue as the KNEEL_FAILURE/
    pause_compressions() case).
    """
    calls = {"count": 0}

    def fn(*args, **kwargs):
        calls["count"] += 1
        return failure_result if calls["count"] <= n else then_result

    return fn


def _cascade_fakes(modules, next_pressed=True, pause_pressed=True,
                    zeroing_result=ErrorCode.ZEROING_FINISHED):
    """Wire the whole-module fakes so main() can freely cascade through every
    Next/Pause-driven and auto (zeroing-finished/audio-finished) transition.

    Flow-only tests don't care what each state's setup actually does (see
    conversation), just which state comes next, so this always overrides
    next_button_pressed/pause_button_pressed/zeroing with fixed values --
    pause_button_pressed is only ever read from COMPRESSION, so leaving it
    True from the start is harmless everywhere else. Returns the
    set_image_audio recorder so a test can check which states were entered
    and in what order.
    """
    setattr(modules["HMI"], "next_button_pressed", lambda: (ErrorCode.NORMAL_OPERATION, next_pressed))
    setattr(modules["HMI"], "pause_button_pressed", lambda: (ErrorCode.NORMAL_OPERATION, pause_pressed))
    setattr(modules["actuation"], "zeroing", lambda: zeroing_result)
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)
    return set_image_audio


# -------------------- static routing tables --------------------

def test_error_state_map_routes_expected_errors_to_states(monkeypatch):
    """Every recoverable error routes to ABORT except the kneel failure, which routes
    to KNEEL_FAILURE so the operator can re-confirm alignment before resuming."""
    install_fake_main_modules(monkeypatch)
    import main

    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_INIT_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_SENSOR_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_ZEROING_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_LOW_BATTERY] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_MOTOR_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_PYGAME_FAILURE] == CPRState.ABORT
    assert main.ERROR_STATE_MAP[ErrorCode.ERROR_IMU_KNEEL_FAILURE] == CPRState.KNEEL_FAILURE


def test_fatal_errors_contains_expected_terminal_codes(monkeypatch):
    """Only truly unrecoverable conditions should end the process outright."""
    install_fake_main_modules(monkeypatch)
    import main

    assert main.FATAL_ERRORS == {
        ErrorCode.EXIT_UNKNOWN,
        ErrorCode.ERROR_PYGAME_INIT_FAILURE,
        ErrorCode.ERROR_PI_DAEMON_FAILURE,
        ErrorCode.ERROR_UNKNOWN_IMAGE,
    }
    assert ErrorCode.NORMAL_OPERATION not in main.FATAL_ERRORS
    # A fatal error should end the process outright, not get silently routed
    # to a state -- so the two tables should never overlap.
    assert main.FATAL_ERRORS.isdisjoint(main.ERROR_STATE_MAP.keys())


# -------------------- fatal-error fast exit --------------------

def test_main_exits_on_hmi_init_failure(monkeypatch):
    """A pygame init failure during startup should abort the motor and exit(1)."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["HMI"], "init_HMI", lambda pi_instance: ErrorCode.ERROR_PYGAME_INIT_FAILURE)
    abort = _Recorder()
    setattr(modules["actuation"], "abort_compressions", abort)

    import main
    with pytest.raises(SystemExit) as exc_info:
        main.main()

    assert exc_info.value.code == 1
    assert len(abort.calls) == 1  # the shutdown sequence aborts the motor before exiting


def test_main_exits_on_unknown_image(monkeypatch):
    """An unrecognized image path during startup is fatal, not routed to ABORT state."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["HMI"], "set_image_audio", lambda image, prompt: ErrorCode.ERROR_UNKNOWN_IMAGE)
    abort = _Recorder()
    setattr(modules["actuation"], "abort_compressions", abort)

    import main
    with pytest.raises(SystemExit) as exc_info:
        main.main()

    assert exc_info.value.code == 1
    assert len(abort.calls) == 1  # the shutdown sequence aborts the motor before exiting


def test_main_exits_on_pi_daemon_failure(monkeypatch):
    """A pigpio daemon failure anywhere is fatal -- GPIO state can't be trusted afterward."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["HMI"], "enable_next_button", lambda: ErrorCode.ERROR_PI_DAEMON_FAILURE)
    abort = _Recorder()
    setattr(modules["actuation"], "abort_compressions", abort)

    import main
    with pytest.raises(SystemExit) as exc_info:
        main.main()

    assert exc_info.value.code == 1
    assert len(abort.calls) == 1  # the shutdown sequence aborts the motor before exiting


# -------------------- non-fatal error routing --------------------

def test_main_routes_init_failure_to_abort_state(monkeypatch):
    """A non-fatal init failure should route to ABORT and run its setup, not exit."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "init_motor", lambda: ErrorCode.ERROR_INIT_FAILURE)
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=3)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_routes_kneel_failure_to_kneel_failure_state(monkeypatch):
    """An IMU kneel failure should route to KNEEL_FAILURE, not ABORT, and run its setup.

    Nothing in the current src/ actually produces ERROR_IMU_KNEEL_FAILURE yet
    (see summary), so this forces it directly from a fake to test main.py's
    routing in isolation from whichever real check eventually raises it.
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "init_motor", lambda: ErrorCode.ERROR_IMU_KNEEL_FAILURE)
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=3)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls


# -------------------- startup happy path --------------------

def test_main_starts_in_startup_state(monkeypatch):
    """The very first tick should run STARTUP's setup (proving state == STARTUP)."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=1)

    assert (main.HMI.Image.STARTUP, main.HMI.AudioPrompt.STARTUP) in set_image_audio.calls


def test_main_calls_pump_events_every_tick_across_states(monkeypatch):
    """pump_events() must run every tick regardless of state ("or the OS can mark
    the window as unresponsive", per main.py's own comment) -- confirmed by
    counting calls across several ticks that span multiple different states,
    not just checking it ran once during STARTUP.
    """
    modules = install_fake_main_modules(monkeypatch)
    pump_events = _Recorder()
    setattr(modules["HMI"], "pump_events", pump_events)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=5)

    # Confirm the run actually spanned multiple different states...
    assert (main.HMI.Image.STARTUP, main.HMI.AudioPrompt.STARTUP) in set_image_audio.calls
    assert (main.HMI.Image.ZEROING_PREP, main.HMI.AudioPrompt.ZEROING_PREP) in set_image_audio.calls
    # ...and pump_events() ran on every single one of those ticks, not just once.
    assert len(pump_events.calls) == 5


def test_main_pump_events_warning_does_not_disrupt_routing(monkeypatch):
    """WARNING_PYGAME_PUMP_FAILURE is deliberately absent from both
    ERROR_STATE_MAP and FATAL_ERRORS (see the static test above) -- confirm a
    live run with pump_events() failing on every tick still advances normally
    through the whole happy path, not just that the tables say it should.
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["HMI"], "pump_events", lambda: ErrorCode.WARNING_PYGAME_PUMP_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.COMPRESSION, main.HMI.AudioPrompt.COMPRESSION) in set_image_audio.calls


def test_main_abort_state_is_terminal(monkeypatch):
    """Once ABORT is entered, nothing else should happen on later ticks: its
    setup ("Halt -- only a power cycle exits this state") must run exactly
    once, never re-triggered across many further ticks.
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "init_motor", lambda: ErrorCode.ERROR_INIT_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    abort_entries = set_image_audio.calls.count((main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT))
    assert abort_entries == 1


def test_main_advances_from_startup_when_next_pressed(monkeypatch):
    """With the Next button held down, each state's setup should run once and advance."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["HMI"], "next_button_pressed", lambda: (ErrorCode.NORMAL_OPERATION, True))
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=3)

    assert (main.HMI.Image.STARTUP, main.HMI.AudioPrompt.STARTUP) in set_image_audio.calls
    assert (main.HMI.Image.UNFOLD, main.HMI.AudioPrompt.UNFOLD) in set_image_audio.calls


# -------------------- flow: remaining happy-path edges --------------------
# STARTUP -> UNFOLD_CUT_CLOTHES -> ALIGNMENT is covered above; these cover the
# rest of the chain: ALIGNMENT -> ZEROING_PREP -> ZEROING -> COMPRESSION_PREP
# -> COMPRESSION -> PAUSE -> COMPRESSION_PREP (loop), and KNEEL_FAILURE -> ALIGNMENT.

def test_main_flow_alignment_to_zeroing_prep(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ZEROING_PREP, main.HMI.AudioPrompt.ZEROING_PREP) in set_image_audio.calls


def test_main_flow_zeroing_prep_to_zeroing(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ZEROING, main.HMI.AudioPrompt.ZEROING) in set_image_audio.calls


def test_main_flow_zeroing_to_compression_prep_when_finished(monkeypatch):
    """zeroing() returning ZEROING_FINISHED should advance to COMPRESSION_PREP."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ZEROING_FINISHED)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.COMPRESSION_PREP, main.HMI.AudioPrompt.COMPRESSION_PREP) in set_image_audio.calls


def test_main_flow_zeroing_stays_while_not_finished(monkeypatch):
    """zeroing() returning plain NORMAL_OPERATION (still in progress) should not advance."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.NORMAL_OPERATION)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ZEROING, main.HMI.AudioPrompt.ZEROING) in set_image_audio.calls
    assert (main.HMI.Image.COMPRESSION_PREP, main.HMI.AudioPrompt.COMPRESSION_PREP) not in set_image_audio.calls


def test_main_flow_zero_position_failure_after_zeroing_finished_routes_to_abort(monkeypatch):
    """zeroing() finishing only advances to COMPRESSION_PREP if sensing.zero_position()
    also succeeds right after -- a failure there should route to ABORT instead of
    silently advancing or getting stuck."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["sensing"], "zero_position", lambda: ErrorCode.ERROR_SENSOR_FAILURE)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ZEROING_FINISHED)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls
    assert (main.HMI.Image.COMPRESSION_PREP, main.HMI.AudioPrompt.COMPRESSION_PREP) not in set_image_audio.calls


def test_main_flow_compression_prep_to_compression_when_audio_finished(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules)
    setattr(modules["HMI"], "audio_finished", lambda: True)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.COMPRESSION, main.HMI.AudioPrompt.COMPRESSION) in set_image_audio.calls


def test_main_flow_compression_prep_stays_while_audio_not_finished(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules)
    setattr(modules["HMI"], "audio_finished", lambda: False)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.COMPRESSION_PREP, main.HMI.AudioPrompt.COMPRESSION_PREP) in set_image_audio.calls
    assert (main.HMI.Image.COMPRESSION, main.HMI.AudioPrompt.COMPRESSION) not in set_image_audio.calls


def test_main_flow_compression_to_pause_on_pause_button(monkeypatch):
    """Regression test: COMPRESSION must read the pause button (Stop), not Next, to
    enter PAUSE -- this was briefly wired to next_button_pressed() and has been fixed.

    Both buttons are held down: Next is what drives every earlier state
    forward to reach COMPRESSION in the first place, and Pause is the actual
    trigger being tested once there (see test_main_flow_compression_does_not_
    pause_on_next_button_alone for the isolated negative case).
    """
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, next_pressed=True, pause_pressed=True)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.PAUSE, main.HMI.AudioPrompt.PAUSE) in set_image_audio.calls


def test_main_flow_compression_does_not_pause_on_next_button_alone(monkeypatch):
    """The Next button alone (pause not pressed) must not enter PAUSE from COMPRESSION."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, next_pressed=True, pause_pressed=False)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.COMPRESSION, main.HMI.AudioPrompt.COMPRESSION) in set_image_audio.calls
    assert (main.HMI.Image.PAUSE, main.HMI.AudioPrompt.PAUSE) not in set_image_audio.calls


def test_main_flow_pause_to_compression_prep_on_next(monkeypatch):
    """Pause resumes back into COMPRESSION_PREP (not straight to COMPRESSION) on Next."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    # COMPRESSION_PREP is entered once on the way to the first COMPRESSION, then
    # again after PAUSE resumes -- two entries proves the loop-back happened.
    compression_prep_entries = set_image_audio.calls.count(
        (main.HMI.Image.COMPRESSION_PREP, main.HMI.AudioPrompt.COMPRESSION_PREP)
    )
    assert compression_prep_entries >= 2


def test_main_flow_pause_loop_abort_failure_routes_to_abort(monkeypatch):
    """PAUSE's ongoing loop (not just its setup) calls actuation.abort_compressions()
    every tick; a motor failure there should route to ABORT.

    Only fails on the first call (from PAUSE's loop): ABORT's own setup also
    calls abort_compressions(), so failing it forever would recreate the same
    busy-loop the KNEEL_FAILURE/pause_compressions() case hit earlier (see
    conversation) -- ABORT's setup would never reach set_image_audio().
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "abort_compressions", _fail_n_times(1, ErrorCode.ERROR_MOTOR_FAILURE))
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.PAUSE, main.HMI.AudioPrompt.PAUSE) in set_image_audio.calls
    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_flow_kneel_failure_to_alignment_on_next(monkeypatch):
    """Kneel failure resumes into ALIGNMENT (re-confirm position), not COMPRESSION_PREP.

    Forces the kneel failure from zeroing_result rather than from init_motor
    (as test_main_routes_kneel_failure_to_kneel_failure_state does), so the
    happy-path cascade naturally passes through ALIGNMENT once *before* the
    failure fires -- letting this test distinguish "landed back on ALIGNMENT"
    from "never left ALIGNMENT in the first place" by requiring two entries.
    """
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ERROR_IMU_KNEEL_FAILURE)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls
    alignment_entries = set_image_audio.calls.count(
        (main.HMI.Image.ALIGNMENT, main.HMI.AudioPrompt.ALIGNMENT)
    )
    assert alignment_entries >= 2


def test_main_flow_kneel_failure_stays_without_next_and_has_no_ongoing_monitoring(monkeypatch):
    """KNEEL_FAILURE intentionally requires only the operator's acknowledgement
    (Next) to return to ALIGNMENT -- its loop does no ongoing sensor/actuation
    monitoring beyond the one-time setup call to pause_compressions(). This is
    confirmed-correct current behavior, not a bug -- locked in as a regression
    test so a future change can't silently reintroduce per-tick monitoring, or
    drop the Next-only exit, without a test catching it.
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "init_motor", lambda: ErrorCode.ERROR_IMU_KNEEL_FAILURE)
    pause_compressions = _Recorder()
    setattr(modules["actuation"], "pause_compressions", pause_compressions)
    setattr(modules["HMI"], "next_button_pressed", lambda: (ErrorCode.NORMAL_OPERATION, False))
    set_image_audio = _Recorder()
    setattr(modules["HMI"], "set_image_audio", set_image_audio)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=10)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls
    assert (main.HMI.Image.ALIGNMENT, main.HMI.AudioPrompt.ALIGNMENT) not in set_image_audio.calls
    assert len(pause_compressions.calls) == 1  # only the one-time setup call, never repeated per tick


# -------------------- flow: error-driven edges --------------------
# Each of these forces a specific state's own function to return the error,
# rather than forcing it generically from init_motor (as the fatal/kneel
# tests above do), to exercise that state's own if/continue handling of the
# error before the generic ERROR_STATE_MAP dispatch takes over.

def test_main_flow_battery_failure_during_startup_routes_to_abort(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["sensing"], "battery_check", lambda: ErrorCode.ERROR_LOW_BATTERY)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=5)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_flow_zeroing_failure_routes_to_abort(monkeypatch):
    """Timeout and max-extension both surface from actuation.zeroing() as the same
    ERROR_ZEROING_FAILURE code (see conversation); main.py just needs to route it."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ERROR_ZEROING_FAILURE)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_flow_sensor_failure_during_zeroing_routes_to_abort(monkeypatch):
    """ZEROING's loop checks `error not in (NORMAL, ZEROING_FINISHED)` before the
    generic dispatch -- a sensor failure must not be mistaken for either."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ERROR_SENSOR_FAILURE)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_flow_imu_kneel_during_zeroing_routes_to_kneel_failure(monkeypatch):
    """All three states that call read_sensors() (Zeroing, Compression, Pause) can
    reach KNEEL_FAILURE on an IMU-kneel code -- see the paired tests below for
    Compression and Pause. actuation.zeroing() passes any read_sensors() result
    through unmodified, so this is the simplest of the three to trigger."""
    modules = install_fake_main_modules(monkeypatch)
    set_image_audio = _cascade_fakes(modules, zeroing_result=ErrorCode.ERROR_IMU_KNEEL_FAILURE)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls


def test_main_flow_imu_kneel_during_compression_routes_to_kneel_failure(monkeypatch):
    """compressions() special-cases ERROR_IMU_KNEEL_FAILURE and passes it through,
    unlike other sensor failures which it converts to ERROR_SENSOR_FAILURE (see
    test_actuation.py for that distinction at the actuation layer)."""
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "compressions", lambda: ErrorCode.ERROR_IMU_KNEEL_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls


def test_main_flow_imu_kneel_during_pause_routes_to_kneel_failure(monkeypatch):
    """pause_compressions() special-cases ERROR_IMU_KNEEL_FAILURE the same way
    compressions() does (see test_actuation.py).

    pause_compressions() is called from both PAUSE's setup and KNEEL_FAILURE's
    own setup, so this fails it *persistently* (every call, not just the
    first from PAUSE) to prove KNEEL_FAILURE's setup tolerates seeing the
    same code again instead of re-routing into itself forever -- previously
    an infinite busy-loop that never reached time.sleep() (see conversation).
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "pause_compressions", lambda: ErrorCode.ERROR_IMU_KNEEL_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) in set_image_audio.calls
    # Proceeding past setup (Next -> ALIGNMENT) despite the persistent failure
    # is what actually proves this isn't a busy-loop, not just that the
    # screen eventually shows -- a livelocked run would never get here either.
    alignment_entries = set_image_audio.calls.count(
        (main.HMI.Image.ALIGNMENT, main.HMI.AudioPrompt.ALIGNMENT)
    )
    assert alignment_entries >= 2


def test_main_flow_other_pause_compressions_failure_routes_to_abort(monkeypatch):
    """The tolerance added to KNEEL_FAILURE's setup is specific to
    ERROR_IMU_KNEEL_FAILURE -- pause_compressions() failing with anything else
    (e.g. a motor fault) must still re-route via the generic ERROR_STATE_MAP
    dispatch to ABORT, not get swallowed the same way, and not loop forever
    either (PAUSE's own setup is what calls it here, but the check that
    matters -- ERROR_MOTOR_FAILURE not in (NORMAL, IMU_KNEEL) -- is identical
    if KNEEL_FAILURE's setup calls the same failing function instead).
    """
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "pause_compressions", lambda: ErrorCode.ERROR_MOTOR_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls
    assert (main.HMI.Image.KNEEL_FAILURE, main.HMI.AudioPrompt.KNEEL_FAILURE) not in set_image_audio.calls


def test_main_flow_sensor_failure_during_compression_routes_to_abort(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "compressions", lambda: ErrorCode.ERROR_SENSOR_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls


def test_main_flow_sensor_failure_during_pause_routes_to_abort(monkeypatch):
    modules = install_fake_main_modules(monkeypatch)
    setattr(modules["actuation"], "pause_compressions", lambda: ErrorCode.ERROR_SENSOR_FAILURE)
    set_image_audio = _cascade_fakes(modules)

    import main
    _run_main_for_ticks(monkeypatch, main, max_ticks=15)

    assert (main.HMI.Image.ABORT, main.HMI.AudioPrompt.ABORT) in set_image_audio.calls
