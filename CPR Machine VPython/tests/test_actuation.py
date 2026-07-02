import sys
import types

from Enums.error_codes import ErrorCode


def test_actuation_placeholders_return_normal_operation(monkeypatch):
    """Check the current placeholder behavior for motor-related functions.

    These functions are still stubs, so the test documents the current expectation:
    they should report that everything is operating normally.
    """
    moteus_mod = types.ModuleType("moteus")
    monkeypatch.setitem(sys.modules, "moteus", moteus_mod)

    import actuation

    assert actuation.init_motor() == ErrorCode.NORMAL_OPERATION
    assert actuation.zeroing() == ErrorCode.NORMAL_OPERATION
    assert actuation.init_compressions() == ErrorCode.NORMAL_OPERATION
    assert actuation.compressions() == ErrorCode.NORMAL_OPERATION
    assert actuation.stop_compressions() == ErrorCode.NORMAL_OPERATION


def test_actuation_placeholder_results_are_members_of_error_code(monkeypatch):
    """Ensure the placeholder functions return enum members rather than raw integers."""
    moteus_mod = types.ModuleType("moteus")
    monkeypatch.setitem(sys.modules, "moteus", moteus_mod)

    import actuation

    for func in [
        actuation.init_motor,
        actuation.zeroing,
        actuation.init_compressions,
        actuation.compressions,
        actuation.stop_compressions,
    ]:
        result = func()
        assert isinstance(result, ErrorCode)
        assert result == ErrorCode.NORMAL_OPERATION
