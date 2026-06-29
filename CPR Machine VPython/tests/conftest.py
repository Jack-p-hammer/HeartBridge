import importlib
import sys
import types
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
SRC_STR = str(SRC.resolve())
if SRC_STR not in sys.path:
    sys.path.insert(0, SRC_STR)

_error_codes = importlib.import_module("error_codes")
ErrorCode = _error_codes.ErrorCode


class FakePi:
    """Small stand-in for the real pigpio object used by the HMI module."""

    def __init__(self, connected=True):
        self.connected = connected
        self.modes = {}
        self.pulls = {}
        self.writes = []
        self.reads = {}

    def set_mode(self, pin, mode):
        self.modes[pin] = mode

    def set_pull_up_down(self, pin, pull):
        self.pulls[pin] = pull

    def write(self, pin, value):
        self.writes.append((pin, value))

    def read(self, pin):
        return self.reads.get(pin, 0)


class FakeI2C:
    """Simple stand-in for the I2C bus used by the sensors."""

    def __init__(self, *args, **kwargs):
        pass


class FakeVL53L0X:
    """Simple stand-in for the VL53L0X sensor object."""

    def __init__(self, *args, **kwargs):
        pass


class FakeBNO08X_I2C:
    """Simple stand-in for the BNO08X sensor object."""

    def __init__(self, *args, **kwargs):
        pass

    def enable_feature(self, feature):
        return None


def install_fake_hardware_modules():
    """Install the fake hardware modules used by the application modules.

    This helper is used both by pytest fixtures and by individual tests that need
    to override one detail for a failure case.
    """
    pigpio_mod = types.ModuleType("pigpio")
    setattr(pigpio_mod, "INPUT", 0)
    setattr(pigpio_mod, "OUTPUT", 1)
    setattr(pigpio_mod, "PUD_UP", 2)
    setattr(pigpio_mod, "pi", lambda: FakePi())
    sys.modules["pigpio"] = pigpio_mod

    board_mod = types.ModuleType("board")
    setattr(board_mod, "SCL", "SCL")
    setattr(board_mod, "SDA", "SDA")
    sys.modules["board"] = board_mod

    busio_mod = types.ModuleType("busio")
    setattr(busio_mod, "I2C", FakeI2C)
    sys.modules["busio"] = busio_mod

    vl53_mod = types.ModuleType("adafruit_vl53l0x")
    setattr(vl53_mod, "VL53L0X", FakeVL53L0X)
    sys.modules["adafruit_vl53l0x"] = vl53_mod

    bno_mod = types.ModuleType("adafruit_bno08x")
    setattr(bno_mod, "BNO_REPORT_ACCELEROMETER", "accel")
    sys.modules["adafruit_bno08x"] = bno_mod

    bno_i2c_mod = types.ModuleType("adafruit_bno08x.i2c")
    setattr(bno_i2c_mod, "BNO08X_I2C", FakeBNO08X_I2C)
    sys.modules["adafruit_bno08x.i2c"] = bno_i2c_mod

    # The motor-driver library is not implemented yet, so the test environment
    # should simply make it importable without crashing.
    moteus_mod = types.ModuleType("moteus")
    sys.modules["moteus"] = moteus_mod

    # Remove any previously imported versions of the app modules so each test starts
    # from a clean state and sees the fake hardware modules we just installed.
    for name in ["sensing", "actuation", "states", "error_codes", "HMI", "multi_system", "main"]:
        sys.modules.pop(name, None)

    return None


@pytest.fixture
def fake_hardware_modules():
    """Provide the fake hardware environment used by tests.

    This fixture gives the code a pretend environment so we can test logic without
    needing the real robot hardware, sensors, or display attached.
    """
    return install_fake_hardware_modules()


def install_fake_main_modules(monkeypatch):
    """Install lightweight fake modules for state-machine tests in main.py."""
    sensing_mod = types.ModuleType("sensing")
    sensing_mod.__dict__["init_sensors"] = lambda: ErrorCode.NORMAL_OPERATION
    sensing_mod.__dict__["get_pi"] = lambda: FakePi()
    sensing_mod.__dict__["battery_check"] = lambda: ErrorCode.NORMAL_OPERATION
    monkeypatch.setitem(sys.modules, "sensing", sensing_mod)

    actuation_mod = types.ModuleType("actuation")
    actuation_mod.__dict__["init_motor"] = lambda: ErrorCode.NORMAL_OPERATION
    actuation_mod.__dict__["zeroing"] = lambda: ErrorCode.NORMAL_OPERATION
    actuation_mod.__dict__[
        "init_compressions"] = lambda: ErrorCode.NORMAL_OPERATION
    actuation_mod.__dict__["compressions"] = lambda: ErrorCode.NORMAL_OPERATION
    actuation_mod.__dict__["stop_compressions"] = lambda: None
    monkeypatch.setitem(sys.modules, "actuation", actuation_mod)

    hmi_mod = types.ModuleType("HMI")
    hmi_mod.__dict__["ErrorCode"] = ErrorCode
    hmi_mod.__dict__["Image"] = types.SimpleNamespace(
        STARTUP="startup",
        UNFOLD="unfold",
        ALIGNMENT="alignment",
        ZEROING_PREP="zeroing_prep",
        ZEROING="zeroing",
        COMPRESSION_PREP="compression_prep",
        COMPRESSION="compression",
        PAUSE="pause",
        ABORT="abort",
        KNEEL_FAILURE="kneel_failure",
    )
    hmi_mod.__dict__["AudioPrompt"] = types.SimpleNamespace(
        STARTUP="startup_prompt",
        UNFOLD="unfold_prompt",
        ALIGNMENT="alignment_prompt",
        ZEROING_PREP="zeroing_prep_prompt",
        ZEROING="zeroing_prompt",
        COMPRESSION_PREP="compression_prep_prompt",
        COMPRESSION="compression_prompt",
        PAUSE="",
        ABORT="",
        KNEEL_FAILURE="",
    )
    hmi_mod.__dict__[
        "init_HMI"] = lambda pi_instance: ErrorCode.NORMAL_OPERATION
    hmi_mod.__dict__["set_screen_audio"] = lambda image, prompt: None
    hmi_mod.__dict__["enable_next_button"] = lambda: None
    hmi_mod.__dict__["disable_next_button"] = lambda: None
    hmi_mod.__dict__["enable_pause_button"] = lambda: None
    hmi_mod.__dict__["disable_pause_button"] = lambda: None
    hmi_mod.__dict__["enable_lasers"] = lambda: None
    hmi_mod.__dict__["disable_lasers"] = lambda: None
    hmi_mod.__dict__["next_button_pressed"] = lambda: False
    hmi_mod.__dict__["pause_button_pressed"] = lambda: False
    hmi_mod.__dict__["audio_finished"] = lambda: True
    monkeypatch.setitem(sys.modules, "HMI", hmi_mod)

    multi_system_mod = types.ModuleType("multi_system")
    monkeypatch.setitem(sys.modules, "multi_system", multi_system_mod)

    return {"sensing": sensing_mod, "actuation": actuation_mod, "HMI": hmi_mod, "multi_system": multi_system_mod}


def install_fake_pygame(monkeypatch):
    """Install a minimal pygame module so HMI code can run without a GUI window."""
    pygame_mod = types.ModuleType("pygame")
    setattr(pygame_mod, "FULLSCREEN", 0)
    setattr(pygame_mod, "init", lambda: None)
    setattr(
        pygame_mod,
        "mixer",
        types.SimpleNamespace(init=lambda *args, **
                              kwargs: None, get_busy=lambda: False),
    )
    setattr(
        pygame_mod,
        "display",
        types.SimpleNamespace(
            Info=lambda: types.SimpleNamespace(current_w=800, current_h=600),
            set_mode=lambda *args, **kwargs: object(),
            set_caption=lambda *args, **kwargs: None,
            flip=lambda: None,
        ),
    )
    setattr(pygame_mod, "image", types.SimpleNamespace(
        load=lambda path: types.SimpleNamespace()))
    setattr(pygame_mod, "transform", types.SimpleNamespace(
        scale=lambda surf, size: surf))
    setattr(pygame_mod, "Surface", object)
    setattr(pygame_mod, "event", types.SimpleNamespace(pump=lambda: None))
    monkeypatch.setitem(sys.modules, "pygame", pygame_mod)


def install_fake_pigpio(monkeypatch, connected=True):
    """Install a minimal pigpio module so HMI code can use GPIO-like calls.

    The optional connected argument makes it easy to simulate both a healthy
    connection and a failed one.
    """
    pigpio_mod = types.ModuleType("pigpio")
    setattr(pigpio_mod, "INPUT", 0)
    setattr(pigpio_mod, "OUTPUT", 1)
    setattr(pigpio_mod, "PUD_UP", 2)
    setattr(pigpio_mod, "pi", lambda: FakePi(connected=connected))
    monkeypatch.setitem(sys.modules, "pigpio", pigpio_mod)
