import importlib
import sys

from conftest import install_fake_hardware_modules, install_fake_pigpio

def test_sensing_init_returns_error_when_pigpio_connection_fails(monkeypatch):
    """Ensure sensor initialization fails gracefully when pigpio is unavailable.

    This uses the shared helper with a failed connection so the test can cover the
    error path without needing the real hardware.
    """
    # Use the shared helper first, then override the pigpio connection to simulate
    # a failed initialization path.
    install_fake_hardware_modules()
    install_fake_pigpio(monkeypatch, connected=False)

    import sensing

    importlib.reload(sensing)

    assert sensing.init_sensors() == sensing.ErrorCode.ERROR_INIT_FAILURE


def test_sensing_init_returns_normal_when_hardware_is_available(fake_hardware_modules):
    """Verify the sensing module succeeds with the fake hardware stack in place."""
    import sensing

    importlib.reload(sensing)

    assert sensing.init_sensors() == sensing.ErrorCode.NORMAL_OPERATION


def test_sensing_init_returns_error_when_i2c_initialization_fails(monkeypatch):
    """Ensure sensor initialization fails cleanly when the I2C bus cannot be created."""
    install_fake_hardware_modules()

    busio_mod = sys.modules["busio"]

    def fail_i2c(*args, **kwargs):
        raise RuntimeError("i2c unavailable")

    monkeypatch.setattr(busio_mod, "I2C", fail_i2c)

    import sensing

    importlib.reload(sensing)

    assert sensing.init_sensors() == sensing.ErrorCode.ERROR_INIT_FAILURE


def test_sensor_helper_functions_are_placeholder_safe():
    """Confirm the existing sensor helper functions behave as placeholders."""
    import sensing

    assert sensing.battery_check() == sensing.ErrorCode.NORMAL_OPERATION
    assert sensing.read_sensors_zeroing() == sensing.ErrorCode.NORMAL_OPERATION
    assert sensing.read_sensors_compressions() == sensing.ErrorCode.NORMAL_OPERATION
    assert sensing.check_sensor_error(1) == sensing.ErrorCode.NORMAL_OPERATION
