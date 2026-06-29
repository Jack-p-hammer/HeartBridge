import importlib
from typing import Any, cast

from conftest import FakePi, install_fake_pigpio, install_fake_pygame
from error_codes import ErrorCode


def test_hmi_init_sets_up_gpio_and_display(monkeypatch):
    """Exercise the HMI startup path with fake pygame and pigpio objects."""
    install_fake_pygame(monkeypatch)
    install_fake_pigpio(monkeypatch)

    import HMI

    fake_pi = FakePi()

    # HMI imports pygame and pigpio when it is imported, so reload it after the
    # fake modules are in place.
    importlib.reload(HMI)
    result = HMI.init_HMI(fake_pi)  # type: ignore[arg-type]

    assert result == ErrorCode.NORMAL_OPERATION
    assert fake_pi.modes[17] == 0
    assert fake_pi.modes[27] == 0
    assert fake_pi.modes[22] == 1
    assert fake_pi.writes[0] == (22, 0)


def test_hmi_button_helpers_write_expected_gpio(monkeypatch):
    """Verify the HMI button helpers issue the expected GPIO writes."""
    install_fake_pygame(monkeypatch)
    install_fake_pigpio(monkeypatch)

    import HMI

    fake_pi = FakePi()
    importlib.reload(HMI)
    HMI.init_HMI(fake_pi)  # type: ignore[arg-type]

    HMI.enable_next_button()
    HMI.disable_next_button()
    HMI.enable_pause_button()
    HMI.disable_pause_button()

    assert fake_pi.writes[-4:] == [(22, 1), (22, 0), (23, 1), (23, 0)]


def test_hmi_init_returns_error_when_pygame_init_fails(monkeypatch):
    """Ensure HMI initialization returns an error code when pygame startup fails."""
    install_fake_pygame(monkeypatch)
    install_fake_pigpio(monkeypatch)
    import pygame

    def fail_init():
        raise RuntimeError("pygame unavailable")

    monkeypatch.setattr(pygame, "init", fail_init)

    import HMI

    importlib.reload(HMI)

    assert HMI.init_HMI(cast(Any, FakePi())) == ErrorCode.ERROR_INIT_FAILURE


def test_hmi_button_readers_and_lasers_use_current_gpio_state(monkeypatch):
    """Ensure the HMI helpers read button state and toggle the laser pin correctly."""
    install_fake_pygame(monkeypatch)
    install_fake_pigpio(monkeypatch)

    import HMI

    fake_pi = FakePi()
    fake_pi.reads[17] = 1
    fake_pi.reads[27] = 0

    importlib.reload(HMI)
    HMI.init_HMI(fake_pi)  # type: ignore[arg-type]

    assert HMI.next_button_pressed() is True
    assert HMI.pause_button_pressed() is False

    HMI.enable_lasers()
    HMI.disable_lasers()

    assert fake_pi.writes[-2:] == [(24, 1), (24, 0)]


def test_hmi_audio_finished_reports_mixer_state(monkeypatch):
    """Ensure the audio-finished helper reflects the current mixer state."""
    install_fake_pygame(monkeypatch)
    install_fake_pigpio(monkeypatch)

    import HMI

    importlib.reload(HMI)

    assert HMI.audio_finished() is True
