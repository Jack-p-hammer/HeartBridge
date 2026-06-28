# hmi.py

import pigpio
from pathlib import Path
import pygame
import logging
from enum import Enum
from error_codes import ErrorCode

# TODO: Set actual GPIO pin numbers to match the hardware hat
# Button Inputs
NEXT_BTN_PIN  = 17
PAUSE_BTN_PIN = 27
# LED Outputs/Button Toggles
NEXT_LED_PIN  = 22
PAUSE_LED_PIN = 23
# Laser PWM Output
LASER_PIN     = 24

# Declare paths for images and audio files relative to this script's directory
IMAGES = Path(__file__).resolve().parent / "Images"
AUDIO = Path(__file__).resolve().parent / "Audio"

class Image(Enum):
    # Enum values are the image file paths
    STARTUP          = ""          # TODO: add startup/911 image
    UNFOLD           = IMAGES / "unfold.jpg"
    CUT_CLOTHES      = IMAGES / "cutClothing.jpg"
    ALIGNMENT        = IMAGES / "alignment.jpg"
    ZEROING_PREP     = IMAGES / "zeroingPrep.jpg"
    ZEROING          = ""          # TODO: Find zeroing image
    COMPRESSION_PREP = IMAGES / "compressionsConfirm.jpg"
    COMPRESSION      = IMAGES / "compressions.jpg"
    PAUSE            = IMAGES / "paused.jpg"
    ABORT            = IMAGES / "abort.jpg"
    KNEEL_FAILURE    = IMAGES / "kneelFailure.jpg"

class AudioPrompt(Enum):
    # Enum values are the audio file paths; empty string means no audio for that state
    STARTUP          = AUDIO / "startup.wav"           
    UNFOLD           = AUDIO / "unfoldExpose.wav"
    CUT_CLOTHES      = AUDIO / "cutClothing.wav"
    ALIGNMENT        = AUDIO / "alignment.wav"
    ZEROING_PREP     = AUDIO / "zeroingPrep.wav"
    ZEROING          = AUDIO / "zeroing.wav"
    COMPRESSION_PREP = AUDIO / "compressionsPrep.wav"
    COMPRESSION      = AUDIO / "compressions.wav"
    PAUSE            = ""   # TODO
    ABORT            = ""   # TODO
    KNEEL_FAILURE    = ""   # TODO

# Global variables for shared instances, _pi is declared in sensing.py and passed to HMI.py for shared GPIO access
_screen: pygame.Surface
_pi: pigpio.pi


def init_HMI(pi_instance: pigpio.pi) -> ErrorCode:
    """Initialize screens, audio, lasers, and buttons.

    Args:
        pi_instance: the shared pigpio.pi() object from sensing.py.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    
    # Initialize the global variables for screens and pigpio instance
    global _screen, _pi
    _pi = pi_instance

    # Initialize Pygame and audio mixer for sound playback
    try:
        pygame.init()
        pygame.mixer.init(frequency=44100, channels=1, buffer=2048)
    except Exception as e:
        logging.error(f"Pygame initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # Initialize the display in fullscreen mode
    try:
        info = pygame.display.Info()
        _screen = pygame.display.set_mode(
            (info.current_w, info.current_h),
            pygame.FULLSCREEN
        )
        pygame.display.set_caption("CPR Machine")
    except Exception as e:
        logging.error(f"Display initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    # Initialize GPIO pins for buttons, LEDs, and lasers
    try:
        # Buttons as inputs with pull-downs
        _pi.set_mode(NEXT_BTN_PIN,  pigpio.INPUT)
        _pi.set_mode(PAUSE_BTN_PIN, pigpio.INPUT)
        _pi.set_pull_up_down(NEXT_BTN_PIN,  pigpio.PUD_UP) # Buttons are active low, so use pull-up resistors
        _pi.set_pull_up_down(PAUSE_BTN_PIN, pigpio.PUD_UP)

        # LEDs and laser as outputs, start LOW
        for pin in (NEXT_LED_PIN, PAUSE_LED_PIN, LASER_PIN):
            _pi.set_mode(pin, pigpio.OUTPUT)
            _pi.write(pin, 0)
    except Exception as e:
        logging.error(f"HMI GPIO initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION


def set_screen_image(image: Image):
    """Set image to display on screens

    Args:
        image (Image): Image enum for current state
    """
    surf = pygame.image.load(image.value)
    surf = pygame.transform.scale(surf, _screen.get_size())
    _screen.blit(surf, (0, 0))
    pygame.display.flip()


def set_screen_audio(image: Image, prompt: AudioPrompt):
    """Sets the screen image and plays the audio prompt once. Call once on state entry.
    
    Args:
        image (Image): Image enum for current state
        prompt (AudioPrompt): Audio prompt enum for current state
    """
    set_screen_image(image)
    
    # TODO: Implement looping audio
    if prompt.value:
        pygame.mixer.Sound(prompt.value).play()


def enable_lasers():
    """Enables alignment lasers
    """
    # TODO: Implement PWM
    _pi.write(LASER_PIN, 1)


def disable_lasers():
    """Disables alignment lasers
    """
    _pi.write(LASER_PIN, 0)
    
def audio_finished() -> bool:
    """Check if audio playback has finished

    Returns:
        bool: True if audio finished, False otherwise
    """
    # TODO: Change such that returns if audio loop finished once, not if audio is still playing
    return not pygame.mixer.get_busy()


# The next and pause buttons each share a ground connection with their built-in LED.
# Enabling the LED is what physically allows the button to register a press.
# Always enable a button before expecting input from it, and disable it when input
# should not be accepted, to prevent unintended state transitions.


def enable_next_button():
    """Enable Next button and Next button LED
    """
    _pi.write(NEXT_LED_PIN, 1)


def disable_next_button():
    """Disable Next button and Next button LED
    """
    _pi.write(NEXT_LED_PIN, 0)


def enable_pause_button():
    """Enable Pause button and Pause button LED
    """
    _pi.write(PAUSE_LED_PIN, 1)


def disable_pause_button():
    """Disable Pause button and Pause button LED
    """
    _pi.write(PAUSE_LED_PIN, 0)


def next_button_pressed() -> bool:
    """Return whether the next button is currently being pressed. Non-blocking.

    Returns:
        bool: True if button press detected, False otherwise
    """
    # pygame.event.pump() must be called regularly to keep the pygame window
    # responsive and prevent the OS from marking it as unresponsive
    pygame.event.pump()
    return bool(_pi.read(NEXT_BTN_PIN))


def pause_button_pressed() -> bool:
    """Return whether the pause button is currently being pressed. Non-blocking.

    Returns:
        bool: True if button press detected, False otherwise
    """
    pygame.event.pump()
    return bool(_pi.read(PAUSE_BTN_PIN))