# hmi.py

import pigpio
import pygame
from enum import Enum
from error_codes import ErrorCode

# TODO: Set actual GPIO pin numbers to match the hardware hat
NEXT_BTN_PIN  = 17
PAUSE_BTN_PIN = 27
NEXT_LED_PIN  = 22
PAUSE_LED_PIN = 23
LASER_PIN     = 24

class Image(Enum):
    # Enum values are the image file paths
    STARTUP          = "Images/startupBmp.jpg"          # TODO: add startup/911 image
    UNFOLD           = "Images/unfoldBmp.jpg"
    CUT_CLOTHES      = "Images/cutClothingBmp.jpg"
    ALIGNMENT        = "Images/alignmentBmp.jpg"
    ZEROING_PREP     = "Images/zeroingPrepBmp.jpg"
    ZEROING          = "Images/zeroingBmp.jpg"
    COMPRESSION_PREP = "Images/compressionsConfirmBmp.jpg"
    COMPRESSION      = "Images/compressionsBmp.jpg"
    PAUSE            = "Images/pausedBmp.jpg"
    ABORT            = "Images/abortBmp.jpg"
    KNEEL_FAILURE    = "Images/kneelFailureBmp.jpg"

class AudioPrompt(Enum):
    # Enum values are the audio file paths; empty string means no audio for that state
    STARTUP          = "Audio/startupWav.wav"           # TODO: add startup audio
    UNFOLD           = "Audio/unfoldExposeWav.wav"
    CUT_CLOTHES      = "Audio/cutClothingWav.wav"
    ALIGNMENT        = "Audio/alignmentWav.wav"
    ZEROING_PREP     = "Audio/zeroingPrepWav.wav"
    ZEROING          = "Audio/zeroingWav.wav"
    COMPRESSION_PREP = "Audio/compressionsPrepWav.wav"
    COMPRESSION      = "Audio/compressionsWav.wav"
    PAUSE            = ""   # TODO
    ABORT            = ""   # TODO
    KNEEL_FAILURE    = ""   # TODO

_screen = None
_pi     = None


def init_screens(pi_instance) -> ErrorCode:
    """pi_instance is the shared pigpio.pi() object from sensing.py."""
    global _screen, _pi
    _pi = pi_instance

    pygame.init()
    pygame.mixer.init(frequency=44100, channels=1, buffer=2048)

    info = pygame.display.Info()
    _screen = pygame.display.set_mode(
        (info.current_w, info.current_h),
        pygame.FULLSCREEN
    )
    pygame.display.set_caption("CPR Machine")

    # Buttons as inputs with pull-downs
    _pi.set_mode(NEXT_BTN_PIN,  pigpio.INPUT)
    _pi.set_mode(PAUSE_BTN_PIN, pigpio.INPUT)
    _pi.set_pull_up_down(NEXT_BTN_PIN,  pigpio.PUD_DOWN)
    _pi.set_pull_up_down(PAUSE_BTN_PIN, pigpio.PUD_DOWN)

    # LEDs and laser as outputs, start LOW
    for pin in (NEXT_LED_PIN, PAUSE_LED_PIN, LASER_PIN):
        _pi.set_mode(pin, pigpio.OUTPUT)
        _pi.write(pin, 0)

    return ErrorCode.NORMAL_OPERATION


def set_screen_image(image: Image):
    surf = pygame.image.load(image.value)
    surf = pygame.transform.scale(surf, _screen.get_size())
    _screen.blit(surf, (0, 0))
    pygame.display.flip()


def set_screen_audio(image: Image, prompt: AudioPrompt):
    """Sets the screen image and plays the audio prompt once. Call once on state entry."""
    set_screen_image(image)
    if prompt.value:
        pygame.mixer.Sound(prompt.value).play()


def audio_finished() -> bool:
    return not pygame.mixer.get_busy()


def enable_lasers():
    _pi.write(LASER_PIN, 1)

def disable_lasers():
    _pi.write(LASER_PIN, 0)


# The next and pause buttons each share a ground connection with their built-in LED.
# Enabling the LED is what physically allows the button to register a press.
# Always enable a button before expecting input from it, and disable it when input
# should not be accepted, to prevent unintended state transitions.

def enable_next_button():
    _pi.write(NEXT_LED_PIN, 1)

def disable_next_button():
    _pi.write(NEXT_LED_PIN, 0)

def enable_pause_button():
    _pi.write(PAUSE_LED_PIN, 1)

def disable_pause_button():
    _pi.write(PAUSE_LED_PIN, 0)


def next_button_pressed() -> bool:
    # pygame.event.pump() must be called regularly to keep the pygame window
    # responsive and prevent the OS from marking it as unresponsive
    pygame.event.pump()
    return bool(_pi.read(NEXT_BTN_PIN))

def pause_button_pressed() -> bool:
    pygame.event.pump()
    return bool(_pi.read(PAUSE_BTN_PIN))