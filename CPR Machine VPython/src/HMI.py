# hmi.py
"""Library of functions for Human-Machine Interface (HMI) operations, including screen display, audio prompts, button handling, and laser control.
"""
# External imports
import pigpio
from pathlib import Path
import pygame
import logging
import time
from enum import Enum

# Internal imports
from Enums.error_codes import ErrorCode

# TODO: Set actual GPIO pin numbers to match the hardware hat
# Button Inputs
NEXT_BTN_PIN = 23
PAUSE_BTN_PIN = 25
# LED Outputs/Button Toggles
NEXT_ENABLE_PIN = 24
PAUSE_ENABLE_PIN = 26

# Laser PWM Output
LASER_PIN = 12
# Duty Cycle Scale: 0-1M
LASER_PWM_SCALE: int = 1000000
# For now, 25 kHz PWM
LASER_PWM_FREQUENCY:int = 25000
LASER_PWM_DUTY_CYCLE: float = 0.5  # 50% duty cycle for now

# Declare paths for images and audio files relative to this script's directory
IMAGES = Path(__file__).resolve().parent / "Images"
AUDIO = Path(__file__).resolve().parent / "Audio"


class Image(Enum):
    # Enum values are the image file paths
    STARTUP = ""          # TODO: add startup/911 image
    UNFOLD = IMAGES / "unfold.jpg"
    CUT_CLOTHES = IMAGES / "cutClothing.jpg"
    ALIGNMENT = IMAGES / "alignment.jpg"
    ZEROING_PREP = IMAGES / "zeroingPrep.jpg"
    ZEROING = ""          # TODO: Find zeroing image
    COMPRESSION_PREP = IMAGES / "compressionsConfirm.jpg"
    COMPRESSION = IMAGES / "compressions.jpg"
    PAUSE = IMAGES / "paused.jpg"
    ABORT = IMAGES / "abort.jpg"
    KNEEL_FAILURE = IMAGES / "kneelFailure.jpg"


class AudioPrompt(Enum):
    # Enum values are the audio file paths; empty string means no audio for that state
    STARTUP = AUDIO / "startup.wav"
    UNFOLD = AUDIO / "unfoldExpose.wav"
    CUT_CLOTHES = AUDIO / "cutClothing.wav"
    ALIGNMENT = AUDIO / "alignment.wav"
    ZEROING_PREP = AUDIO / "zeroingPrep.wav"
    ZEROING = AUDIO / "zeroing.wav"
    COMPRESSION_PREP = AUDIO / "compressionsPrep.wav"
    COMPRESSION = AUDIO / "compressions.wav"
    PAUSE = ""   # TODO
    ABORT = ""   # TODO
    KNEEL_FAILURE = ""   # TODO


# Global variables for shared instances, _pi is declared in sensing.py and passed to HMI.py for shared GPIO access
_screen: pygame.Surface
_pi: pigpio.pi

# Audio playback state for user prompting
_audio_channel: pygame.mixer.Channel
_audio_cache: dict = {}       # AudioPrompt -> loaded pygame.mixer.Sound
_audio_start_time: float = 0.0    # time.monotonic() when current audio loop started
_audio_length: float = 0.0    # length in seconds of the currently playing prompt


def init_HMI(pi_instance: pigpio.pi) -> ErrorCode:
    """Initialize screens, audio, lasers, and buttons.

    Args:
        pi_instance: the shared pigpio.pi() object from sensing.py.

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    global _screen, _pi, _audio_channel
    logging.debug("Initializing HMI...")
    
    # Initialize the global variable for the pigpio instance
    _pi = pi_instance

    # Initialize Pygame and audio mixer for sound playback
    try:
        pygame.init()
        pygame.mixer.init(frequency=44100, channels=1, buffer=2048)
        # Reserve a dedicated channel for audio prompts
        _audio_channel = pygame.mixer.Channel(0)
    except Exception as e:
        logging.error(f"Pygame initialization failed: {e}")
        return ErrorCode.ERROR_PYGAME_INIT_FAILURE

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
        # Buttons as inputs with pull-ups
        _pi.set_mode(NEXT_BTN_PIN,  pigpio.INPUT)
        _pi.set_mode(PAUSE_BTN_PIN, pigpio.INPUT)
        _pi.set_pull_up_down(NEXT_BTN_PIN,  pigpio.PUD_UP)
        _pi.set_pull_up_down(PAUSE_BTN_PIN, pigpio.PUD_UP)

        # LEDs and laser as outputs, start LOW
        for pin in (NEXT_ENABLE_PIN, PAUSE_ENABLE_PIN, LASER_PIN):
            _pi.set_mode(pin, pigpio.OUTPUT)
            _pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
        
    except Exception as e:
        logging.error(f"HMI GPIO initialization failed: {e}")
        return ErrorCode.ERROR_INIT_FAILURE
    
    try:            
        # Initialize laser PWM
        # Duty cycle ranges from 0-1M
        # Initialize to off
        error_num = _pi.hardware_PWM(LASER_PIN, 0, 0*LASER_PWM_SCALE)
        if error_num < 0:
            logging.error(f"Failed to initialize laser PWM: {error_num}")
            return ErrorCode.ERROR_INIT_FAILURE
    except Exception as e:
        logging.error(f"Failed to communicate with pigpio daemon: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE

    return ErrorCode.NORMAL_OPERATION


def pump_events() -> ErrorCode:
    """Service the pygame event queue. Must be called exactly once per main
    loop tick, unconditionally, regardless of state
    
    Returns:
        ErrorCode: NORMAL_OPERATION if successful, WARNING_PYGAME_PUMP_FAILURE if failed
    """
    try:
        pygame.event.pump()
    except Exception as e:
        # Pump should only fail if pygame is uninitialized, which should never happen after init_HMI() succeeds
        logging.warning(f"Pygame event pump failed: {e}\n\tThis should happen once and only once, before init_HMI() is called.")
        return ErrorCode.WARNING_PYGAME_PUMP_FAILURE
    return ErrorCode.NORMAL_OPERATION


def set_image(image: Image) -> ErrorCode:
    """Set image to display on screens

    Args:
        image (Image): Image enum for current state
    
    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_UNKNOWN_IMAGE if failed
    """
    global _screen
    try:
        surf = pygame.image.load(image.value)
        surf = pygame.transform.scale(surf, _screen.get_size())
        _screen.blit(surf, (0, 0))
        pygame.display.flip()
    except Exception as e:
        logging.error(f"Failed to set image {image.name}: {e}")
        return ErrorCode.ERROR_UNKNOWN_IMAGE
    return ErrorCode.NORMAL_OPERATION


def set_audio(prompt: AudioPrompt) -> ErrorCode:
    """Play audio prompt on loop. Call once on state entry; restarts playback
    from the beginning even if called again with the same prompt.

    Args:
        prompt (AudioPrompt): Audio prompt enum for current state
        
    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PYGAME_FAILURE if failed
    """
    global _audio_channel, _audio_cache, _audio_start_time, _audio_length

    # Some states have no audio (empty path); stop whatever was playing
    if not prompt.value:
        _audio_channel.stop()
        _audio_length = 0.0
        return ErrorCode.NORMAL_OPERATION

    # Load and cache each prompt's audio once instead of hitting disk every call
    if prompt not in _audio_cache:
        try:
            _audio_cache[prompt] = pygame.mixer.Sound(str(prompt.value))
        except Exception as e:
            logging.error(f"Failed to load audio prompt {prompt.name}: {e}")
            return ErrorCode.ERROR_PYGAME_FAILURE
    prompt_audio = _audio_cache[prompt]

    # Channel.play() always restarts from the beginning, so this swaps
    # cleanly to a new prompt and restarts on a repeated one
    try:
        _audio_channel.play(prompt_audio, loops=-1)
        _audio_start_time = time.monotonic()
        _audio_length = prompt_audio.get_length()
    except Exception as e:
        logging.error(f"Failed to play audio prompt {prompt.name}: {e}")
        return ErrorCode.ERROR_PYGAME_FAILURE
    
    return ErrorCode.NORMAL_OPERATION


def set_image_audio(image: Image, prompt: AudioPrompt) -> ErrorCode:
    """Set both image and audio prompt for current state. Call once on state entry.

    Args:
        image (Image): Image enum for current state
        prompt (AudioPrompt): Audio prompt enum for current state
    
    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_UNKNOWN_IMAGE or ERROR_PYGAME_FAILURE if failed
    """
    error = set_image(image)
    if error != ErrorCode.NORMAL_OPERATION:
        return error
    return set_audio(prompt)
    


def enable_lasers() -> ErrorCode:
    """Enables alignment lasers

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi

    try:
        # Enable lasers by giving nonzero duty cycle and PWM frequency
        error_num = _pi.hardware_PWM(LASER_PIN, LASER_PWM_FREQUENCY, int(LASER_PWM_DUTY_CYCLE * LASER_PWM_SCALE))
        if error_num < 0:
            logging.error(f"Failed to enable lasers: {error_num}")
            return ErrorCode.ERROR_PI_DAEMON_FAILURE
    except Exception as e:
        logging.error(f"Failed to communicate with pigpio daemon: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def disable_lasers() -> ErrorCode:
    """Disables alignment lasers

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi

    try:
        # Disable lasers by setting duty cycle and PWM frequency to zero
        error_num = _pi.hardware_PWM(LASER_PIN, 0, 0*LASER_PWM_SCALE)
        if error_num < 0:
            logging.error(f"Failed to disable lasers: {error_num}")
            return ErrorCode.ERROR_PI_DAEMON_FAILURE
    except Exception as e:
        logging.error(f"Failed to communicate with pigpio daemon: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def audio_finished() -> bool:
    """Check if the current audio prompt has completed its first loop.
    Playback continues looping regardless; this only reports elapsed time.

    Returns:
        bool: True once one full loop has elapsed (or no audio is set), False otherwise
    """
    global _audio_start_time, _audio_length

    if _audio_length == 0.0:
        return True
    return (time.monotonic() - _audio_start_time) >= _audio_length


# The next and pause buttons each share a ground connection with their built-in LED.
# Enabling the LED is what physically allows the button to register a press.
# Always enable a button before expecting input from it, and disable it when input
# should not be accepted, to prevent unintended state transitions.


def enable_next_button() -> ErrorCode:
    """Enable Next button and Next button LED

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi
    
    try:
        _pi.write(NEXT_ENABLE_PIN, 1)
    except Exception as e:
        logging.error(f"Failed to enable Next button: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def disable_next_button() -> ErrorCode:
    """Disable Next button and Next button LED

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi
    
    try:
        _pi.write(NEXT_ENABLE_PIN, 0)
    except Exception as e:
        logging.error(f"Failed to disable Next button: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def enable_pause_button() -> ErrorCode:
    """Enable Pause button and Pause button LED

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi
    
    try:
        _pi.write(PAUSE_ENABLE_PIN, 1)
    except Exception as e:
        logging.error(f"Failed to enable Pause button: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def disable_pause_button() -> ErrorCode:
    """Disable Pause button and Pause button LED

    Returns:
        ErrorCode: NORMAL_OPERATION if successful, ERROR_PI_DAEMON_FAILURE if failed
    """
    global _pi
    
    try:
        _pi.write(PAUSE_ENABLE_PIN, 0)
    except Exception as e:
        logging.error(f"Failed to disable Pause button: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE
    return ErrorCode.NORMAL_OPERATION


def next_button_pressed() -> tuple[ErrorCode, bool]:
    """Return state of next button. Non-blocking.

    Returns:
        tuple[ErrorCode, bool]: A tuple containing the error code and the button state
    """
    global _pi
    
    try:
        read_val = bool(_pi.read(NEXT_BTN_PIN))
    except Exception as e:
        logging.error(f"Failed to read Next button state: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE, False

    return ErrorCode.NORMAL_OPERATION, read_val

def pause_button_pressed() -> tuple[ErrorCode, bool]:
    """Return state of pause button. Non-blocking.

    Returns:
        tuple[ErrorCode, bool]: A tuple containing the error code and the button state
    """
    global _pi
    
    try:
        read_val = bool(_pi.read(PAUSE_BTN_PIN))
    except Exception as e:
        logging.error(f"Failed to read Pause button state: {e}")
        return ErrorCode.ERROR_PI_DAEMON_FAILURE, False

    return ErrorCode.NORMAL_OPERATION, read_val
