"""
Toggle the alignment lasers each time you press Enter.

    Enter      -> flip lasers (off -> on -> off -> ...)
    q + Enter  -> turn lasers off and exit
    Ctrl-C     -> same as quit

Standalone: no moteus or force sensor needed, just the laser GPIO.

Usage:
    python laser_toggle.py
"""

import subprocess

LASER_PIN: int = 12  # BCM numbering, same as HMI.py / slow_jog.py. High -> lasers on


def set_lasers(on: bool) -> None:
    """Drive the laser GPIO. Uses pigpio like HMI.py; falls back to pinctrl
    (Pi 5, or pigpiod not running). Never raises."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_mode(LASER_PIN, pigpio.OUTPUT)
            pi.write(LASER_PIN, int(on))
            pi.stop()
            print(f"Lasers {'ON' if on else 'OFF'} (pigpio)")
            return
    except ImportError:
        pass

    try:
        subprocess.run(["pinctrl", "set", str(LASER_PIN), "op", "dh" if on else "dl"], check=True)
        print(f"Lasers {'ON' if on else 'OFF'} (pinctrl)")
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"Could not set lasers {'on' if on else 'off'}: {e}")


def main() -> None:
    lasers_on: bool = False
    set_lasers(False)  # start from a known state
    try:
        while True:
            text = input("Press Enter to toggle lasers (q + Enter to quit)... ")
            if text.strip().lower() == "q":
                break
            lasers_on = not lasers_on
            set_lasers(lasers_on)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        set_lasers(False)


if __name__ == "__main__":
    main()
