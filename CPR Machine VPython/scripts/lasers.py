"""
Toggle the alignment lasers on and off with the Enter key.

Starts with the lasers off. Each press of Enter flips them. Ctrl+C (or
typing q then Enter) turns them off and exits.

Standalone on purpose -- it drives the laser GPIO directly instead of
importing src/, so it runs without the rest of the hardware attached.

Usage:
    python lasers.py
"""

import subprocess

LASER_PIN: int = 12  # BCM numbering, same as HMI.py. High -> Q5 on -> lasers on


def set_lasers(on: bool) -> None:
    """Drive the laser GPIO. Uses pigpio like HMI.py; falls back to pinctrl
    (Pi 5, or pigpiod not running)."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_mode(LASER_PIN, pigpio.OUTPUT)
            pi.write(LASER_PIN, int(on))
            pi.stop()
            print(f"Lasers {'on' if on else 'off'} (pigpio)")
            return
    except ImportError:
        pass

    try:
        subprocess.run(["pinctrl", "set", str(LASER_PIN), "op", "dh" if on else "dl"], check=True)
        print(f"Lasers {'on' if on else 'off'} (pinctrl)")
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"Could not set lasers {'on' if on else 'off'}: {e}")


def main() -> None:
    lasers_on = False
    set_lasers(lasers_on)
    print("Press Enter to toggle the lasers. q + Enter or Ctrl+C to quit.")
    try:
        while True:
            if input().strip().lower() == "q":
                break
            lasers_on = not lasers_on
            set_lasers(lasers_on)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        set_lasers(False)


if __name__ == "__main__":
    main()
