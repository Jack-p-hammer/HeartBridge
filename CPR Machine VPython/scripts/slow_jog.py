"""
Slow jog test for the plunger motor, with optional alignment lasers.

Sequence (each step is one press of Enter):
    1st Enter: lower the plunger LOWER_IN inches and hold there
    2nd Enter: cycle down TRAVEL_IN inches and back up, over and over
    3rd Enter: stop the motor (stop/coast mode), turn off the lasers, exit

Standalone on purpose -- it talks to the moteus directly instead of importing
src/, so it runs without the force sensor / I2C hardware attached.

Usage:
    python slow_jog.py                     # lasers off, default snail's pace
    python slow_jog.py --laser on          # lasers on for the whole run
    python slow_jog.py --speed 0.5         # inches per second
"""

import argparse
import asyncio
import math
import subprocess
import time

import moteus

CONTROLLER_ID: int = 1  # same as moteus_thread.py
PINION_RADIUS_M: float = 0.01  # same as moteus_thread.py
MAX_TORQUE_NM: float = 2.0  # kept low for bench testing (main code allows 5.0)
COMMAND_PERIOD_S: float = 0.001  # 100 Hz, well under the moteus watchdog timeout
REPLY_TIMEOUT_S: float = 0.05  # give up on a reply well before the ~0.1 s watchdog
LASER_PIN: int = 12  # BCM numbering, same as HMI.py. High -> Q5 on -> lasers on

LOWER_IN: float = 1.0  # first move, positive = down on the rack
TRAVEL_IN: float = 2.0  # cycle stroke below the lowered position
SPEED_IN_PER_S: float = 0.1  # snail's pace


def inches_to_rev(inches: float) -> float:
    """Convert linear rack travel to motor revolutions through the pinion."""
    return inches * 0.0254 / (2.0 * math.pi * PINION_RADIUS_M)


def set_lasers(on: bool) -> None:
    """Drive the laser GPIO. Uses pigpio like HMI.py; falls back to pinctrl
    (Pi 5, or pigpiod not running). Never raises -- a laser problem shouldn't
    stop the motor test."""
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


async def run(speed_in_per_s: float, lower_in: float, travel_in: float, lasers_on: bool) -> None:
    controller = moteus.Controller(id=CONTROLLER_ID)
    await controller.set_stop()  # clear any latched fault before commanding
    set_lasers(lasers_on)

    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, input, f"Press Enter to lower {lower_in} in...")
    press = loop.run_in_executor(None, input, "Lowering. Press Enter to start cycling.\n")

    start_rev: float = (await controller.query()).values[moteus.Register.POSITION]
    lower_rev: float = inches_to_rev(lower_in)
    travel_rev: float = inches_to_rev(travel_in)
    speed_rev_s: float = inches_to_rev(abs(speed_in_per_s))
    base_rev: float = start_rev + lower_rev  # top of the cycle stroke
    lower_s: float = abs(lower_rev) / speed_rev_s
    period_s: float = 2.0 * abs(travel_rev) / speed_rev_s  # down and back up
    print(f"Start position {start_rev:.3f} rev, lowering to {base_rev:.3f} rev at {speed_rev_s:.3f} rev/s")

    stage: str = "lowering"  # -> "cycle_requested" -> "cycling"
    t0: float = time.monotonic()
    t_cycle: float = 0.0
    last_print: float = 0.0
    last_limit: int = 0
    sent: int = 0  # replies received since the last status line
    missed: int = 0  # total commands that got no reply

    try:
        while True:
            now: float = time.monotonic()
            if press.done():
                if stage != "lowering":
                    break  # 3rd Enter
                stage = "cycle_requested"
                press = loop.run_in_executor(None, input, "Press Enter to stop and turn off lasers.\n")
            # Don't start cycling until the 1" move has finished, so there's no jump
            if stage == "cycle_requested" and now - t0 >= lower_s:
                stage, t_cycle = "cycling", now
                print(f"Cycling {travel_in} in below {base_rev:.3f} rev")

            if stage == "cycling":
                # Triangle wave: base -> base + travel -> base, constant speed
                phase: float = ((now - t_cycle) % period_s) / period_s
                if phase < 0.5:
                    frac, direction = 2.0 * phase, 1.0
                else:
                    frac, direction = 2.0 - 2.0 * phase, -1.0
                target: float = base_rev + frac * travel_rev
                velocity: float = direction * math.copysign(speed_rev_s, travel_rev)
            elif now - t0 < lower_s:
                target = start_rev + lower_rev * (now - t0) / lower_s
                velocity = math.copysign(speed_rev_s, lower_rev)
            else:
                target, velocity = base_rev, 0.0  # hold at the lowered position

            # A lost CAN reply can make this await hang forever, which freezes the
            # script and lets the controller's watchdog stop the motor silently.
            try:
                result = await asyncio.wait_for(controller.set_position(
                    position=target,
                    velocity=velocity,
                    maximum_torque=MAX_TORQUE_NM,
                    query=True,
                ), timeout=REPLY_TIMEOUT_S)
            except asyncio.TimeoutError:
                missed += 1
                print(f"[{stage}] no reply from controller ({missed} missed so far)")
                continue
            sent += 1

            # Mode 10 = position control, 1 = fault, 11 = watchdog timeout.
            # In a running mode the fault register holds a limit code instead (96-103,
            # e.g. 102 = hit maximum_torque) -- that's informational, not a fault.
            mode = result.values[moteus.Register.MODE]
            fault = result.values[moteus.Register.FAULT]
            if mode != 1 and fault != last_limit:  # only print when the limit code changes
                last_limit = fault
                print(f"[{stage}] limit code {fault}{' (cleared)' if not fault else ''}  "
                      f"target {target:.3f}  pos {result.values[moteus.Register.POSITION]:.3f}")
            if now - last_print > 1.0:
                # cmds/s shows the loop is alive; torque near MAX_TORQUE_NM with pos lagging
                # target means the motor can't keep up
                print(f"[{stage}] target {target:.3f}  pos {result.values[moteus.Register.POSITION]:.3f}  "
                      f"torque {result.values[moteus.Register.TORQUE]:.2f} Nm  mode {mode}  fault {fault}  "
                      f"{result.values[moteus.Register.VOLTAGE]:.1f} V  {sent / (now - last_print):.0f} cmds/s")
                last_print, sent = now, 0
            if mode in (1, 11):
                print(f"Controller {'fault ' + str(fault) if mode == 1 else 'watchdog timeout'} "
                      "-- stopping (press Enter to exit). See moteus docs 'fault' register.")
                break
            await asyncio.sleep(COMMAND_PERIOD_S)
    finally:
        await controller.set_stop()
        print("Motor stopped.")
        set_lasers(False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-laser", "--laser", choices=("on", "off"), default="off", help="alignment lasers during the run")
    parser.add_argument("--speed", type=float, default=SPEED_IN_PER_S, help="inches per second")
    parser.add_argument("--lower", type=float, default=LOWER_IN, help="first move, inches (positive = down)")
    parser.add_argument("--travel", type=float, default=TRAVEL_IN, help="cycle stroke, inches (positive = down)")
    args = parser.parse_args()
    asyncio.run(run(args.speed, args.lower, args.travel, args.laser == "on"))


if __name__ == "__main__":
    main()
