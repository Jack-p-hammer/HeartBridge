"""
Slow back-and-forth jog test for the plunger motor.

Moves the rack TRAVEL_IN inches from wherever it is when started and back
again, over and over, at SPEED_IN_PER_S. Press Enter to start moving, then
Enter again to stop (motor is put in stop/coast mode).

Standalone on purpose -- it talks to the moteus directly instead of importing
src/, so it runs without the force sensor / I2C hardware attached.

Usage:
    python slow_jog.py              # default snail's pace
    python slow_jog.py --speed 0.5  # inches per second
"""

import argparse
import asyncio
import math
import time

import moteus

CONTROLLER_ID: int = 1  # same as moteus_thread.py
PINION_RADIUS_M: float = 0.01  # same as moteus_thread.py
MAX_TORQUE_NM: float = 2.0  # kept low for bench testing (main code allows 5.0)
COMMAND_PERIOD_S: float = 0.01  # 100 Hz, well under the moteus watchdog timeout

TRAVEL_IN: float = 2.0  # positive = down on the rack, negative = up
SPEED_IN_PER_S: float = 0.1  # snail's pace


def inches_to_rev(inches: float) -> float:
    """Convert linear rack travel to motor revolutions through the pinion."""
    return inches * 0.0254 / (2.0 * math.pi * PINION_RADIUS_M)


async def run(speed_in_per_s: float, travel_in: float) -> None:
    controller = moteus.Controller(id=CONTROLLER_ID)
    await controller.set_stop()  # clear any latched fault before commanding

    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, input, "Press Enter to start moving...")
    stop_pressed = loop.run_in_executor(None, input, "Moving. Press Enter to stop.\n")

    start_rev: float = (await controller.query()).values[moteus.Register.POSITION]
    travel_rev: float = inches_to_rev(travel_in)
    speed_rev_s: float = inches_to_rev(abs(speed_in_per_s))
    period_s: float = 2.0 * abs(travel_rev) / speed_rev_s  # out and back
    t0: float = time.monotonic()
    last_print: float = 0.0
    print(f"Start position {start_rev:.3f} rev, travel {travel_rev:.3f} rev at {speed_rev_s:.3f} rev/s")

    try:
        while not stop_pressed.done():
            # Triangle wave: start -> start + travel -> start, constant speed
            phase: float = ((time.monotonic() - t0) % period_s) / period_s
            if phase < 0.5:
                frac, direction = 2.0 * phase, 1.0
            else:
                frac, direction = 2.0 - 2.0 * phase, -1.0

            target: float = start_rev + frac * travel_rev
            result = await controller.set_position(
                position=target,
                velocity=direction * math.copysign(speed_rev_s, travel_rev),
                maximum_torque=MAX_TORQUE_NM,
                query=True,
            )

            # Mode 10 = position control. Anything else (esp. 1 = fault) means it isn't moving.
            mode = result.values[moteus.Register.MODE]
            fault = result.values[moteus.Register.FAULT]
            if time.monotonic() - last_print > 1.0:
                last_print = time.monotonic()
                print(f"target {target:.3f}  pos {result.values[moteus.Register.POSITION]:.3f}  "
                      f"mode {mode}  fault {fault}  {result.values[moteus.Register.VOLTAGE]:.1f} V")
            if fault:
                print(f"Controller fault {fault} -- stopping (press Enter to exit). See moteus docs 'fault' register.")
                break
            await asyncio.sleep(COMMAND_PERIOD_S)
    finally:
        await controller.set_stop()
        print("Motor stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=SPEED_IN_PER_S, help="inches per second")
    parser.add_argument("--travel", type=float, default=TRAVEL_IN, help="inches (positive = down)")
    args = parser.parse_args()
    asyncio.run(run(args.speed, args.travel))


if __name__ == "__main__":
    main()
