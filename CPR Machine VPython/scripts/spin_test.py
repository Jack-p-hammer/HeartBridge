"""
Spin the motor TURNS rotations one way, then TURNS rotations back, then stop.

Works directly in motor rotations (no inch conversion). Press Enter to start;
press Enter again at any time to stop early.

NOTE: the controller's servopos.position_min/max limits still apply -- the
motor will stop at the limit if TURNS goes past it. To spin freely, disable
them first:  python set_position_limits.py --min nan --max nan

Usage:
    python spin_test.py                       # 10 turns each way
    python spin_test.py --turns 5 --speed 1   # 5 turns at 1 rotation/s
"""

import argparse
import asyncio
import time

import moteus

CONTROLLER_ID: int = 1  # same as moteus_thread.py
MAX_TORQUE_NM: float = 2.0  # kept low for bench testing (main code allows 5.0)
COMMAND_PERIOD_S: float = 0.01  # 100 Hz, well under the moteus watchdog timeout

TURNS: float = 10.0  # positive spins the "down" direction first
SPEED_REV_S: float = 0.5  # rotations per second -> 20 s each way


async def run(turns: float, speed_rev_s: float) -> None:
    controller = moteus.Controller(id=CONTROLLER_ID)
    await controller.set_stop()  # clear any latched fault before commanding

    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, input, "Press Enter to start spinning...")
    stop_pressed = loop.run_in_executor(None, input, "Spinning. Press Enter to stop early.\n")

    start: float = (await controller.query()).values[moteus.Register.POSITION]
    leg_s: float = abs(turns) / speed_rev_s  # time for one direction
    direction: float = 1.0 if turns >= 0 else -1.0
    t0: float = time.monotonic()
    last_print: float = 0.0
    print(f"Start position {start:.3f} rev, {turns} turns each way at {speed_rev_s} rev/s")

    try:
        while not stop_pressed.done():
            t: float = time.monotonic() - t0
            if t >= 2.0 * leg_s:
                print("Done: back at start.")
                break
            # Out for the first leg, back for the second, constant speed
            if t < leg_s:
                target, velocity = start + direction * speed_rev_s * t, direction * speed_rev_s
            else:
                target, velocity = start + turns - direction * speed_rev_s * (t - leg_s), -direction * speed_rev_s

            result = await controller.set_position(
                position=target, velocity=velocity, maximum_torque=MAX_TORQUE_NM, query=True,
            )

            fault = result.values[moteus.Register.FAULT]
            if time.monotonic() - last_print > 1.0:
                last_print = time.monotonic()
                print(f"target {target:.3f}  pos {result.values[moteus.Register.POSITION]:.3f}  "
                      f"mode {result.values[moteus.Register.MODE]}  fault {fault}")
            if fault:
                print(f"Controller fault {fault} -- stopping.")
                break
            await asyncio.sleep(COMMAND_PERIOD_S)
    finally:
        await controller.set_stop()
        print("Motor stopped. (Press Enter to exit if the prompt is still waiting.)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--turns", type=float, default=TURNS, help="rotations each way")
    parser.add_argument("--speed", type=float, default=SPEED_REV_S, help="rotations per second")
    args = parser.parse_args()
    asyncio.run(run(args.turns, abs(args.speed)))


if __name__ == "__main__":
    main()
