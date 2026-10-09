"""
Set the moteus position limits (servopos.position_min / position_max) and save
them to flash, without needing the interactive moteus_tool console.

Limits are in motor rotations, positive = down on the rack. The defaults allow
the full 8" stroke (~3.2 rev with the 10 mm pinion) starting from zero at the
fully retracted (top) position.

Usage:
    python set_position_limits.py                 # -0.1 to 3.3 rev
    python set_position_limits.py --min nan --max nan   # disable limits
"""

import argparse
import asyncio

import moteus

CONTROLLER_ID: int = 1  # same as moteus_thread.py


async def run(position_min: str, position_max: str) -> None:
    controller = moteus.Controller(id=CONTROLLER_ID)
    stream = moteus.Stream(controller)
    await controller.set_stop()

    for cmd in (
        f"conf set servopos.position_min {position_min}",
        f"conf set servopos.position_max {position_max}",
        "conf write",
    ):
        reply = await stream.command(cmd.encode())
        print(f"{cmd} -> {reply.decode().strip() or 'OK'}")

    for key in ("servopos.position_min", "servopos.position_max"):
        reply = await stream.command(f"conf get {key}".encode())
        print(f"{key} = {reply.decode().strip()}")

    position = (await controller.query()).values[moteus.Register.POSITION]
    print(f"Current position: {position:.3f} rev")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min", default="-0.1", help="rotations, or nan for no limit")
    parser.add_argument("--max", default="3.3", help="rotations, or nan for no limit")
    args = parser.parse_args()
    asyncio.run(run(args.min, args.max))


if __name__ == "__main__":
    main()
