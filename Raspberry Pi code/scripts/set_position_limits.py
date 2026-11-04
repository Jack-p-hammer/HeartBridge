"""
Set the moteus position limits (servopos.position_min / position_max) and save
them to flash, without needing the interactive moteus_tool console.

Limits are in motor rotations. The 1:1 gear train reverses rotation, so down on the
ball screw is negative motor rotation (BLDC_GEAR_RATIO). The defaults allow the full
10 cm ball screw travel (10 rev with the 10 mm lead, same as moteus_thread.py)
starting from zero at the fully retracted (top) position, plus LIMIT_MARGIN_REV.

Usage:
    python set_position_limits.py                 # -10.1 to 0.1 rev
    python set_position_limits.py --min nan --max nan   # disable limits
"""

import argparse
import asyncio

import moteus

CONTROLLER_ID: int = 1  # same as moteus_thread.py
BALL_SCREW_LEAD_M: float = 0.01  # PLACEHOLDER, same as moteus_thread.py, meters of plunger travel per ball screw revolution
BALL_SCREW_TRAVEL_M: float = 0.10  # same as moteus_thread.py, usable ball screw travel down from the top
BLDC_GEAR_RATIO: float = -1.0  # same as moteus_thread.py, motor revolutions per ball screw revolution, 1:1 gear train that reverses rotation
LIMIT_MARGIN_REV: float = 0.1  # slack past each end of travel, in motor rotations

# Full travel in motor rotations from the top, signed by the gear train, so the limits follow BLDC_GEAR_RATIO
TRAVEL_REV: float = BALL_SCREW_TRAVEL_M * BLDC_GEAR_RATIO / BALL_SCREW_LEAD_M
DEFAULT_MIN_REV: float = min(0.0, TRAVEL_REV) - LIMIT_MARGIN_REV
DEFAULT_MAX_REV: float = max(0.0, TRAVEL_REV) + LIMIT_MARGIN_REV


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
    parser.add_argument("--min", default=f"{DEFAULT_MIN_REV:g}", help="rotations, or nan for no limit")
    parser.add_argument("--max", default=f"{DEFAULT_MAX_REV:g}", help="rotations, or nan for no limit")
    args = parser.parse_args()
    asyncio.run(run(args.min, args.max))


if __name__ == "__main__":
    main()
