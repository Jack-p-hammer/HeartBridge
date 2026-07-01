# actuation.py
"""Unfortunately, the python moteus API is written entirely on top of asyncio, and everything else is synchronous. Part of setup will involve creating a dedicated moteus thread that runs the asyncio event loop, and then we can use asyncio.run_coroutine_threadsafe to call async functions from synchronous code.

Returns:
    _type_: _description_
"""

import moteus
import logging
import asyncio
import dataclasses
import threading
import time
import math
from typing import List, Optional
from error_codes import ErrorCode

CONTROLLER_ID: int = 1  # The ID of the moteus-x1 controller on the CAN bus
COMMAND_RATE_HZ: float = 100.0  # well under the ~0.1s watchdog timeout


@dataclasses.dataclass(frozen=True)
class MotorTarget:
    position: float
    velocity: float = 0.0


@dataclasses.dataclass(frozen=True)
class MotorState:
    mode: int
    fault: int
    position: float
    velocity: float
    voltage: float
    stale: bool          # True if we've never gotten a response yet
    timestamp: float      # time.monotonic() of last successful update


class MoteusThread:
    """Background thread + event loop that talks to a single moteus
    controller at a fixed rate. All public methods are non-blocking and
    never raise -- failures surface as ErrorCode via get_last_error()."""

    def __init__(
        self,
        controller_id: int = CONTROLLER_ID,
        rate_hz: float = COMMAND_RATE_HZ,
    ) -> None:
        self._controller_id: int = controller_id
        self._period_s: float = 1.0 / rate_hz

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._controller: Optional[moteus.Controller] = None
        self._thread: threading.Thread = threading.Thread(
            target=self._run_loop, daemon=True
        )
        self._ready: threading.Event = threading.Event()
        self._stop_requested: bool = False

        self._target: MotorTarget = MotorTarget(position=math.nan, velocity=0.0)
        self._state: MotorState = MotorState(
            mode=0, fault=0, position=0.0, velocity=0.0, voltage=0.0,
            stale=True, timestamp=0.0,
        )

        # Communication/setup-level error, as reported to the state
        # machine. This is distinct from MotorState.fault, which is the
        # *controller's own* fault register (uncalibrated, out of
        # bounds, etc) -- that's read via get_state(), not this.
        self._last_error: ErrorCode = ErrorCode.ERROR_INIT_FAILURE
        self._init_error: ErrorCode = ErrorCode.ERROR_INIT_FAILURE

    # --- lifecycle ---------------------------------------------------

    def start(self, timeout: float = 5.0) -> ErrorCode:
        """Start the background thread and wait for setup to finish
        (thread/object construction only -- no CAN I/O happens here).
        Returns None on success, or an ErrorCode if setup failed."""
        self._thread.start()
        if not self._ready.wait(timeout=timeout):
            return ErrorCode.ERROR_INIT_FAILURE
        return self._init_error

    def stop(self) -> None:
        self._stop_requested = True

    def _run_loop(self) -> None:
        try:
            loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop
            self._controller = moteus.Controller(id=self._controller_id)
        except Exception:
            self._init_error = ErrorCode.ERROR_INIT_FAILURE
            self._ready.set()
            return

        self._init_error = ErrorCode.NORMAL_OPERATION  # explicit confirmation of success
        self._ready.set()
        try:
            loop.run_until_complete(self._command_loop())
        finally:
            loop.close()

    async def _command_loop(self) -> None:
        assert self._controller is not None
        while not self._stop_requested:
            cycle_start: float = time.monotonic()
            target: MotorTarget = self._target

            try:
                result: moteus.Result = await self._controller.set_position( # pyright: ignore[reportAttributeAccessIssue]
                    position=target.position,
                    velocity=target.velocity,
                    query=True,
                )
                self._state = MotorState(
                    mode=result.values[moteus.Register.MODE],
                    fault=result.values[moteus.Register.FAULT],
                    position=result.values[moteus.Register.POSITION],
                    velocity=result.values[moteus.Register.VELOCITY],
                    voltage=result.values[moteus.Register.VOLTAGE],
                    stale=False,
                    timestamp=time.monotonic(),
                )
                self._last_error = ErrorCode.NORMAL_OPERATION
            except Exception as e:
                # Communication-level failure (timeout, transport, # error, etc).
                logging.error(f"Moteus command loop failed: {e}")
                self._last_error = ErrorCode.ERROR_MOTOR_FAILURE

            elapsed: float = time.monotonic() - cycle_start
            await asyncio.sleep(max(0.0, self._period_s - elapsed))

        try:
            await self._controller.set_stop()
        except Exception:
            pass

    # --- public, non-blocking API --------------------------------------

    def set_target(self, position: float, velocity: float = 0.0) -> None:
        self._target = MotorTarget(position=position, velocity=velocity)

    def get_state(self) -> MotorState:
        return self._state

    def get_last_error(self) -> ErrorCode:
        """Returns the ErrorCode describing the failure. Never raises."""
        return self._last_error


_motor_controller: MoteusThread  # The shared moteus controller instance


def init_motor() -> ErrorCode:
    """Initialize moteus-x1 motor driver thread and controller settings.
    Motor controller communication is done via CAN-FD board over USB (from raspi side)

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    global _motor_controller 
    logging.debug("Initializing motor driver")
    
    _motor_controller = MoteusThread(controller_id=CONTROLLER_ID)
    motor_error: ErrorCode = _motor_controller.start()
    
    if motor_error != ErrorCode.NORMAL_OPERATION:
        logging.error(f"Motor controller initialization failed: {motor_error}")
        return ErrorCode.ERROR_INIT_FAILURE

    return ErrorCode.NORMAL_OPERATION


def zeroing() -> ErrorCode:
    """Perform the zeroing procedure. Non-blocking.

    Returns:
        ErrorCode: Normal while zeroing, ZEROING_FINISHED when finished, ERROR_ZEROING_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION


def init_compressions() -> ErrorCode:
    """Perform pre-compressions setup

    Returns:
        ErrorCode: Normal operation if successful, ERROR_INIT_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION


def compressions() -> ErrorCode:
    """Perform compressions. Non-blocking

    Returns:
        ErrorCode: Normal operation while compressing, ERROR_SENSOR_FAILURE if sensor failure detected
    """
    return ErrorCode.NORMAL_OPERATION


def stop_compressions() -> ErrorCode:
    """Return plunger to zeroed position for compressions pause

    Returns:
        ErrorCode: Normal operation if successful, ERROR_ZEROING_FAILURE if failed
    """
    return ErrorCode.NORMAL_OPERATION
