# motor.py
""" Definition of the MoteusThread class, which runs a background thread that communicates with the moteus-x1 motor controller at a fixed rate. The class provides a non-blocking API for setting motor targets and querying motor state.

Unfortunately, the python moteus API is written entirely on top of asyncio, and everything else is synchronous. Part of setup will involve creating a dedicated moteus thread that runs the asyncio event loop, and then we can use asyncio.run_coroutine_threadsafe to call async functions from synchronous code.
"""

import moteus
import asyncio
import dataclasses
import threading
import math
import time
import logging
from typing import List, Optional
from numpy import long
from error_codes import ErrorCode
from control_modes import ControlMode

CONTROLLER_ID: int = 1  # The ID of the moteus-x1 controller on the CAN bus
COMMAND_RATE_HZ: float = 100.0  # well under the ~0.1s watchdog timeout
ZEROING_VELOCITY: float = 0.04  # meters per second, velocity of the plunger during zeroing
RETRACT_VELOCITY: float = 2*ZEROING_VELOCITY  # meters per second, velocity of the plunger during retract
TORQUE_LIMIT: float = 5.0 # Torque command should never exceed 5 Nm for max 500 N force out of rack
PINION_RADIUS: float = 0.01 # meters, radius of the pinion gear on the motor shaft.

# --- data structures ---------------------------------------------------

@dataclasses.dataclass(frozen=True)
class MotorTarget:
    """Package of every value that the Moteus needs for a target command
    NOTE: UNLIKE THE ARDUINO VERSION, values are retained between calls! If a value is not specified, it will be retained from the previous call.
    """
    # Default to velocity control for zeroing
    position: float = math.nan
    velocity: float = 0.0
    kp_scale: float = 1.0
    kd_scale: float = 1.0
    feedforward_torque: float = 0.0
    velocity_limit: float = 0.0
    maximum_torque: float = 5.0 # Torque command should never exceed 5 Nm for max 500 N force out of rack


@dataclasses.dataclass(frozen=True)
class MotorState:
    """Package of query values the Moteus returns, updated every motor loop tick
    """
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
        try:
            assert self._controller is not None
        except AssertionError:
            logging.error("Moteus controller not initialized - Command loop assert error")
            self._last_error = ErrorCode.ERROR_INIT_FAILURE
            return
        
        while not self._stop_requested:
            cycle_start: float = time.monotonic()
            target: MotorTarget = self._target

            # No controlMode decoding here, just pass values through to the controller
            try:
                result: moteus.Result = await self._controller.set_position( # pyright: ignore[reportAttributeAccessIssue]
                    position=target.position,
                    velocity=target.velocity,
                    kp_scale=target.kp_scale,
                    kd_scale=target.kd_scale,
                    feedforward_torque=target.feedforward_torque,
                    velocity_limit=target.velocity_limit,
                    maximum_torque=target.maximum_torque,
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

    # --- public, non-blocking API --------------------------------------

    def set_target(self, control_mode: ControlMode, compressionSetpoint: float = 0) -> ErrorCode:
        # Grab zeroed position from global variable, which is set during zeroing.
        global zeroed_position
        zeroed_position: float
        
        global starting_position
        starting_position: float
        
        match control_mode:
            # Positive rotation of motor is down on rack, so down is positive up is negative
            case ControlMode.ZEROING:
                position: float = math.nan
                velocity: float = ZEROING_VELOCITY # meters per second, downwards
                kp_scale: float = 0.0 # Set kp_scale to 0.0 to disable position control
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = ZEROING_VELOCITY # meters per second, downwards
                maximum_torque: float = TORQUE_LIMIT
            case ControlMode.COMPRESSIONS:
                position: float = compressionSetpoint + zeroed_position # meters, relative to zeroed position
                velocity: float = math.nan # meters per second, downwards
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = math.nan
                maximum_torque: float = TORQUE_LIMIT # Torque command should never exceed 5 Nm for max 500 N force out of rack
            case ControlMode.HOLD_POSITION:
                position: float = math.nan # TODO: Verify that NaN actually forces the controller to hold position
                velocity: float = 0.0 
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = math.nan
                maximum_torque: float = TORQUE_LIMIT # TODO: Change to 1.5x min torque to hold plunger against gravity
            case ControlMode.PAUSE_RETRACT:
                position: float = zeroed_position # meters, retract to zeroed position
                velocity: float = math.nan # meters per second, upwards
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = RETRACT_VELOCITY
                maximum_torque: float = TORQUE_LIMIT
                query: bool = True
            case ControlMode.ABORT_RETRACT:
                position: float = starting_position # meters, retract to absolute zero
                velocity: float = math.nan # meters per second, upwards
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = RETRACT_VELOCITY
                maximum_torque: float = TORQUE_LIMIT
                query: bool = True
            case _:
                logging.error(f"Invalid control mode: {control_mode}")
                return ErrorCode.ERROR_MOTOR_FAILURE
        
        # Set the target values for the motor controller. The MoteusThread will pick up these values in its next loop and send them to the motor controller.
        self._target = MotorTarget(
            position=position, 
            velocity=velocity, 
            kp_scale=kp_scale, 
            kd_scale=kd_scale, 
            feedforward_torque=feedforward_torque, 
            velocity_limit=velocity_limit, 
            maximum_torque=maximum_torque
            )
        
        return ErrorCode.NORMAL_OPERATION

    def get_state(self) -> MotorState:
        return self._state

    def get_last_error(self) -> ErrorCode:
        """Returns the ErrorCode describing the failure. Never raises."""
        return self._last_error