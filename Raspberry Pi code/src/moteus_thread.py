# motor.py
""" Definition of the MoteusThread class, which runs a background thread that communicates with the moteus-x1 motor controller at a fixed rate. The class provides a non-blocking API for setting motor targets and querying motor state.

Unfortunately, the python moteus API is written entirely on top of asyncio, and everything else is synchronous. Part of setup will involve creating a dedicated moteus thread that runs the asyncio event loop, and then we can use asyncio.run_coroutine_threadsafe to call async functions from synchronous code.
"""
# External imports
import moteus
import asyncio
import dataclasses
import threading
import math
import time
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

# Internal imports
from Enums.error_codes import ErrorCode
from Enums.control_modes import ControlMode

CONTROLLER_ID: int = 1  # The ID of the moteus-x1 controller on the CAN bus
COMMAND_RATE_HZ: float = 100.0  # well under the ~0.1s watchdog timeout
REPLY_TIMEOUT_S: float = 0.05  # give up on a reply well before the ~0.1s watchdog, same as scripts/slow_jog.py
STOP_ATTEMPTS: int = 5  # tries at the startup stop command before commanding anyway (the command loop then logs errors)
# About 0.2% of commands never fully reach the CAN adapter (seen in the raw traffic log), costing one or two cycles each:
# the lost command times out with no reply, then the adapter rejects the next command, which it reads joined onto the
# lost one, with "ERR unknown flag". For those two failures only, get_last_error() waits until there's been no good
# reply for this long before reporting a motor failure, so one lost frame doesn't abort. Any other failure is reported
# straight away. Two back-to-back lost frames take about 0.13 s; the moteus's own ~0.1 s watchdog still stops it sooner
LOST_REPLY_TOLERANCE_S: float = 0.2
LOST_FRAME_ADAPTER_ERROR: str = "unknown flag"  # in the adapter's reply to a command read joined onto a lost one
# Mode the moteus latches when its watchdog sees no command for ~0.1 s. It ignores position commands until a stop
MODE_POSITION_TIMEOUT: int = 11
RETRACT_VELOCITY_MPS: float = 0.08  # meters per second, velocity of the plunger along the ball screw during retract

# Diagnostics: write every raw line sent to and received from the CAN adapter to Logs/moteus_raw_<timestamp>.log.
# Each line is stamped with time.time(); the command loop's error messages include the same clock to line them up.
# TODO: Turn off once the lost frames are diagnosed, it adds a file write per line on the moteus thread
RAW_TRAFFIC_LOG: bool = True
RAW_TRAFFIC_LOG_DIR: Path = Path(__file__).resolve().parent.parent / "Logs"  # same folder as the main logs

# --- Ball screw constants (PLACEHOLDERS, TODO: replace with real hardware values) ---
BALL_SCREW_LEAD_M: float = 0.0254*0.5  # meters of plunger travel per ball screw revolution
# Signed: the 1:1 gear train reverses rotation, so negative (matches scripts/slow_jog.py). This sign is the only place
# the BLDC's direction is set; flip it to reverse the motor. Plunger travel is always positive down
BLDC_GEAR_RATIO: float = -1.0  # motor revolutions per ball screw revolution
BLDC_ROTATIONS_PER_M: float = BLDC_GEAR_RATIO / BALL_SCREW_LEAD_M  # motor rotations per meter of plunger travel, signed like BLDC_GEAR_RATIO
# Ball screw travel only covers compressions plus safety room at either end. Carriage travel is handled by the lead screw steppers (stepper.py)
BALL_SCREW_TOP_CLEARANCE_M: float = 0.01  # meters, the plunger sits this far below the top of travel when zeroing, so pause/abort can lift off the chest
BALL_SCREW_TRAVEL_M: float = 0.12  # meters, total usable ball screw travel down from the startup (absolute zero) position

MAX_PLUNGER_FORCE_N: float = 500.0
# Ideal (100% efficient) ball screw: F = 2*pi*T / lead. Real efficiency only lowers the force, so this limit is conservative
TORQUE_LIMIT_NM: float = MAX_PLUNGER_FORCE_N * BALL_SCREW_LEAD_M / (2 * math.pi * abs(BLDC_GEAR_RATIO))

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
    maximum_torque: float = TORQUE_LIMIT_NM # Torque command should never exceed MAX_PLUNGER_FORCE_N out of the ball screw
    # Acceleration cap on the way to position, in rotations per second squared. NaN leaves it out of the command entirely,
    # so the controller uses its configured default (servo.default_accel_limit), exactly as before this field existed
    accel_limit: float = math.nan


@dataclasses.dataclass(frozen=True)
class MotorState:
    """Package of query values the Moteus returns, updated every motor loop tick
    """
    mode: int
    fault: int
    position: float
    velocity: float
    voltage: float
    q_current: float     # amps, torque-producing (q-axis) phase current. Not battery draw
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
        # Set by request_stop_sequence(), cleared by the command loop when it runs the stop sequence
        self._stop_sequence_requested: bool = False

        self._target: MotorTarget = MotorTarget(position=math.nan, velocity=0.0)
        self._state: MotorState = MotorState(
            mode=0, fault=0, position=0.0, velocity=0.0, voltage=0.0,
            q_current=0.0, stale=True, timestamp=0.0,
        )

        # Communication/setup-level error, as reported to the state
        # machine. This is distinct from MotorState.fault, which is the
        # *controller's own* fault register (uncalibrated, out of
        # bounds, etc) -- that's read via get_state(), not this.
        self._last_error: ErrorCode = ErrorCode.ERROR_INIT_FAILURE
        # True when _last_error came from a lost frame (no reply in time, or the adapter rejecting the next command
        # joined onto the lost one), the only failures get_last_error() tolerates for LOST_REPLY_TOLERANCE_S
        self._last_error_from_lost_frame: bool = False
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
            # Q current is ignored by the default query, so request it explicitly
            query_resolution = moteus.QueryResolution()
            query_resolution.q_current = moteus.F32
            controller_kwargs: dict[str, Any] = {}
            if RAW_TRAFFIC_LOG:
                # The transport only takes a debug log when it's first built, so build it here instead of letting the
                # controller build it on the first command. Built the same way, from moteus's own transport options
                try:
                    RAW_TRAFFIC_LOG_DIR.mkdir(parents=True, exist_ok=True)
                    log_path: Path = RAW_TRAFFIC_LOG_DIR / f"moteus_raw_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                    parser = argparse.ArgumentParser()
                    moteus.make_transport_args(parser) # pyright: ignore[reportAttributeAccessIssue]
                    transport_args = parser.parse_args(["--can-debug", str(log_path)])

                    # The fdcanusb transport needs a running event loop to be built (it starts its reader task then),
                    # so build it inside this thread's loop. The reader task waits in the loop until the command loop runs
                    async def build_transport() -> Any:
                        return moteus.get_singleton_transport(transport_args) # pyright: ignore[reportAttributeAccessIssue]
                    controller_kwargs["transport"] = loop.run_until_complete(build_transport())
                    logging.info(f"Logging raw moteus traffic to {log_path}")
                except Exception as e:
                    # Diagnostics only: carry on exactly as without the log
                    logging.warning(f"Raw moteus traffic log not started, continuing without it: {e}")
            self._controller = moteus.Controller(id=self._controller_id, query_resolution=query_resolution, **controller_kwargs)
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
            self._last_error_from_lost_frame = False
            return

        # When a previous run ends, commands stop and the moteus watchdog latches it in position timeout mode (11),
        # which ignores position commands until a stop is received. Clear it first, like scripts/slow_jog.py
        await self._stop_sequence(self._controller, "startup")

        while not self._stop_requested:
            cycle_start: float = time.monotonic()
            cycle_wall_time: float = time.time()  # same clock as the raw traffic log, to find this cycle's lines in it

            # A reset (request_stop_sequence()) runs the same stop as startup. Cleared first, so a request made
            # while it runs gets its own stop next cycle
            if self._stop_sequence_requested:
                self._stop_sequence_requested = False
                await self._stop_sequence(self._controller, "reset")

            target: MotorTarget = self._target

            # No controlMode decoding here, just pass values through to the controller
            try:
                # Over the fdcanusb, the moteus library waits forever for a lost reply (the first one after connecting is
                # often lost), which would freeze this loop for good. The timeout turns it into a normal failure below
                result: moteus.Result = await asyncio.wait_for(self._controller.set_position( # pyright: ignore[reportAttributeAccessIssue]
                    position=target.position,
                    velocity=target.velocity,
                    kp_scale=target.kp_scale,
                    kd_scale=target.kd_scale,
                    feedforward_torque=target.feedforward_torque,
                    velocity_limit=target.velocity_limit,
                    maximum_torque=target.maximum_torque,
                    # None omits the register, so commands without an accel limit are unchanged on the bus
                    accel_limit=None if math.isnan(target.accel_limit) else target.accel_limit,
                    query=True,
                ), timeout=REPLY_TIMEOUT_S)
                self._state = MotorState(
                    mode=result.values[moteus.Register.MODE],
                    fault=result.values[moteus.Register.FAULT],
                    position=result.values[moteus.Register.POSITION],
                    velocity=result.values[moteus.Register.VELOCITY],
                    voltage=result.values[moteus.Register.VOLTAGE],
                    q_current=result.values[moteus.Register.Q_CURRENT],
                    stale=False,
                    timestamp=time.monotonic(),
                )
                self._last_error = ErrorCode.NORMAL_OPERATION
                self._last_error_from_lost_frame = False

                # If this thread was held up long enough for the moteus watchdog to trip (e.g. while pygame starts up),
                # the moteus latched position timeout mode and is ignoring these commands. Clear it with the same stop
                # as startup; the next cycle's command takes over again. Only sent when the moteus is already
                # ignoring commands, so it never interrupts motion that's being followed
                if self._state.mode == MODE_POSITION_TIMEOUT:
                    logging.warning(f"Moteus in position timeout mode ({MODE_POSITION_TIMEOUT}), ignoring commands: "
                                    f"sending a stop to clear it (reply at {time.time():.6f}, raw log clock)")
                    await self._stop_sequence(self._controller, "position timeout")
            except asyncio.TimeoutError:
                # Lost reply. The next cycle sends the target again
                logging.error(f"Moteus command loop failed: no reply within {REPLY_TIMEOUT_S} s "
                              f"(sent at {cycle_wall_time:.6f}, raw log clock)")
                self._last_error = ErrorCode.ERROR_MOTOR_FAILURE
                self._last_error_from_lost_frame = True
            except Exception as e:
                # Communication-level failure (transport, adapter error, etc). Only the adapter rejecting a command read
                # joined onto a lost one counts as a lost frame; anything else is reported straight away
                logging.error(f"Moteus command loop failed: {e} (sent at {cycle_wall_time:.6f}, raw log clock)")
                self._last_error = ErrorCode.ERROR_MOTOR_FAILURE
                self._last_error_from_lost_frame = LOST_FRAME_ADAPTER_ERROR in str(e)

            elapsed: float = time.monotonic() - cycle_start
            await asyncio.sleep(max(0.0, self._period_s - elapsed))

    async def _stop_sequence(self, controller: moteus.Controller, reason: str) -> None:
        """Send a stop, retrying up to STOP_ATTEMPTS times. Clears any latched fault or position timeout mode (11).
        Queried so a lost frame (common right after connecting) is retried instead of silently skipped.
        If every attempt fails it carries on anyway, and the command loop then logs errors.

        Args:
            controller (moteus.Controller): The controller to stop
            reason (str): What the stop is for ("startup", "reset" or "position timeout"), for logging
        """
        for attempt in range(1, STOP_ATTEMPTS + 1):
            try:
                await asyncio.wait_for(controller.set_stop(query=True), timeout=REPLY_TIMEOUT_S) # pyright: ignore[reportAttributeAccessIssue]
                logging.info(f"Moteus {reason} stop sent")
                return
            except Exception as e:
                if attempt == STOP_ATTEMPTS:
                    logging.error(f"Moteus {reason} stop attempt {attempt}/{STOP_ATTEMPTS} failed: {str(e) or 'no reply'}")
                else:
                    logging.warning(f"Moteus {reason} stop attempt {attempt}/{STOP_ATTEMPTS} failed: {str(e) or 'no reply'}")

    # --- public, non-blocking API --------------------------------------

    def request_stop_sequence(self) -> None:
        """Put the motor back the way it is after a power cycle: the target goes back to the startup default (hold
        position), and the command loop runs the same stop sequence as at startup at the start of its next cycle.
        The target is reset here, before the request, so a set_target() made after this call is never overwritten.
        """
        self._target = MotorTarget(position=math.nan, velocity=0.0)
        self._stop_sequence_requested = True

    def set_target(self, control_mode: ControlMode, zero_position: float, abs_zero_position: float,
                   compressionSetpoint: float = 0,
                   compressionVelocity: float = 0.0,
                   compressionVelocityLimit: float = math.nan,
                   compressionAccelLimit: float = math.nan) -> ErrorCode:
        """Set the target values for the motor controller. The MoteusThread will pick up these values in its next loop and send them to the motor controller.
        Setpoints come in as meters of ball screw travel and are converted to motor rotations here.

            Args:
                control_mode (ControlMode): What the plunger should be doing
                zero_position (float): Zeroed (chest contact) motor position, in rotations
                abs_zero_position (float): Startup (top of ball screw travel) motor position, in rotations
                compressionSetpoint (float): Compression depth below zero_position, in meters. COMPRESSIONS: the extreme
                    of the current stroke. COMPRESSIONS_STREAMED_DEPRECATED: the current position along the stroke
                compressionVelocity (float): Plunger velocity at compressionSetpoint, in meters per second, positive down.
                    Only used for COMPRESSIONS_STREAMED_DEPRECATED
                compressionVelocityLimit (float): Speed cap on the way to the stroke's extreme, in meters per second.
                    Required (positive) for COMPRESSIONS
                compressionAccelLimit (float): Acceleration cap on the way to the stroke's extreme, in meters per second
                    squared. Required (positive) for COMPRESSIONS

            Returns:
                ErrorCode: ERROR_MOTOR_FAILURE if the control mode is invalid, a COMPRESSIONS limit is missing, or the target is outside the ball screw travel, NORMAL_OPERATION otherwise
        """
        retract_velocity_rps: float = RETRACT_VELOCITY_MPS * abs(BLDC_ROTATIONS_PER_M)  # velocity_limit is a magnitude
        accel_limit: float = math.nan  # NaN leaves accel_limit out of the command, only COMPRESSIONS sets it
        match control_mode:
            # Setpoints are meters of plunger travel, positive down. Multiplying by BLDC_ROTATIONS_PER_M converts them to
            # motor rotations, including the gear train's reversal
            case ControlMode.ZEROING:
                # The steppers lower the whole carriage during zeroing, the plunger just holds just below the top of its travel
                position: float = abs_zero_position + BALL_SCREW_TOP_CLEARANCE_M * BLDC_ROTATIONS_PER_M # rotations
                velocity: float = 0.0
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = retract_velocity_rps # rotations per second
                maximum_torque: float = TORQUE_LIMIT_NM
            case ControlMode.COMPRESSIONS:
                # Same as demo/jog_compressions.py: the position is the extreme of the current stroke, with velocity 0
                # there. The moteus moves toward it with speed capped at velocity_limit and acceleration capped at
                # accel_limit, so it speeds up, cruises, and slows down to stop at the extreme. That happens inside the
                # moteus, so the motion is smooth between main loop ticks and a lost frame doesn't interrupt the stroke.
                # Without both caps the moteus would jump to each extreme as fast as it can, so they're required.
                # Written so NaN fails
                if not (compressionVelocityLimit > 0.0 and compressionAccelLimit > 0.0):
                    logging.error(f"Compressions need positive velocity and accel limits, got "
                                  f"{compressionVelocityLimit} m/s and {compressionAccelLimit} m/s^2")
                    return ErrorCode.ERROR_MOTOR_FAILURE
                position: float = zero_position + compressionSetpoint * BLDC_ROTATIONS_PER_M # rotations, relative to zeroed position
                velocity: float = 0.0 # stop at the end of the stroke
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = compressionVelocityLimit * abs(BLDC_ROTATIONS_PER_M) # rotations per second, a magnitude
                accel_limit = compressionAccelLimit * abs(BLDC_ROTATIONS_PER_M) # rotations per second squared, a magnitude
                maximum_torque: float = TORQUE_LIMIT_NM # Torque command should never exceed MAX_PLUNGER_FORCE_N out of the ball screw
            case ControlMode.COMPRESSIONS_STREAMED_DEPRECATED:
                # Deprecated: every position along the stroke is streamed from the main loop, with the matching velocity so
                # the moteus keeps moving between commands instead of stepping. No speed or acceleration caps
                position: float = zero_position + compressionSetpoint * BLDC_ROTATIONS_PER_M # rotations, relative to zeroed position
                velocity: float = compressionVelocity * BLDC_ROTATIONS_PER_M # rotations per second, signed like position
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = math.nan
                maximum_torque: float = TORQUE_LIMIT_NM # Torque command should never exceed MAX_PLUNGER_FORCE_N out of the ball screw
            case ControlMode.HOLD_POSITION:
                position: float = math.nan # TODO: Verify that NaN actually forces the controller to hold position
                velocity: float = 0.0 
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = math.nan
                maximum_torque: float = TORQUE_LIMIT_NM # TODO: Change to 1.5x min torque to hold plunger against gravity
            case ControlMode.PAUSE_RETRACT:
                position: float = zero_position # rotations, retract to zeroed position
                velocity: float = math.nan
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = retract_velocity_rps # rotations per second
                maximum_torque: float = TORQUE_LIMIT_NM
            case ControlMode.ABORT_RETRACT:
                position: float = abs_zero_position # rotations, retract to top of ball screw travel
                velocity: float = math.nan
                kp_scale: float = 1.0
                kd_scale: float = 1.0
                feedforward_torque: float = 0.0
                velocity_limit: float = retract_velocity_rps # rotations per second
                maximum_torque: float = TORQUE_LIMIT_NM
            case _:
                logging.error(f"Invalid control mode: {control_mode}")
                return ErrorCode.ERROR_MOTOR_FAILURE

        # Never command the plunger past either end of the ball screw
        if not math.isnan(position):
            travel_m: float = (position - abs_zero_position) / BLDC_ROTATIONS_PER_M
            if not (0.0 <= travel_m <= BALL_SCREW_TRAVEL_M):
                logging.error(f"Ball screw target out of travel: {travel_m} m, limit {BALL_SCREW_TRAVEL_M} m")
                return ErrorCode.ERROR_MOTOR_FAILURE
        
        # Set the target values for the motor controller. The MoteusThread will pick up these values in its next loop and send them to the motor controller.
        self._target = MotorTarget(
            position=position, 
            velocity=velocity, 
            kp_scale=kp_scale, 
            kd_scale=kd_scale, 
            feedforward_torque=feedforward_torque, 
            velocity_limit=velocity_limit,
            maximum_torque=maximum_torque,
            accel_limit=accel_limit
        )
        
        return ErrorCode.NORMAL_OPERATION

    def set_raw_target(self, target: MotorTarget) -> None:
        """Send a target in raw motor units (rotations, rotations per second), bypassing set_target()'s ControlMode
        conversion and ball screw travel check. The caller must keep the target inside the ball screw travel.
        Used by demo/jog_compressions.py for its fixed-stroke bench test

        Args:
            target (MotorTarget): Every value the moteus is sent on its next command loop tick
        """
        self._target = target

    def get_state(self) -> MotorState:
        """Get motor state

        Returns:
            MotorState: State object containing the latest motor state, as reported by the moteus controller. If the state is stale (i.e. the controller has not responded yet), the stale field will be True.
        """
        return self._state

    def get_last_error(self) -> ErrorCode:
        """Returns the ErrorCode describing the failure/lack thereof from the motor controller thread.
        A lost frame (no reply in time, or the adapter's "ERR unknown flag" for the next command) is only reported once
        there's been no good reply for LOST_REPLY_TOLERANCE_S, so an occasional lost frame isn't treated as a motor
        failure. Each failed cycle is still logged. Every other error is reported straight away, as is any error before
        the moteus has replied at least once.

        Possible values:
            NORMAL_OPERATION: the latest cycle got a good reply, or a lost frame within the tolerance
            ERROR_INIT_FAILURE: the command loop hasn't run yet, or started without a controller
            ERROR_MOTOR_FAILURE: a command failed (lost frames only after the tolerance)
        """
        if (self._last_error == ErrorCode.NORMAL_OPERATION or
            not self._last_error_from_lost_frame or
            self._state.stale):
            return self._last_error
        if time.monotonic() - self._state.timestamp <= LOST_REPLY_TOLERANCE_S:
            return ErrorCode.NORMAL_OPERATION
        return self._last_error
    
    def get_battery_voltage(self) -> float:
        """Returns the battery voltage of the 6S Lipo battery pack, as reported by the moteus controller."""
        return self._state.voltage
    
    def get_rotary_position(self) -> float:
        """Returns the rotary position of the motor, in rotations, as reported by the moteus controller."""
        return self._state.position

    def get_motor_current(self) -> float:
        """Returns the q-axis (torque-producing) phase current of the motor, in amps, as reported by the moteus controller."""
        return self._state.q_current