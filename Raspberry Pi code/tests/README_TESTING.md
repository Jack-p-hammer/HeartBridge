# CPR Machine Code Tests
This folder contains simple tests for the parts of the project that can be checked without the real hardware attached.

## Why these tests exist
The machine depends on sensors, buttons, LEDs, and a motor controller. Those pieces are hard to test on a laptop, so these tests use small fake versions of the hardware interfaces instead.

## How to run the tests
From the `Raspberry Pi code/` folder, with its virtual environment active, run:

```bash
python -m pytest -q
```

If you only want to run the tests in this folder, that same command will do it.

## Notes for future contributors
- These tests use fake hardware.
- If new hardware logic is added, add or update tests in this folder.


## Current test inventory
Format: `case -> result`.

### HMI tests
- init_Pi(): pigpio daemon connected -> NORMAL_OPERATION, get_pi() returns
  the shared instance. daemon unavailable -> ERROR_INIT_FAILURE.
- init_HMI(): healthy GPIO -> pins configured (Next/Pause input+pullup,
  LED/laser output+pulldown), laser PWM off. pygame/mixer failure ->
  ERROR_PYGAME_INIT_FAILURE. display failure -> ERROR_INIT_FAILURE. button/
  LED GPIO failure -> ERROR_INIT_FAILURE. laser PWM init failure ->
  ERROR_PI_DAEMON_FAILURE.
- enable/disable_next/pause_button(): writes HIGH/LOW to LED pin. Repeated
  call -> idempotent. pigpio write exception -> ERROR_PI_DAEMON_FAILURE.
- next/pause_button_pressed(): active low, pin LOW/HIGH -> True/False. press
  within BUTTON_TIMEOUT_SEC of the last accepted press (either button) ->
  False, accepted again once the timeout elapses. Both pins LOW -> first read
  True, second locked out. pigpio read exception -> ERROR_PI_DAEMON_FAILURE,
  False.
- enable/disable_lasers(): writes nonzero/zero duty cycle to laser pin.
  LASER_PWM_DUTY_CYCLE in {0, 0.25, 0.5, 1.0} -> raw value DUTY_CYCLE *
  SCALE, NORMAL_OPERATION. LASER_PWM_DUTY_CYCLE in {-0.1, 1.5} (real pigpio
  rejects via a negative return code) -> ERROR_PI_DAEMON_FAILURE. pigpio
  exception (e.g. daemon/socket failure) -> ERROR_PI_DAEMON_FAILURE /
  ERROR_INIT_FAILURE, never raises. repeated enable or repeated disable ->
  idempotent, same PWM call each time.
- pump_events(): healthy pygame -> NORMAL_OPERATION. broken event queue ->
  WARNING_PYGAME_PUMP_FAILURE.
- set_image()/set_audio()/set_image_audio(): valid prompt -> plays looped.
  pygame load failure (real missing file) or Channel.play() failure ->
  ERROR_UNKNOWN_IMAGE / ERROR_PYGAME_FAILURE. image failure -> audio not
  attempted. image succeeds, audio fails -> audio's error code propagated,
  not silently NORMAL_OPERATION. no audio configured (empty-path stand-in
  prompt, since every real prompt has a file) -> playback stopped, NORMAL_OPERATION, not a distinct
  ERROR_UNKNOWN_AUDIO code. same prompt set twice -> playback restarted both
  times, but Sound loaded from disk only once (cached, not reloaded on
  repeat).
- audio_finished(): elapsed < prompt length -> False. elapsed >= length ->
  True. audio-less state -> True. no prompt ever played -> True.

### Sensing tests
- init_sensors(): I2C/ToF (VL53L0X)/IMU unavailable -> ERROR_INIT_FAILURE.
  rotary/ToF/force absolute-zero read raises -> ERROR_INIT_FAILURE.
  healthy hardware -> NORMAL_OPERATION, absolute-zero positions captured.
- read_force_sensor()/read_ToF_sensor()/read_IMU(): raw ADC bytes/ToF
  range/IMU accel -> matching converted value.
- read_sensors(): within limits -> NORMAL_OPERATION. force past zeroing
  threshold -> ZEROING_FINISHED. IMU beyond tolerance, rotary/ToF
  disagreement, or invalid mode -> ERROR_SENSOR_FAILURE. I2C/rotary/ToF/IMU
  raises mid-operation -> ERROR_SENSOR_FAILURE.
- zero_position(): zeroing finished -> position captured, NORMAL_OPERATION.
  zeroing in progress -> ERROR_SENSOR_FAILURE.
- zero_rotary_encoder()/zero_ToF_sensor(): read raises ->
  ERROR_SENSOR_FAILURE, never raises.
- battery_check(): healthy/low/unreadable voltage -> NORMAL_OPERATION /
  ERROR_LOW_BATTERY / ERROR_MOTOR_FAILURE.
- check_lead_screw_position(): encoders within tolerance of the command, or
  anywhere between the position and setpoint -> NORMAL_OPERATION. encoders
  lagging the position (missed steps) or screws too far apart from each other
  (racking) -> ERROR_MOTOR_FAILURE. no encoder readings -> ERROR_SENSOR_FAILURE.

### Actuation tests
- init_motor()/get_motor_controller(): healthy moteus construction ->
  NORMAL_OPERATION, same instance on later calls. failing construction ->
  ERROR_INIT_FAILURE. command loop with no controller -> ERROR_INIT_FAILURE,
  returns without raising.
- init_steppers(): any one pigpio setup call (pin mode, STEP/DIR/EN write,
  waveform clear) raises -> ERROR_INIT_FAILURE.
- init_zeroing()/init_compressions(): records start time.
- zeroing(): timeout / motor error / max extension -> matching error.
  pigpio failure while lowering the carriage -> ERROR_MOTOR_FAILURE, steppers
  halted. healthy sensors -> NORMAL_OPERATION.
- stepper.update(): pigpio rejects a waveform, or any one pigpio call (read
  playing wave, delete wave, DIR write, add pulses, create wave, send wave)
  raises -> ERROR_MOTOR_FAILURE.
- computeCompressionSetpoint(): time in 0.56s cycle -> flat, ramp, plateau,
  ramp, repeat. exact segment-boundary instants -> correct value, not off by
  one segment.
- compressions()/pause_compressions(): healthy / sensor failure / motor
  failure -> NORMAL_OPERATION / ERROR_SENSOR_FAILURE / ERROR_MOTOR_FAILURE.
  IMU kneel failure -> passed through unchanged, not ERROR_SENSOR_FAILURE.
  pigpio failure in the stepper update -> ERROR_MOTOR_FAILURE.
- abort_compressions(): no sensing init needed -> NORMAL_OPERATION.
- stepper.set_target(): out of travel, NaN, non-positive velocity, or wrong
  type -> ERROR_MOTOR_FAILURE, previous setpoint kept, never raises.
- retract_carriage(): pigpio failure -> ERROR_MOTOR_FAILURE, never raises.

### Main-state tests
- ERROR_STATE_MAP: each recoverable error -> ABORT, except IMU kneel ->
  KNEEL_FAILURE. FATAL_ERRORS = {EXIT_UNKNOWN, ERROR_PYGAME_INIT_FAILURE,
  ERROR_PI_DAEMON_FAILURE, ERROR_UNKNOWN_IMAGE}, disjoint from
  ERROR_STATE_MAP.
- Fatal fast exit: HMI init failure / unknown image / pigpio daemon failure
  in STARTUP -> motor aborted (each of the three verifies abort_compressions()
  was called), SystemExit(1).
- State chain, Next held: STARTUP -> UNFOLD_CUT_CLOTHES -> ALIGNMENT ->
  ZEROING_PREP -> ZEROING. zeroing finished -> COMPRESSION_PREP, else stays.
  zeroing finished but sensing.zero_position() fails -> ABORT, not
  COMPRESSION_PREP. audio finished -> COMPRESSION, else stays.
- Pause button (not Next) -> COMPRESSION to PAUSE. Next -> PAUSE to
  COMPRESSION_PREP. Next -> KNEEL_FAILURE to ALIGNMENT.
- Error routing: battery failure in STARTUP; zeroing or sensor failure in
  ZEROING; sensor failure in COMPRESSION/PAUSE -> ABORT. motor failure from
  abort_compressions() in PAUSE's ongoing loop (not just its setup) ->
  ABORT. IMU kneel in ZEROING/COMPRESSION/PAUSE -> KNEEL_FAILURE.
- KNEEL_FAILURE setup: repeated IMU kneel from pause_compressions() ->
  screen still shown, no loop. any other error -> ABORT.
- KNEEL_FAILURE loop: no Next press -> stays put, no ALIGNMENT transition,
  pause_compressions() not called again (only user acknowledgement is
  needed, no ongoing sensor/actuation monitoring).
- pump_events() called once per main loop tick, in every state. a
  WARNING_PYGAME_PUMP_FAILURE from it doesn't disrupt state routing.
- ABORT is terminal: once entered, stays entered exactly once even as the
  loop keeps ticking. retract_carriage() failing in ABORT's loop -> setup
  still runs only once, retract keeps being retried every tick.