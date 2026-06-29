# Testing the CPR Machine Code

This folder contains simple tests for the parts of the project that can be checked without the real hardware attached.

## Why these tests exist
The machine depends on sensors, buttons, LEDs, and a motor controller. Those pieces are hard to test on a laptop, so these tests use small fake versions of the hardware interfaces instead.

That means we can still check that the Python code behaves the way we expect.

## What is covered
- HMI behavior such as startup setup and button control
- Sensing behavior such as initialization and placeholder-safe helper functions
- Actuation placeholder behavior for the motor-related functions
- Main-state helper behavior for the control flow and error routing

## How to run the tests
From the project root, run:

```bash
python -m pytest -q
```

If you only want to run the tests in this folder, that same command will do it.

## Notes for future contributors
- These tests are intentionally simple and use fake hardware.
- They are meant to verify logic and basic behavior, not to test the real physical device.
- If new hardware logic is added, add or update tests in this folder.


## Current test inventory
### HMI tests
- Checks that HMI startup sets up the display and GPIO pins correctly.
- Checks that button helpers write the expected LED and button states.
- Checks that HMI initialization fails gracefully when pygame startup fails.
- Checks that button readers and laser controls respond to fake GPIO input.
- Checks that the audio-finished helper reports the current mixer state.

### Sensing tests
- Checks that sensing initialization fails gracefully when the pigpio connection is unavailable.
- Checks that sensing initialization succeeds with the fake hardware stack in place.
- Checks that sensing initialization fails gracefully when the I2C bus cannot be created.
- Checks that the current sensor helper functions behave safely as placeholders.

### Actuation tests
- Checks that the current motor placeholder functions report normal operation.
- Checks that those placeholder functions return enum values rather than raw values.

### Main-state tests
- Checks that the main flow advances through the early startup states when the next button is pressed.
- Checks that initialization errors are routed to the abort state.
- Checks that the fatal-error set includes the expected terminal conditions.