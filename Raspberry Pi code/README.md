# HeartBridge Raspberry Pi code

The Python controller for HeartBridge. `src/main.py` runs the CPR state machine; `sensing.py`, `actuation.py` and `HMI.py` handle the sensors, the moteus motor controller, and the screen, audio, buttons and lasers.

All commands below are run from inside this folder (`Raspberry Pi code/`). The folder name has spaces, so quote it when you `cd` into it:

```bash
cd "Raspberry Pi code"
```

## Code writing notes

Use `logging.debug/.warning/.error` instead of `print` statements. That way messages land in the HeartBridge log files, not just the console.

---

## First-time setup

Do this once per machine (the Pi or a laptop).

### 1. Raspberry Pi only: system setup

1. Enable I2C (used by the time-of-flight and IMU sensors): `sudo raspi-config` → Interface Options → I2C → Enable, then reboot.
2. Install the pigpio daemon, which the code uses for buttons, LEDs and lasers:
   ```bash
   sudo apt install pigpio
   ```
3. Optional: start the daemon automatically on every boot, so you don't have to run `sudo pigpiod` each time:
   ```bash
   sudo systemctl enable --now pigpiod
   ```

This should all be done aready.

### 2. Create the virtual environment

Create the environment:

```bash
python -m venv .venv
```

Activate it:

- **Raspberry Pi / Linux / macOS:** `source .venv/bin/activate`
- **Windows (PowerShell):** `.venv\Scripts\Activate.ps1`
- **Windows (Git Bash):** `source .venv/Scripts/activate`

Your prompt should now start with `(.venv)`.

### 3. Install the requirements

```bash
pip install -r requirements.txt
```

### 4. Run code tests

```bash
python -m pytest -q
```

The tests use fake hardware, so they run on a laptop too. See [tests/README_TESTING.md](tests/README_TESTING.md).

---

## When `requirements.txt` changes

After pulling changes that touch `requirements.txt`, activate the environment (step 2 above) and reinstall:

```bash
pip install -r requirements.txt
```

This installs anything new and upgrades anything whose version changed. It does **not** remove packages that were taken out of `requirements.txt`.

---

## Running the machine (on the Pi)

1. Make sure the pigpio daemon is running. If it wasn't enabled at boot, run `sudo pigpiod`.
2. Activate the environment.
3. Start the controller:
   ```bash
   python src/main.py
   ```

Running over SSH works: if no display is set, the screen and audio are sent to the Pi's own desktop. Each run writes a log to `Logs/heartBridge_<timestamp>.log`.

### Demo mode

`python demo/main.py` runs the same state machine from `src/`, but with `demo/sensing.py` in place of `src/sensing.py`. The force sensor, ToF sensor and IMU aren't used, and zeroing stops when the carriage reaches `DEMO_ZERO_POSITION_M` (set in `demo/sensing.py`). The motor and steppers still move. Demo runs log to `Logs/heartBridge_demo_<timestamp>.log`.

The demo only needs the Next and Pause buttons, the steppers and their drivers (STEP on GPIO 20, DIR on GPIO 21, EN on GPIO 16, see `src/stepper.py`), the moteus and its CAN adapter, the screen and the speaker. No sensors, lasers or keyboard.

**Button-only tour.** With no IMU or other sensors, the Pause presses during compressions take the machine through the states the sensors would trigger, in order (`PAUSE_PRESS_TOUR` in `demo/main.py`):

1. 1st Pause press: PAUSE as normal. Press Next to resume.
2. 2nd Pause press: kneel failure. Press Next to go back to alignment, and on through zeroing to compressions.
3. 3rd Pause press: ABORT. The tour ends here; only a restart leaves ABORT (Ctrl+C in the terminal stops the demo cleanly).

After the tour, Pause presses are normal pauses. Set `PAUSE_PRESS_TOUR = []` to turn it off.

Optional keys, on a keyboard plugged into the Pi (the screen window gets them, not an SSH terminal):

- **Esc** or **Q**: stop the motor and exit. Ctrl+C in the terminal does the same.
- **K**: simulate a kneel failure, which goes to the kneel failure screen.
- **A**: simulate a sensor failure, which goes to ABORT. Only a restart leaves ABORT.

K and A take effect the next time the sensors are read: during zeroing, compressions, or on entering pause.

## Bench and bring-up scripts

Run these the same way, e.g. `python3 scripts/slow_jog.py`.

- `scripts/`: bench tools
  - `slow_jog.py`: slow jog test for the plunger motor
  - `spin_test.py`: spin the motor a set number of turns and back
  - `set_position_limits.py`: set the moteus position limits and save them to flash
  - `lasers.py`, `laser_toggle.py`: toggle the alignment lasers
  - `I2C_scanner.py`: list devices on the I2C bus
- `PCB validation/`: board validation checks for the buttons, force sensor and lasers