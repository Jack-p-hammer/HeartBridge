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

## Bench and bring-up scripts

Run these the same way, e.g. `python3 scripts/slow_jog.py`.

- `scripts/`: bench tools
  - `slow_jog.py`: slow jog test for the plunger motor
  - `spin_test.py`: spin the motor a set number of turns and back
  - `set_position_limits.py`: set the moteus position limits and save them to flash
  - `lasers.py`, `laser_toggle.py`: toggle the alignment lasers
  - `I2C_scanner.py`: list devices on the I2C bus
- `PCB validation/`: board validation checks for the buttons, force sensor and lasers