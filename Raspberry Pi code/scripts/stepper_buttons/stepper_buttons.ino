// stepper_buttons.ino
//
// Two buttons drive both DRV8434 stepper drivers on an Arduino Nano. Both
// motors always turn together, in the same direction.
//   Forward button: the motors start turning forward and keep going.
//   Reverse button: the motors turn backward instead.
//   Press the button for the direction they're already going to stop them.
// Set HOLD_TO_RUN to true if you'd rather the motors only turn while a button
// is held down. The drivers go to sleep whenever the motors stop, so nothing
// sits there heating up. The Serial Monitor (115200 baud) shows forward/reverse/stopped.
//
// Wiring, Nano pin -> Pololu DRV8434 carrier (#3762) pin:
//   Driver 1 (motor 1): D9 -> STEP, D10 -> DIR
//   Driver 2 (motor 2): D5 -> STEP, D6  -> DIR
//   Both drivers:       D7 -> SLEEP (one Nano pin wakes/sleeps both boards;
//                       the carrier pulls SLEEP low, so it stays asleep until D7 goes high)
//   On both carriers:
//     EN     -> leave unconnected (floating = enabled; sleep is what turns the coils off)
//     M0, M1 -> both to GND = full steps. Left unconnected the carrier defaults to
//               1/128 steps and the motors would barely move. MICROSTEPS below must
//               match; Pololu's product page has the full M0/M1 table.
//     DECAY0, DECAY1, TOFF -> leave unconnected (default smart tune decay)
//     FAULT  -> leave unconnected
//     GND    -> Nano GND and motor supply ground
//     VIN    -> separate motor supply, 6-48 V (below 6 V the max current drops), with
//               a 47 uF+ capacitor across VIN/GND close to the board
//     One stepper coil on the A outputs, the other on the B outputs
//     No logic supply pin: the carrier makes its own from VIN.
//   Unlike the TB6612, the DRV8434 limits coil current itself. Set the limit
//   with the board's trimpot BEFORE connecting a motor: current limit = VREF / 1.32,
//   so 1.32 V = 1 A. Measure VREF at the small hole circled on the bottom
//   silkscreen. Keep it at or below the motor's rated current per phase, and
//   under ~1.2 A (VREF 1.58 V) without a heatsink. Never connect or disconnect a
//   motor while VIN is powered: that can destroy the driver.
//
// Buttons (no resistors needed: the sketch turns on the Nano's internal pull-ups):
//   Forward button: one leg -> D2, the diagonally opposite leg -> GND
//   Reverse button: one leg -> D3, the diagonally opposite leg -> GND
//   A 4-leg tactile button has its legs joined in pairs. The diagonal pair is
//   always the switched one, whichever way the button sits in the breadboard.
//
// If a motor buzzes instead of turning, a coil pair is wrong. If it turns the
// wrong way, swap the two wires of one coil (or flip that motor's entry in
// DIR_INVERT).
//
// Upload: open this file in the Arduino IDE, choose Tools -> Board -> Arduino
// Nano, pick the port under Tools -> Port, then Upload. If the upload fails
// with "not in sync", set Tools -> Processor to "ATmega328P (Old Bootloader)".

// ---- Settings ----
const int STEPS_PER_REV = 200;         // full steps per revolution (1.8 degree motor)
const int MICROSTEPS = 1;              // must match the M0/M1 wiring: 1 = full steps
                                       // (M0 and M1 to GND); 128 if both are left
                                       // unconnected (the carrier's default)
const float RPM = 40.0;                // motor speed
const bool HOLD_TO_RUN = false;        // false: press to start, switch direction, or stop
                                       // true: motors turn only while a button is held
const unsigned long DEBOUNCE_MS = 20;  // a button has to read steady this long to count

// ---- Pins ----
const uint8_t FORWARD_BUTTON_PIN = 2;
const uint8_t REVERSE_BUTTON_PIN = 3;
const uint8_t SLEEP_PIN = 7;                  // nSLEEP on both drivers: low = asleep, coils off
const uint8_t STEP_PINS[2] = {9, 5};          // driver 1, driver 2
const uint8_t DIR_PINS[2] = {10, 6};          // driver 1, driver 2
const bool DIR_INVERT[2] = {false, false};    // true flips that motor's direction

const unsigned long STEP_INTERVAL_US = (unsigned long)(60.0e6 / (RPM * STEPS_PER_REV * MICROSTEPS));
const unsigned long WAKE_US = 2000;  // DRV8434 needs ~1.2 ms after nSLEEP goes high before it takes steps
const unsigned int STEP_PULSE_US = 2;  // STEP high time; the DRV8434 needs about 1 us

int8_t direction = 0;          // +1 forward, -1 reverse, 0 stopped
unsigned long lastStepUs = 0;  // micros() of the last step

struct Button {
  uint8_t pin;
  bool pressed;              // debounced state
  bool lastReading;          // raw state on the previous check
  unsigned long changedMs;   // millis() when the raw state last changed
};

Button forwardButton = {FORWARD_BUTTON_PIN, false, false, 0};
Button reverseButton = {REVERSE_BUTTON_PIN, false, false, 0};

// Point both motors' DIR pins forward (+1) or backward (-1).
void writeDirection(int8_t dir) {
  for (int m = 0; m < 2; m++) {
    bool high = (dir > 0) != DIR_INVERT[m];
    digitalWrite(DIR_PINS[m], high ? HIGH : LOW);
  }
}

// Send one step pulse to both drivers. The driver steps on STEP's rising edge.
void pulseStep() {
  for (int m = 0; m < 2; m++) {
    digitalWrite(STEP_PINS[m], HIGH);
  }
  delayMicroseconds(STEP_PULSE_US);
  for (int m = 0; m < 2; m++) {
    digitalWrite(STEP_PINS[m], LOW);
  }
}

// Update a button's debounced state. Returns true once for each new press.
// Contacts bounce for a few ms when pressed, so a change only counts once the
// reading has stayed the same for DEBOUNCE_MS.
bool newPress(Button &b) {
  bool reading = digitalRead(b.pin) == LOW;  // pressing connects the pin to GND
  unsigned long now = millis();
  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.changedMs = now;
  }
  if (reading != b.pressed && now - b.changedMs >= DEBOUNCE_MS) {
    b.pressed = reading;
    return reading;
  }
  return false;
}

// Start, stop, or reverse the motors.
void setDirection(int8_t newDirection) {
  if (newDirection == direction) {
    return;
  }
  if (newDirection == 0) {
    digitalWrite(SLEEP_PIN, LOW);  // sleep: coils off, shaft turns freely, no current draw
    Serial.println(F("stopped"));
  } else {
    writeDirection(newDirection);
    if (direction == 0) {
      // Waking up: the first step goes out once the driver has had WAKE_US to start.
      digitalWrite(SLEEP_PIN, HIGH);
      lastStepUs = micros() + WAKE_US - STEP_INTERVAL_US;
    }
    // Reversing while running keeps the step timing, so the motor gets a full
    // step period before its first step the other way.
    Serial.println(newDirection > 0 ? F("forward") : F("reverse"));
  }
  direction = newDirection;
}

void setup() {
  Serial.begin(115200);
  pinMode(FORWARD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(REVERSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);  // start asleep
  for (int m = 0; m < 2; m++) {
    pinMode(STEP_PINS[m], OUTPUT);
    pinMode(DIR_PINS[m], OUTPUT);
    digitalWrite(STEP_PINS[m], LOW);
  }
  writeDirection(1);
  Serial.println(F("DRV8434 stepper buttons: D2 = forward, D3 = reverse"));
}

void loop() {
  bool forwardPressed = newPress(forwardButton);
  bool reversePressed = newPress(reverseButton);

  if (HOLD_TO_RUN) {
    // Turn while exactly one button is held, stop otherwise.
    if (forwardButton.pressed && !reverseButton.pressed) {
      setDirection(1);
    } else if (reverseButton.pressed && !forwardButton.pressed) {
      setDirection(-1);
    } else {
      setDirection(0);
    }
  } else if (forwardPressed) {
    setDirection(direction == 1 ? 0 : 1);  // already going forward -> stop
  } else if (reversePressed) {
    setDirection(direction == -1 ? 0 : -1);  // already going backward -> stop
  }

  // Signed difference, so the wake-up delay (lastStepUs set slightly ahead) works.
  if (direction != 0 && (long)(micros() - lastStepUs) >= (long)STEP_INTERVAL_US) {
    lastStepUs += STEP_INTERVAL_US;  // fixed spacing, so the speed doesn't drift
    pulseStep();
  }
}
