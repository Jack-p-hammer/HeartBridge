// stepper_buttons.ino
//
// Two buttons drive both TB6612 stepper drivers on an Arduino Nano. Both
// motors always turn together, in the same direction.
//   Forward button: the motors start turning forward and keep going.
//   Reverse button: the motors turn backward instead.
//   Press the button for the direction they're already going to stop them.
// Set HOLD_TO_RUN to true if you'd rather the motors only turn while a button
// is held down. The coils switch off whenever the motors stop, so nothing sits
// there heating up. The Serial Monitor (115200 baud) shows forward/reverse/stopped.
//
// Wiring, Nano pin -> TB6612 breakout pin:
//   Driver 1 (motor 1): D9 -> AIN1, D10 -> AIN2, D11 -> BIN1, D12 -> BIN2
//   Driver 2 (motor 2): D5 -> AIN1, D6 -> AIN2, D7 -> BIN1, D8 -> BIN2
//   On both breakouts:
//     PWMA and PWMB -> 5V (held high; a stepper doesn't use speed PWM)
//     VCC  -> Nano 5V
//     GND  -> Nano GND and motor supply ground
//     VM   -> separate motor supply, 4.5-13.5 V (the Nano's 5V pin can't power two steppers)
//     STBY -> leave unconnected (pulled up on the breakout)
//     One stepper coil on MOTORA, the other on MOTORB
//   The TB6612 doesn't limit current, so pick VM so that VM / coil resistance
//   stays under 1.2 A.
//
// Buttons (no resistors needed: the sketch turns on the Nano's internal pull-ups):
//   Forward button: one leg -> D2, the diagonally opposite leg -> GND
//   Reverse button: one leg -> D3, the diagonally opposite leg -> GND
//   A 4-leg tactile button has its legs joined in pairs. The diagonal pair is
//   always the switched one, whichever way the button sits in the breadboard.
//
// If a motor buzzes instead of turning, a coil pair is wrong. If it turns the
// wrong way, swap the two wires of one coil.
//
// Upload: open this file in the Arduino IDE, choose Tools -> Board -> Arduino
// Nano, pick the port under Tools -> Port, then Upload. If the upload fails
// with "not in sync", set Tools -> Processor to "ATmega328P (Old Bootloader)".

// ---- Settings ----
const int STEPS_PER_REV = 200;         // full steps per revolution (1.8 degree motor)
const float RPM = 40.0;                // motor speed
const bool HALF_STEPS = false;         // false: full steps with both coils on (most torque)
                                       // true: half steps (smoother, twice the steps per rev)
const bool HOLD_TO_RUN = false;        // false: press to start, switch direction, or stop
                                       // true: motors turn only while a button is held
const unsigned long DEBOUNCE_MS = 20;  // a button has to read steady this long to count

// ---- Pins ----
const uint8_t FORWARD_BUTTON_PIN = 2;
const uint8_t REVERSE_BUTTON_PIN = 3;
const uint8_t MOTOR_PINS[2][4] = {  // AIN1, AIN2, BIN1, BIN2
  {9, 10, 11, 12},  // driver 1
  {5, 6, 7, 8},     // driver 2
};

// Coil drive for each half-step phase, as {AIN1, AIN2, BIN1, BIN2}. On the TB6612,
// IN1 high + IN2 low pushes current through a coil one way, IN1 low + IN2 high
// the other way, and both low lets the coil go slack. Odd rows have both coils
// on, so full steps use those; half steps walk through all eight.
const uint8_t PHASES[8][4] = {
  {1, 0, 0, 0},  // A+
  {1, 0, 1, 0},  // A+ B+
  {0, 0, 1, 0},  //    B+
  {0, 1, 1, 0},  // A- B+
  {0, 1, 0, 0},  // A-
  {0, 1, 0, 1},  // A- B-
  {0, 0, 0, 1},  //    B-
  {1, 0, 0, 1},  // A+ B-
};
const uint8_t COILS_OFF[4] = {0, 0, 0, 0};

const int STEPS_PER_TURN = STEPS_PER_REV * (HALF_STEPS ? 2 : 1);  // steps actually taken per rev
const unsigned long STEP_INTERVAL_US = (unsigned long)(60.0e6 / (RPM * STEPS_PER_TURN));
const int PHASE_STEP = HALF_STEPS ? 1 : 2;  // full steps skip every other row

uint8_t phase = 1;             // current row of PHASES (starts odd, for full steps); both motors share it
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

// Drive both motors' pins for one row of PHASES (or COILS_OFF).
void writeCoils(const uint8_t coils[4]) {
  for (int m = 0; m < 2; m++) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(MOTOR_PINS[m][i], coils[i]);
    }
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
    writeCoils(COILS_OFF);  // coils off: shaft turns freely, no current draw
    Serial.println(F("stopped"));
  } else {
    if (direction == 0) {
      lastStepUs = micros() - STEP_INTERVAL_US;  // starting up: first step goes out right away
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
  for (int m = 0; m < 2; m++) {
    for (int i = 0; i < 4; i++) {
      pinMode(MOTOR_PINS[m][i], OUTPUT);
    }
  }
  writeCoils(COILS_OFF);
  Serial.println(F("TB6612 stepper buttons: D2 = forward, D3 = reverse"));
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

  if (direction != 0 && micros() - lastStepUs >= STEP_INTERVAL_US) {
    lastStepUs += STEP_INTERVAL_US;  // fixed spacing, so the speed doesn't drift
    phase = (phase + 8 + direction * PHASE_STEP) % 8;
    writeCoils(PHASES[phase]);
  }
}
