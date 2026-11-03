// stepper_test.ino
//
// Breadboard test for the two TB6612 stepper drivers on an Arduino Nano.
// Once uploaded it runs on its own. USB only powers the Nano and shows what
// it's doing in the Serial Monitor (115200 baud).
//
// Test sequence, repeated forever:
//   1. motor 1 alone: 1 rev forward, then back
//   2. motor 2 alone: 1 rev forward, then back
//   3. both motors together: 1 rev forward, then back
//   4. coils off for 2 s
// If a motor sits still during its solo move, check that driver's wiring.
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
// If a motor buzzes instead of turning, a coil pair is wrong. If it turns the
// wrong way, swap the two wires of one coil.
//
// Upload: open this file in the Arduino IDE, choose Tools -> Board -> Arduino
// Nano, pick the port under Tools -> Port, then Upload. If the upload fails
// with "not in sync", set Tools -> Processor to "ATmega328P (Old Bootloader)".

// ---- Settings ----
const int STEPS_PER_REV = 200;       // full steps per revolution (1.8 degree motor)
const float RPM = 30.0;              // speed of every move
const float REVS = 1.0;              // size of each move, in revolutions
const bool HALF_STEPS = false;       // false: full steps with both coils on (most torque)
                                     // true: half steps (smoother, twice the steps per rev)
const unsigned long PAUSE_MS = 500;  // pause after each move, coils on (holding)
const unsigned long REST_MS = 2000;  // coils off at the end of each round

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

const int STEPS_PER_TURN = STEPS_PER_REV * (HALF_STEPS ? 2 : 1);  // steps actually taken per rev
const long STEPS_PER_MOVE = lround(REVS * STEPS_PER_TURN);
const unsigned long STEP_INTERVAL_US = (unsigned long)(60.0e6 / (RPM * STEPS_PER_TURN));
const int PHASE_STEP = HALF_STEPS ? 1 : 2;  // full steps skip every other row

struct Motor {
  uint8_t pins[4];           // AIN1, AIN2, BIN1, BIN2
  uint8_t phase;             // current row of PHASES (starts odd, for full steps)
  long stepsLeft;            // steps still to go in the current move
  int8_t direction;          // +1 forward, -1 backward
  unsigned long lastStepUs;  // micros() of the last step
};

Motor motors[] = {
  {{9, 10, 11, 12}, 1, 0, 1, 0},  // driver 1
  {{5, 6, 7, 8}, 1, 0, 1, 0},     // driver 2
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// Drive a motor's four pins for its current phase.
void energize(Motor &m) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(m.pins[i], PHASES[m.phase][i]);
  }
}

// Coils off: the shaft turns freely and the motor stops drawing current.
void release(Motor &m) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(m.pins[i], LOW);
  }
}

// Queue a move of `steps` steps (negative = backward). runMoves() does the stepping.
void startMove(Motor &m, long steps) {
  m.direction = steps >= 0 ? 1 : -1;
  m.stepsLeft = labs(steps);
  m.lastStepUs = micros() - STEP_INTERVAL_US;  // first step goes out right away
}

// Step every motor with a queued move until all of them have finished. Each
// motor keeps its own timing, so moves on both motors run at the same time.
void runMoves() {
  bool busy = true;
  while (busy) {
    busy = false;
    for (int k = 0; k < NUM_MOTORS; k++) {
      Motor &m = motors[k];
      if (m.stepsLeft == 0) {
        continue;
      }
      busy = true;
      if (micros() - m.lastStepUs >= STEP_INTERVAL_US) {
        m.lastStepUs += STEP_INTERVAL_US;  // fixed spacing, so the speed doesn't drift
        m.phase = (m.phase + 8 + m.direction * PHASE_STEP) % 8;
        energize(m);
        m.stepsLeft--;
      }
    }
  }
  delay(STEP_INTERVAL_US / 1000 + 1);  // give the last step time to land
}

// Move the chosen motors `steps` steps at the same time, then pause with the coils on.
void moveMotors(bool useMotor1, bool useMotor2, long steps) {
  if (useMotor1) startMove(motors[0], steps);
  if (useMotor2) startMove(motors[1], steps);
  runMoves();
  delay(PAUSE_MS);
}

// Forward REVS, back to the start, then coils off.
void testMotors(bool useMotor1, bool useMotor2, const __FlashStringHelper *label) {
  Serial.print(label);
  Serial.println(F(": forward"));
  moveMotors(useMotor1, useMotor2, STEPS_PER_MOVE);
  Serial.print(label);
  Serial.println(F(": back"));
  moveMotors(useMotor1, useMotor2, -STEPS_PER_MOVE);
  release(motors[0]);
  release(motors[1]);
}

void setup() {
  Serial.begin(115200);
  for (int k = 0; k < NUM_MOTORS; k++) {
    for (int i = 0; i < 4; i++) {
      pinMode(motors[k].pins[i], OUTPUT);
    }
    release(motors[k]);
  }
  Serial.println(F("TB6612 stepper test"));
  Serial.print(F("Steps per move: "));
  Serial.print(STEPS_PER_MOVE);
  Serial.print(F(", one step every "));
  Serial.print(STEP_INTERVAL_US);
  Serial.println(F(" us"));
  delay(1000);
}

void loop() {
  testMotors(true, false, F("motor 1"));
  testMotors(false, true, F("motor 2"));
  testMotors(true, true, F("both motors"));
  Serial.println(F("coils off"));
  delay(REST_MS);
}
