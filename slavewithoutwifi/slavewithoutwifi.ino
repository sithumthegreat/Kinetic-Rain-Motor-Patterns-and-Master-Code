#include <AccelStepper.h>

// ═══════════════════════════════════════════════════════
// TRAVEL CONSTANTS
// ═══════════════════════════════════════════════════════
const long FULL    = 45000;
const long THREE_Q = 33750;
const long HALF    = 22500;
const long THIRD   = 15000;
const long QTR     = 11250;

const unsigned long HOME_TIMEOUT_MS = 180000; // 3 minutes

// ═══════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════
const int ENABLE_PIN = 12;

AccelStepper m1(AccelStepper::DRIVER, 25, 26);
AccelStepper m2(AccelStepper::DRIVER, 32, 33);
AccelStepper m3(AccelStepper::DRIVER, 22, 23);
AccelStepper m4(AccelStepper::DRIVER, 27,  4);
AccelStepper m5(AccelStepper::DRIVER, 16, 17);

AccelStepper* motors[] = {&m1, &m2, &m3, &m4, &m5};
const int homePins[]   = {21, 19, 18, 14, 13};

// ═══════════════════════════════════════════════════════
// FUZZY VELOCITY CONTROL
// ═══════════════════════════════════════════════════════
float calculateFuzzySpeed(float distanceToGo, float maxSpeed) {
  float d = abs(distanceToGo);
  if (d < 1000) return maxSpeed * 0.3f;
  if (d > 7500) return maxSpeed;
  return (maxSpeed * 0.3f) + (maxSpeed * 0.7f * ((d - 1000.0f) / 6500.0f));
}

void applyFuzzyControl() {
  for (int i = 0; i < 5; i++)
    motors[i]->setMaxSpeed(calculateFuzzySpeed(motors[i]->distanceToGo(), 2500));
}

// ═══════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════
void runAllUntilDone() {
  while (m1.distanceToGo() != 0 || m2.distanceToGo() != 0 ||
         m3.distanceToGo() != 0 || m4.distanceToGo() != 0 ||
         m5.distanceToGo() != 0) {
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }
}

void moveAll(long pos) {
  m1.moveTo(pos); m2.moveTo(pos); m3.moveTo(pos);
  m4.moveTo(-pos);
  m5.moveTo(pos);
  runAllUntilDone();
}

void moveOne(AccelStepper &motor, long pos) {
  if (&motor == &m4) motor.moveTo(-pos);
  else               motor.moveTo(pos);
  while (motor.distanceToGo() != 0) {
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }
}

// ═══════════════════════════════════════════════════════
// HOMING — exact same as slave code
// ═══════════════════════════════════════════════════════
void homeAllMotors() {
  bool h[5]         = {false, false, false, false, false};
  unsigned long tStart[5];
  unsigned long now = millis();
  for (int i = 0; i < 5; i++) tStart[i] = now;

  m1.moveTo(-999999); m2.moveTo(-999999); m3.moveTo(-999999);
  m4.moveTo( 999999);
  m5.moveTo(-999999);

  while (!(h[0] && h[1] && h[2] && h[3] && h[4])) {
    unsigned long t = millis();
    for (int i = 0; i < 5; i++) {
      if (h[i]) continue;
      if (digitalRead(homePins[i]) == LOW) {
        motors[i]->stop();
        motors[i]->setCurrentPosition(0);
        h[i] = true;
      }
      else if (t - tStart[i] >= HOME_TIMEOUT_MS) {
        motors[i]->stop();
        motors[i]->setCurrentPosition(0);
        h[i] = true;
      }
    }
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }
}

// ═══════════════════════════════════════════════════════
// PATTERNS — exact same as slave code
// ═══════════════════════════════════════════════════════
void patternOne() {
  moveAll(FULL);  delay(500);
  moveAll(0);     delay(500);
  moveAll(HALF);  delay(300);
  moveAll(0);     delay(1000);
}

void patternTwo() {
  moveOne(m1, FULL); delay(100);
  moveOne(m2, FULL); delay(100);
  moveOne(m3, FULL); delay(100);
  moveOne(m4, FULL); delay(100);
  moveOne(m5, FULL); delay(500);
  moveAll(0);
}

void patternThree() {
  for (int r = 0; r < 3; r++) {
    m1.moveTo(FULL);  m3.moveTo(FULL);  m5.moveTo(FULL);
    m2.moveTo(QTR);   m4.moveTo(-QTR);
    runAllUntilDone(); delay(400);
    m1.moveTo(QTR);   m3.moveTo(QTR);   m5.moveTo(QTR);
    m2.moveTo(FULL);  m4.moveTo(-FULL);
    runAllUntilDone(); delay(400);
  }
  moveAll(0); delay(1000);
}

void patternFour() {
  m1.moveTo(THIRD);  m2.moveTo(THREE_Q);
  m3.moveTo(FULL);   m4.moveTo(-THREE_Q); m5.moveTo(THIRD);
  runAllUntilDone(); delay(800);
  moveAll(0); delay(500);
  m1.moveTo(FULL);   m2.moveTo(THREE_Q);
  m3.moveTo(THIRD);  m4.moveTo(-THREE_Q); m5.moveTo(FULL);
  runAllUntilDone(); delay(800);
  moveAll(0); delay(1000);
}

// ═══════════════════════════════════════════════════════
// SETUP — home then run all patterns once
// ═══════════════════════════════════════════════════════
void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  for (int i = 0; i < 5; i++) {
    pinMode(homePins[i], INPUT_PULLUP);
    motors[i]->setMaxSpeed(2500);
    motors[i]->setAcceleration(2000);
    motors[i]->setMinPulseWidth(20);
  }

  // Home first
  homeAllMotors();
  delay(2000);

  // Run all 4 patterns in sequence
  patternOne();
  delay(2000);

  patternTwo();
  delay(2000);

  patternThree();
  delay(2000);

  patternFour();
  delay(2000);

  // Return to home when done
  moveAll(0);
}

// ═══════════════════════════════════════════════════════
// LOOP — repeats patterns continuously after setup
// Comment out loop body if you only want one run
// ═══════════════════════════════════════════════════════
void loop() {
  patternOne();   delay(2000);
  patternTwo();   delay(2000);
  patternThree(); delay(2000);
  patternFour();  delay(2000);
}