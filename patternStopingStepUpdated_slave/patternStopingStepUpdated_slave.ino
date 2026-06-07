#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ═══════════════════════════════════════════════════════
// SLAVE ID - CHANGE THIS FOR EACH SLAVE (1 through 10)
// ═══════════════════════════════════════════════════════
const int SLAVE_ID = 8; // ⚠️ UPDATE THIS FOR EACH SLAVE!

// ═══════════════════════════════════════════════════════
// MASTER MAC ADDRESS (same for all slaves)
// ═══════════════════════════════════════════════════════
uint8_t masterAddress[] = {0xF4, 0x2D, 0xC9, 0x71, 0x28, 0xB0};

// ═══════════════════════════════════════════════════════
// TRAVEL CONSTANTS (90,000 steps = full travel)
// ═══════════════════════════════════════════════════════
const long FULL    = 90000;
const long THREE_Q = 67500;
const long HALF    = 45000;
const long THIRD   = 30000;
const long QTR     = 22500;

// ═══════════════════════════════════════════════════════
// HOMING TIMEOUT (3 minutes per motor)
// ═══════════════════════════════════════════════════════
const unsigned long HOME_TIMEOUT_MS  = 180000;
const unsigned long DEBOUNCE_MS      = 1; // Pin must stay LOW this long to confirm real contact

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
// COMMUNICATION STRUCTURE
// ═══════════════════════════════════════════════════════
typedef struct {
  char command[32];
  int  value;
  int  slaveId;
} Message;

Message incomingMsg;
Message outgoingMsg;

// ═══════════════════════════════════════════════════════
// STATE FLAGS
// ═══════════════════════════════════════════════════════
volatile int activePattern = 0; // 0=Idle, 1-4=Pattern, 99=Homing
bool connectedToMaster     = false;
int  foundChannel          = 1;

// ═══════════════════════════════════════════════════════
// FUZZY VELOCITY CONTROL
// Throttled to every 20ms to avoid interfering with
// step pulse timing during heavy float calculations
// ═══════════════════════════════════════════════════════
unsigned long lastFuzzyUpdate = 0;

float calculateFuzzySpeed(float distanceToGo, float maxSpeed) {
  float d = abs(distanceToGo);
  if (d < 2000)  return maxSpeed * 0.3f;  // slow zone
  if (d > 15000) return maxSpeed;          // fast zone
  return (maxSpeed * 0.3f) + (maxSpeed * 0.7f * ((d - 2000.0f) / 13000.0f));
}

void applyFuzzyControl() {
  unsigned long now = millis();
  if (now - lastFuzzyUpdate >= 20) {
    lastFuzzyUpdate = now;
    for (int i = 0; i < 5; i++)
      motors[i]->setMaxSpeed(calculateFuzzySpeed(motors[i]->distanceToGo(), 3500));
  }
}

// ═══════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════
void runAllUntilDone() {
  while (m1.distanceToGo() != 0 || m2.distanceToGo() != 0 ||
         m3.distanceToGo() != 0 || m4.distanceToGo() != 0 ||
         m5.distanceToGo() != 0) {
    if (activePattern == 0) return; // Emergency stop
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }
}

void moveAll(long pos) {
  m1.moveTo(pos); m2.moveTo(pos); m3.moveTo(pos);
  m4.moveTo(-pos); // Motor 4 reversed
  m5.moveTo(pos);
  runAllUntilDone();
}

void moveOne(AccelStepper &motor, long pos) {
  if (&motor == &m4) motor.moveTo(-pos);
  else               motor.moveTo(pos);
  while (motor.distanceToGo() != 0) {
    if (activePattern == 0) return;
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }
}

// ═══════════════════════════════════════════════════════
// HOMING
// - 2ms software debounce on each home pin
//   Pin must stay LOW continuously for 2ms to confirm
//   real nail contact vs electrical noise spike
// - If pin goes HIGH before 2ms — resets, was noise
// - 3 minute timeout per motor as safety fallback
// ═══════════════════════════════════════════════════════
void homeAllMotors() {
  Serial.println("🏠 Homing...");

  bool  h[5]            = {false, false, false, false, false};
  unsigned long tStart[5];       // homing start time per motor
  unsigned long lowSince[5];     // when pin first went LOW
  bool  pinWasLow[5]    = {false, false, false, false, false};

  unsigned long now = millis();
  for (int i = 0; i < 5; i++) {
    tStart[i]  = now;
    lowSince[i] = 0;
  }

  // Drive all motors toward home switches
  m1.moveTo(-999999); m2.moveTo(-999999); m3.moveTo(-999999);
  m4.moveTo( 999999); // Reversed
  m5.moveTo(-999999);

  while (!(h[0] && h[1] && h[2] && h[3] && h[4])) {
    unsigned long t = millis();

    for (int i = 0; i < 5; i++) {
      if (h[i]) continue;

      bool pinLow = (digitalRead(homePins[i]) == LOW);

      if (pinLow) {
        if (!pinWasLow[i]) {
          // Pin just went LOW — start debounce timer
          pinWasLow[i] = true;
          lowSince[i]  = t;
        } else if (t - lowSince[i] >= DEBOUNCE_MS) {
          // Pin has stayed LOW for DEBOUNCE_MS — real contact confirmed
          motors[i]->stop();
          motors[i]->setCurrentPosition(0);
          h[i] = true;
          Serial.printf("  ✓ M%d homed (nail contact)\n", i + 1);
        }
        // else still within debounce window — keep waiting
      } else {
        // Pin went HIGH — was noise, reset debounce
        if (pinWasLow[i]) {
          Serial.printf("  ~ M%d noise spike rejected\n", i + 1);
        }
        pinWasLow[i] = false;
        lowSince[i]  = 0;
      }

      // Timeout safety — force zero if no contact within 3 minutes
      if (!h[i] && (t - tStart[i] >= HOME_TIMEOUT_MS)) {
        motors[i]->stop();
        motors[i]->setCurrentPosition(0);
        h[i] = true;
        Serial.printf("  ⚠️ M%d homing TIMEOUT — force-zeroed\n", i + 1);
      }
    }

    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }

  Serial.println("✅ Homing complete!");
  sendToMaster("HOMING_DONE", 1);
}

// ═══════════════════════════════════════════════════════
// PATTERNS
// ═══════════════════════════════════════════════════════
void patternOne() {
  moveAll(FULL);   delay(500);
  moveAll(5000);     delay(500);
  moveAll(HALF);   delay(300);
  moveAll(5000);     delay(1000);
}

void patternTwo() {
  moveOne(m1, FULL); delay(100);
  moveOne(m2, FULL); delay(100);
  moveOne(m3, FULL); delay(100);
  moveOne(m4, FULL); delay(100);
  moveOne(m5, FULL); delay(500);
  moveAll(200);
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
  moveAll(200); delay(1000);
}

void patternFour() {
  m1.moveTo(THIRD);  m2.moveTo(THREE_Q);
  m3.moveTo(FULL);   m4.moveTo(-THREE_Q); m5.moveTo(THIRD);
  runAllUntilDone(); delay(800);

  moveAll(200); delay(500);

  m1.moveTo(FULL);   m2.moveTo(THREE_Q);
  m3.moveTo(THIRD);  m4.moveTo(-THREE_Q); m5.moveTo(FULL);
  runAllUntilDone(); delay(800);

  moveAll(200); delay(1000);
}

// ═══════════════════════════════════════════════════════
// COMMUNICATION
// ═══════════════════════════════════════════════════════
void sendToMaster(const char* cmd, int val) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value   = val;
  outgoingMsg.slaveId = SLAVE_ID;
  esp_now_send(masterAddress, (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
}

void onDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  memcpy(&incomingMsg, data, sizeof(incomingMsg));

  if (!connectedToMaster) { connectedToMaster = true; return; }

  if (strcmp(incomingMsg.command, "HEARTBEAT") == 0) return;

  if      (strcmp(incomingMsg.command, "PATTERN_1") == 0) activePattern = 1;
  else if (strcmp(incomingMsg.command, "PATTERN_2") == 0) activePattern = 2;
  else if (strcmp(incomingMsg.command, "PATTERN_3") == 0) activePattern = 3;
  else if (strcmp(incomingMsg.command, "PATTERN_4") == 0) activePattern = 4;
  else if (strcmp(incomingMsg.command, "HOME")      == 0) activePattern = 99;
  else if (strcmp(incomingMsg.command, "STOP")      == 0) {
    activePattern = 0;
    m1.stop(); m2.stop(); m3.stop(); m4.stop(); m5.stop();
  }
}

// ═══════════════════════════════════════════════════════
// CHANNEL SCANNING
// ═══════════════════════════════════════════════════════
void scanForMaster() {
  int ch = 1;
  while (!connectedToMaster) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    unsigned long start = millis();
    while (millis() - start < 400) {
      if (connectedToMaster) { foundChannel = ch; return; }
      yield();
    }
    if (++ch > 13) ch = 1;
  }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.printf("\n╔══════════════════════════════════════╗\n");
  Serial.printf("║  KINETIC RAIN — SLAVE %-2d              ║\n", SLAVE_ID);
  Serial.printf("╚══════════════════════════════════════╝\n\n");

  WiFi.mode(WIFI_STA);
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  // Keep drivers DISABLED during boot to prevent radio
  // startup spikes from causing false home pin triggers
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Disabled

  for (int i = 0; i < 5; i++) {
    pinMode(homePins[i], INPUT_PULLUP);
    motors[i]->setMaxSpeed(3500);
    motors[i]->setAcceleration(2800);
    motors[i]->setMinPulseWidth(20);
  }

  delay(200); // Let pins settle electrically after WiFi init
  digitalWrite(ENABLE_PIN, LOW); // Enable drivers now

  WiFi.disconnect();
  esp_now_init();
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);

  Serial.println("🔍 Scanning for Master...");
  scanForMaster();

  esp_now_del_peer(masterAddress);
  peerInfo.channel = foundChannel;
  esp_now_add_peer(&peerInfo);

  Serial.printf("✅ Master found on channel %d\n\n", foundChannel);

  activePattern = 99; // Auto-home on startup
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  if (activePattern > 0) {
    if      (activePattern == 1)  patternOne();
    else if (activePattern == 2)  patternTwo();
    else if (activePattern == 3)  patternThree();
    else if (activePattern == 4)  patternFour();
    else if (activePattern == 99) homeAllMotors();

    if (activePattern != 0 && activePattern != 99)
      sendToMaster("PATTERN_DONE", 1);

    activePattern = 0;
  }
  yield();
}