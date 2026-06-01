#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════
// WIFI CONFIGURATION
// ═══════════════════════════════════════════════════════
const char* ssid = "Sadewna";
const char* password = "sadewna1";

// ═══════════════════════════════════════════════════════
// SLAVE COUNT
// ═══════════════════════════════════════════════════════
const int NUM_SLAVES = 10;

// ═══════════════════════════════════════════════════════
// SLAVE MAC ADDRESSES [0] = Slave 1 ... [9] = Slave 10
// ═══════════════════════════════════════════════════════
uint8_t slaveAddresses[NUM_SLAVES][6] = {
  {0x68, 0x09, 0x47, 0x44, 0x51, 0x1C},  // Slave 1
  {0x68, 0x09, 0x47, 0x60, 0x5D, 0x84},  // Slave 2
  {0x68, 0x09, 0x47, 0x51, 0xC0, 0x0C},  // Slave 3
  {0x8C, 0x94, 0xDF, 0xAA, 0x44, 0xFC},  // Slave 4
  {0x68, 0x09, 0x47, 0x51, 0xB8, 0x3C},  // Slave 5
  {0x68, 0x09, 0x47, 0x5F, 0x1C, 0x40},  // Slave 6
  {0x30, 0x76, 0xF5, 0xF7, 0xD3, 0x64},  // Slave 7
  {0xF4, 0x2D, 0xC9, 0x71, 0x52, 0xA8},  // Slave 8
  {0xB4, 0xBF, 0xE9, 0x1A, 0xDA, 0x70},  // Slave 9
  {0x68, 0x09, 0x47, 0x5F, 0x1F, 0x4C},  // Slave 10
};

// ═══════════════════════════════════════════════════════
// HOW MANY SLAVES ARE PHYSICALLY CONNECTED RIGHT NOW
// Increment this as you add hardware (max = NUM_SLAVES)
// ═══════════════════════════════════════════════════════
const int ACTIVE_SLAVES = 10;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ═══════════════════════════════════════════════════════
// COMMUNICATION STRUCTURE
// ═══════════════════════════════════════════════════════
typedef struct {
  char command[32];
  int  value;
  int  slaveId;
} Message;

Message outgoingMsg;
Message incomingMsg;

// ═══════════════════════════════════════════════════════
// SYSTEM STATE
// ═══════════════════════════════════════════════════════
String systemStatus                = "INITIALIZING";
bool   autoCycleRunning            = false;
bool   slaveReady[NUM_SLAVES]      = {};
bool   slaveDone[NUM_SLAVES]       = {};
int    currentPattern              = 0;
unsigned long lastBeacon           = 0;

// ═══════════════════════════════════════════════════════
// BROADCAST STATUS TO REACT DASHBOARD
// ═══════════════════════════════════════════════════════
void broadcastToReact() {
  StaticJsonDocument<1024> doc;
  doc["status"]          = systemStatus;
  doc["autoCycleRunning"]= autoCycleRunning;
  doc["currentPattern"]  = currentPattern;
  doc["numSlaves"]       = NUM_SLAVES;
  doc["activeSlaves"]    = ACTIVE_SLAVES;

  JsonArray arr = doc["slaves"].to<JsonArray>();
  for (int i = 0; i < NUM_SLAVES; i++) {
    JsonObject s = arr.add<JsonObject>();
    s["id"]    = i + 1;
    s["ready"] = slaveReady[i];
    s["done"]  = slaveDone[i];
  }

  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

// ═══════════════════════════════════════════════════════
// SEND COMMAND TO ALL ACTIVE SLAVES
// ═══════════════════════════════════════════════════════
void broadcastToAllSlaves(const char* cmd, int value) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value   = value;
  outgoingMsg.slaveId = 0;

  Serial.printf("\n📡 Broadcasting: %s\n", cmd);

  for (int i = 0; i < ACTIVE_SLAVES; i++) {
    esp_err_t result = esp_now_send(slaveAddresses[i],
                                    (uint8_t*)&outgoingMsg,
                                    sizeof(outgoingMsg));
    Serial.printf("  → Slave %d: %s\n", i + 1,
                  result == ESP_OK ? "✓" : "✗ FAILED");
  }
}

// ═══════════════════════════════════════════════════════
// CHECK IF ALL ACTIVE SLAVES ARE DONE / READY
// ═══════════════════════════════════════════════════════
bool allActiveSlavesDone() {
  for (int i = 0; i < ACTIVE_SLAVES; i++)
    if (!slaveDone[i]) return false;
  return true;
}

bool allActiveSlavesReady() {
  for (int i = 0; i < ACTIVE_SLAVES; i++)
    if (!slaveReady[i]) return false;
  return true;
}

// ═══════════════════════════════════════════════════════
// ESP-NOW RECEIVE CALLBACK
// ═══════════════════════════════════════════════════════
void onDataReceive(const esp_now_recv_info_t *recv_info,
                   const uint8_t *data, int len) {
  memcpy(&incomingMsg, data, sizeof(incomingMsg));
  int id = incomingMsg.slaveId;

  if (id < 1 || id > NUM_SLAVES) return;
  int idx = id - 1;

  Serial.printf("📩 Slave %d: %s\n", id, incomingMsg.command);

  if (strcmp(incomingMsg.command, "SLAVE_READY") == 0) {
    slaveReady[idx] = true;
  }
  else if (strcmp(incomingMsg.command, "HOMING_DONE") == 0) {
    slaveReady[idx] = true;
    Serial.printf("✓ Slave %d homed and ready\n", id);
  }
  else if (strcmp(incomingMsg.command, "PATTERN_DONE") == 0) {
    slaveDone[idx] = true;
    Serial.printf("✓ Slave %d pattern done (%d/%d)\n",
                  id,
                  [&]{ int c=0; for(int i=0;i<ACTIVE_SLAVES;i++) c+=slaveDone[i]; return c; }(),
                  ACTIVE_SLAVES);

    if (allActiveSlavesDone()) {
      Serial.println("✅ ALL ACTIVE SLAVES DONE");

      if (autoCycleRunning) {
        for (int i = 0; i < NUM_SLAVES; i++) slaveDone[i] = false;
        currentPattern = (currentPattern % 4) + 1;
        Serial.printf("🔄 Auto-Cycle → Pattern %d\n", currentPattern);
        delay(1000);
        char cmd[32];
        sprintf(cmd, "PATTERN_%d", currentPattern);
        broadcastToAllSlaves(cmd, 1);
        systemStatus = String("PATTERN_") + currentPattern + "_RUNNING";
      } else {
        systemStatus = "ALL_SLAVES_READY";
      }
    }
  }
  else if (strcmp(incomingMsg.command, "STOPPED") == 0) {
    systemStatus = "STOPPED";
  }

  if (allActiveSlavesReady() &&
      (systemStatus == "INITIALIZING" || systemStatus == "HOMING")) {
    systemStatus = "ALL_SLAVES_READY";
    Serial.println("✅ ALL ACTIVE SLAVES READY");
  }

  broadcastToReact();
}

// ═══════════════════════════════════════════════════════
// WEBSOCKET EVENT HANDLER
// ═══════════════════════════════════════════════════════
void onWebSocketEvent(AsyncWebSocket* server,
                      AsyncWebSocketClient* client,
                      AwsEventType type,
                      void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("🌐 Dashboard connected: %s\n",
                  client->remoteIP().toString().c_str());
    broadcastToReact();
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 &&
        info->len == len && info->opcode == WS_TEXT) {

      data[len] = 0;
      String msg = (char*)data;
      Serial.printf("🌐 Dashboard: %s\n", msg.c_str());

      if (msg.startsWith("PATTERN_")) {
        autoCycleRunning = false;
        currentPattern   = msg.substring(8).toInt();
        for (int i = 0; i < NUM_SLAVES; i++) slaveDone[i] = false;
        broadcastToAllSlaves(msg.c_str(), 1);
        systemStatus = msg + "_RUNNING";
      }
      else if (msg == "START_AUTO_CYCLE") {
        autoCycleRunning = true;
        currentPattern   = 1;
        for (int i = 0; i < NUM_SLAVES; i++) slaveDone[i] = false;
        broadcastToAllSlaves("PATTERN_1", 1);
        systemStatus = "PATTERN_1_RUNNING";
      }
      else if (msg == "HOME_ALL") {
        autoCycleRunning = false;
        for (int i = 0; i < NUM_SLAVES; i++) slaveReady[i] = false;
        broadcastToAllSlaves("HOME", 1);
        systemStatus = "HOMING";
      }
      else if (msg == "STOP_ALL") {
        autoCycleRunning = false;
        broadcastToAllSlaves("STOP", 1);
        systemStatus = "STOPPED";
      }
      else if (msg == "REQUEST_STATUS") {
        // just reply
      }

      broadcastToReact();
    }
  }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  KINETIC RAIN — MASTER CONTROLLER    ║");
  Serial.printf( "║  Active Slaves: %-2d / %-2d               ║\n",
                 ACTIVE_SLAVES, NUM_SLAVES);
  Serial.println("╚══════════════════════════════════════╝\n");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500); Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ WiFi connected — IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi failed — check credentials");
  }

  Serial.printf("Master MAC: %s\n\n", WiFi.macAddress().c_str());

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataReceive);

    for (int i = 0; i < ACTIVE_SLAVES; i++) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, slaveAddresses[i], 6);
      peer.channel = WiFi.channel();
      peer.ifidx   = WIFI_IF_STA;
      if (esp_now_add_peer(&peer) == ESP_OK)
        Serial.printf("✓ Slave %d registered\n", i + 1);
      else
        Serial.printf("✗ Slave %d registration failed\n", i + 1);
    }
  } else {
    Serial.println("❌ ESP-NOW init failed");
  }

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();

  Serial.println("\n✅ MASTER READY\n");
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  ws.cleanupClients();

  if (millis() - lastBeacon > 3000) {
    broadcastToAllSlaves("HEARTBEAT", WiFi.channel());
    lastBeacon = millis();
  }
}