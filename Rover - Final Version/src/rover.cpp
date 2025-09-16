// rover.cpp - Enhanced Mars Rover with Fixed Autonomous Navigation
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include <DHT.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ----------------- PIN DEFINITIONS -----------------
#define MOTOR_IN1 26
#define MOTOR_IN2 14
#define MOTOR_IN3 27
#define MOTOR_IN4 12
#define MOTOR_ENA 25
#define MOTOR_ENB 33
#define TRIG_PIN 4
#define ECHO_PIN 5
#define SCAN_SERVO_PIN 15
#define MOISTURE_SERVO 23
#define SERVO_PAN_PIN 22
#define SERVO_TILT_PIN 13
#define HALL_SENSOR_PIN 34
#define MQ135_PIN 32
#define LDR_PIN 36
#define DHT_PIN 2
#define DHT_TYPE DHT11

// LED Pins
#define PIN_FRONT 16
#define PIN_RIGHT 17
#define PIN_LEFT 18
#define PIN_BACK 21
#define NUM_LEDS 10

// ----------------- GLOBAL VARIABLES ----------------
const char *ssid = "10xTC-AP2";         // <- update if needed
const char *password = "10xTechClub#"; // <- update if needed

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DHT dht(DHT_PIN, DHT_TYPE);
Servo scanServo, moistureServoControl, servoPan, servoTilt;

// LED Strips
Adafruit_NeoPixel frontStrip(NUM_LEDS, PIN_FRONT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rightStrip(NUM_LEDS, PIN_RIGHT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leftStrip(NUM_LEDS, PIN_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel backStrip(NUM_LEDS, PIN_BACK, NEO_GRB + NEO_KHZ800);

#define TEAL_COLOR 0x00FFBF
#define WHITE_COLOR 0xFFFFFF
#define RED_COLOR 0xFF0000

// Control & Navigation
enum ControlMode { MANUAL, AUTONOMOUS };
enum MovementState { STOP, FORWARD, RIGHT, LEFT, BACKWARD };

// Fixed autonomous navigation states
enum SimpleAutoState { 
  AUTO_FORWARD,           // Go straight until obstacle
  AUTO_BACKUP,           // Back up for 2 seconds
  AUTO_SCAN_SETUP,       // Prepare for scanning
  AUTO_SCAN_SLIGHT_LEFT, // Scan slight left (135°)
  AUTO_SCAN_EXTREME_LEFT,// Scan extreme left (165°)
  AUTO_SCAN_SLIGHT_RIGHT,// Scan slight right (45°)
  AUTO_SCAN_EXTREME_RIGHT,// Scan extreme right (15°)
  AUTO_SCAN_ANALYZE,     // Analyze scan results and decide
  AUTO_TURN_LEFT,        // Execute left turn
  AUTO_TURN_RIGHT,       // Execute right turn
  AUTO_BACKUP_MORE       // Additional backup when both sides blocked
};

SimpleAutoState simpleAutoState = AUTO_FORWARD;

// Scan data storage
struct ScanData {
  int slightLeft = 0;     // 135°
  int extremeLeft = 0;    // 165°
  int slightRight = 0;    // 45°
  int extremeRight = 0;   // 15°
  bool scanComplete = false;
} scanData;

ControlMode currentMode = MANUAL;
MovementState currentMovement = STOP;

// Enhanced timing and state management
unsigned long lastDistanceRead = 0;
unsigned long lastMovementUpdate = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastWiFiCheck = 0;
unsigned long autoTimer = 0;
unsigned long lastValidDistance = 0;
unsigned long lastClientActivity = 0;

// Timing intervals - optimized for reliability
const int DISTANCE_INTERVAL = 100;    // 10Hz
const int MOVEMENT_INTERVAL = 150;    // ~6.6Hz
const int SENSOR_INTERVAL = 2000;     // 0.5Hz
const int WIFI_CHECK_INTERVAL = 5000; // 5s
const int CLIENT_TIMEOUT = 10000;     // 10s

// Enhanced navigation parameters
const int OBSTACLE_THRESHOLD = 40;        // Distance to start avoiding
const int EMERGENCY_THRESHOLD = 20;       // Emergency stop distance
const int SAFE_DISTANCE = 50;             // Safe following distance
const int BACKUP_DURATION = 2000;         // 2 seconds backup
const int TURN_DURATION = 1500;           // Turn duration
const int SERVO_SETTLE_TIME = 600;        // Time for servo to reach position
const int EXTRA_BACKUP_DURATION = 3000;   // Additional backup when both sides blocked
const int MIN_SAFE_DISTANCE = 60;         // Minimum distance to consider safe for turning

const int defaultSpeed = 100;
const int turnSpeed = 90;
const int slowSpeed = 80;

volatile int distance = 999;
int panAngle = 90, tiltAngle = 90, scanServoAngle = 90;

String autoStatus = "";
bool wifiConnected = false;
bool hasActiveClients = false;

struct SensorData {
  float temp = 0;
  float humidity = 0;
  int light = 0;
  int hall = 0;
  int gas = 0;
} sensors;

// Function declarations
void updateDistanceSensor();
void processMovement();
void updateOtherSensors();
void sendCriticalData();
void sendFullData();
void handleCommand(const String &msg);
void simplifiedAutonomousNavigate();
void emergencyStop();
void setLEDs(MovementState state);
void initializeSystem();
void checkWiFiConnection();
int fastDistance();
void clearAllLEDs();
void setStripColor(Adafruit_NeoPixel& strip, uint32_t color);
uint32_t adjustBrightness(uint32_t color, int brightness);
void showAllLEDs();

// Movement helper declarations
void moveForward(int spd = defaultSpeed);
void moveBackward(int spd = slowSpeed);
void turnLeft(int spd = turnSpeed);
void turnRight(int spd = turnSpeed);
void stopMotors();

void wsEvent(AsyncWebSocket *serverWS, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);

// ----------------- SETUP ---------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Enhanced Mars Rover v4.0");
  initializeSystem();
  Serial.println("🚀 Enhanced Mars Rover Ready - Fixed Autonomous Navigation!");
}

// ----------------- MAIN LOOP - ENHANCED -----------
void loop() {
  unsigned long now = millis();
  
  // PRIORITY 1: WiFi connection monitoring
  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    checkWiFiConnection();
  }
  
  // PRIORITY 2: Distance sensor (Critical for safety)
  if (now - lastDistanceRead >= DISTANCE_INTERVAL) {
    lastDistanceRead = now;
    updateDistanceSensor();
    
    // Emergency stop check
    if (distance < EMERGENCY_THRESHOLD && currentMovement != STOP) {
      emergencyStop();
    }
  }
  
  // PRIORITY 3: Movement processing and autonomous navigation
  if (now - lastMovementUpdate >= MOVEMENT_INTERVAL) {
    lastMovementUpdate = now;
    processMovement();
    
    // Send critical data more frequently when autonomous
    if (currentMode == AUTONOMOUS || hasActiveClients) {
      sendCriticalData();
    }
  }
  
  // PRIORITY 4: Sensor updates and full data transmission
  if (now - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = now;
    updateOtherSensors();
    if (hasActiveClients) {
      sendFullData();
    }
  }
  
  ws.cleanupClients();
}

// ----------------- WIFI CONNECTION MANAGEMENT -------
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("📡 WiFi connection lost! Attempting reconnection...");
      wifiConnected = false;
    }
    
    // Try to reconnect
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      attempts++;
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ WiFi reconnected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
    }
  } else {
    wifiConnected = true;
  }
  
  // Check for active clients
  hasActiveClients = (ws.count() > 0) && (millis() - lastClientActivity < CLIENT_TIMEOUT);
  
  // If no clients and in autonomous mode, continue anyway
  if (!hasActiveClients && currentMode == AUTONOMOUS) {
    Serial.println("🤖 Autonomous mode continues without UI connection");
  }
}

// ----------------- ENHANCED SENSOR FUNCTIONS -------
void updateDistanceSensor() {
  int newDistance = fastDistance();
  
  // Validate reading - filter out obviously wrong values
  if (newDistance > 0 && newDistance <= 500) {
    // Smooth out noisy readings
    if (distance == 999) distance = newDistance;
    distance = (distance * 3 + newDistance) / 4;
    lastValidDistance = millis();
  } else if (millis() - lastValidDistance > 2000) {
    // If no valid reading for 2 seconds, assume clear path
    distance = 999;
  }
}

int fastDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Longer timeout
  if (duration == 0) return 999;
  
  int calculatedDistance = duration * 0.034 / 2;
  return constrain(calculatedDistance, 0, 999);
}

// ----------------- ENHANCED MOVEMENT PROCESSING ----
void processMovement() {
  if (currentMode == AUTONOMOUS) {
    simplifiedAutonomousNavigate();
  }
  setLEDs(currentMovement);
}

// ----------------- FIXED AUTONOMOUS NAVIGATION -----
void simplifiedAutonomousNavigate() {
  unsigned long now = millis();
  
  switch (simpleAutoState) {
    
    case AUTO_FORWARD:
      // Go straight until obstacle detected
      if (distance > OBSTACLE_THRESHOLD) {
        if (currentMovement != FORWARD) {
          moveForward(defaultSpeed);
          autoStatus = "Moving forward - Clear path (" + String(distance) + "cm)";
        }
      } else {
        // Obstacle detected - start backup sequence
        stopMotors();
        moveBackward(slowSpeed);
        simpleAutoState = AUTO_BACKUP;
        autoTimer = now;
        autoStatus = "🚨 Obstacle at " + String(distance) + "cm - Backing up";
        Serial.println(autoStatus);
      }
      break;

    case AUTO_BACKUP:
      // Back up for 2 seconds using millis (non-blocking)
      if (now - autoTimer >= BACKUP_DURATION) {
        stopMotors();
        // Reset scan data
        scanData.slightLeft = 0;
        scanData.extremeLeft = 0;
        scanData.slightRight = 0;
        scanData.extremeRight = 0;
        scanData.scanComplete = false;
        
        // Start scanning sequence
        simpleAutoState = AUTO_SCAN_SETUP;
        autoTimer = now;
        autoStatus = "Backup complete - Starting scan sequence";
        Serial.println(autoStatus);
      }
      break;

    case AUTO_SCAN_SETUP:
      // Brief pause before starting scan
      if (now - autoTimer >= 300) {
        scanServo.write(135); // Position for slight left scan
        simpleAutoState = AUTO_SCAN_SLIGHT_LEFT;
        autoTimer = now;
        autoStatus = "Scanning slight left (135°)...";
      }
      break;

    case AUTO_SCAN_SLIGHT_LEFT:
      // Wait for servo to settle, then take reading
      if (now - autoTimer >= SERVO_SETTLE_TIME) {
        scanData.slightLeft = fastDistance();
        scanServo.write(165); // Position for extreme left scan
        simpleAutoState = AUTO_SCAN_EXTREME_LEFT;
        autoTimer = now;
        autoStatus = "Slight left: " + String(scanData.slightLeft) + "cm - Scanning extreme left (165°)...";
        Serial.println("Scan slight left (135°): " + String(scanData.slightLeft) + "cm");
      }
      break;

    case AUTO_SCAN_EXTREME_LEFT:
      // Wait for servo to settle, then take reading
      if (now - autoTimer >= SERVO_SETTLE_TIME) {
        scanData.extremeLeft = fastDistance();
        scanServo.write(45); // Position for slight right scan
        simpleAutoState = AUTO_SCAN_SLIGHT_RIGHT;
        autoTimer = now;
        autoStatus = "Extreme left: " + String(scanData.extremeLeft) + "cm - Scanning slight right (45°)...";
        Serial.println("Scan extreme left (165°): " + String(scanData.extremeLeft) + "cm");
      }
      break;

    case AUTO_SCAN_SLIGHT_RIGHT:
      // Wait for servo to settle, then take reading
      if (now - autoTimer >= SERVO_SETTLE_TIME) {
        scanData.slightRight = fastDistance();
        scanServo.write(15); // Position for extreme right scan
        simpleAutoState = AUTO_SCAN_EXTREME_RIGHT;
        autoTimer = now;
        autoStatus = "Slight right: " + String(scanData.slightRight) + "cm - Scanning extreme right (15°)...";
        Serial.println("Scan slight right (45°): " + String(scanData.slightRight) + "cm");
      }
      break;

    case AUTO_SCAN_EXTREME_RIGHT:
      // Wait for servo to settle, then take reading
      if (now - autoTimer >= SERVO_SETTLE_TIME) {
        scanData.extremeRight = fastDistance();
        scanServo.write(90); // Return servo to center
        simpleAutoState = AUTO_SCAN_ANALYZE;
        autoTimer = now;
        autoStatus = "Extreme right: " + String(scanData.extremeRight) + "cm - Analyzing results...";
        Serial.println("Scan extreme right (15°): " + String(scanData.extremeRight) + "cm");
        scanData.scanComplete = true;
      }
      break;

    case AUTO_SCAN_ANALYZE:
      // Analyze scan results and make decision
      if (now - autoTimer >= 500) { // Brief pause for analysis
        // Calculate average distances for left and right sides
        int leftAverage = (scanData.slightLeft + scanData.extremeLeft) / 2;
        int rightAverage = (scanData.slightRight + scanData.extremeRight) / 2;
        
        Serial.println("🔍 SCAN ANALYSIS:");
        Serial.println("Left side average: " + String(leftAverage) + "cm");
        Serial.println("Right side average: " + String(rightAverage) + "cm");
        Serial.println("Current distance ahead: " + String(distance) + "cm");
        
        // Decision logic with safety thresholds
        if (leftAverage >= MIN_SAFE_DISTANCE && rightAverage >= MIN_SAFE_DISTANCE) {
          // Both sides have space - choose the side with more space
          if (leftAverage > rightAverage) {
            turnLeft(turnSpeed);
            simpleAutoState = AUTO_TURN_LEFT;
            autoStatus = "✅ Both sides clear - Left has more space (" + String(leftAverage) + "cm vs " + String(rightAverage) + "cm) - Turning left";
          } else {
            turnRight(turnSpeed);
            simpleAutoState = AUTO_TURN_RIGHT;
            autoStatus = "✅ Both sides clear - Right has more space (" + String(rightAverage) + "cm vs " + String(leftAverage) + "cm) - Turning right";
          }
        } else if (leftAverage >= MIN_SAFE_DISTANCE) {
          // Only left side is clear
          turnLeft(turnSpeed);
          simpleAutoState = AUTO_TURN_LEFT;
          autoStatus = "⬅️ Only left side clear (" + String(leftAverage) + "cm) - Turning left";
        } else if (rightAverage >= MIN_SAFE_DISTANCE) {
          // Only right side is clear
          turnRight(turnSpeed);
          simpleAutoState = AUTO_TURN_RIGHT;
          autoStatus = "➡️ Only right side clear (" + String(rightAverage) + "cm) - Turning right";
        } else {
          // Both sides blocked - need more backup
          moveBackward(slowSpeed);
          simpleAutoState = AUTO_BACKUP_MORE;
          autoStatus = "🚫 Both sides blocked (L:" + String(leftAverage) + "cm R:" + String(rightAverage) + "cm) - Backing up more";
          Serial.println("⚠️ Both sides blocked - additional backup required");
        }
        
        autoTimer = now;
        Serial.println(autoStatus);
      }
      break;

    case AUTO_TURN_LEFT:
      // Execute left turn for specified duration
      if (now - autoTimer >= TURN_DURATION) {
        stopMotors();
        simpleAutoState = AUTO_FORWARD;
        autoStatus = "✅ Left turn complete - Resuming forward movement";
        Serial.println(autoStatus);
        delay(300); // Brief pause before resuming
      }
      break;

    case AUTO_TURN_RIGHT:
      // Execute right turn for specified duration
      if (now - autoTimer >= TURN_DURATION) {
        stopMotors();
        simpleAutoState = AUTO_FORWARD;
        autoStatus = "✅ Right turn complete - Resuming forward movement";
        Serial.println(autoStatus);
        delay(300); // Brief pause before resuming
      }
      break;

    case AUTO_BACKUP_MORE:
      // Additional backup when both sides are blocked
      if (now - autoTimer >= EXTRA_BACKUP_DURATION) {
        stopMotors();
        // After more backup, scan again
        simpleAutoState = AUTO_SCAN_SETUP;
        autoTimer = now;
        autoStatus = "Extended backup complete - Rescanning area";
        Serial.println("🔄 Extended backup complete - starting new scan");
      }
      break;
  }
}

// Enhanced emergency stop function
void emergencyStop() {
  stopMotors();
  if (currentMode == AUTONOMOUS) {
    // Force immediate backup if too close
    if (distance < EMERGENCY_THRESHOLD) {
      moveBackward(slowSpeed);
      simpleAutoState = AUTO_BACKUP;
      autoTimer = millis();
      autoStatus = "🚨 EMERGENCY STOP! Distance: " + String(distance) + "cm - Emergency backup";
      Serial.println(autoStatus);
    }
  }
}

// ----------------- DATA TRANSMISSION (Enhanced) ----
void sendCriticalData() {
  if (ws.count() == 0) return;
  
  StaticJsonDocument<512> doc;
  doc["distance"] = distance;
  doc["connected"] = wifiConnected;
  if (currentMode == AUTONOMOUS && autoStatus.length() > 0) {
    doc["autoStatus"] = autoStatus;
    doc["autoState"] = (int)simpleAutoState;
    
    // Send scan data if available
    if (scanData.scanComplete) {
      doc["slightLeft"] = scanData.slightLeft;
      doc["extremeLeft"] = scanData.extremeLeft;
      doc["slightRight"] = scanData.slightRight;
      doc["extremeRight"] = scanData.extremeRight;
    }
  }
  
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
  lastClientActivity = millis();
}

void sendFullData() {
  if (ws.count() == 0) return;
  
  StaticJsonDocument<768> doc;
  doc["distance"] = distance;
  doc["temp"] = (int)sensors.temp;
  doc["humidity"] = (int)sensors.humidity;
  doc["light"] = sensors.light;
  doc["hall"] = sensors.hall;
  doc["gas"] = sensors.gas;
  doc["pan"] = panAngle;
  doc["tilt"] = tiltAngle;
  doc["connected"] = wifiConnected;
  doc["clients"] = ws.count();
  
  if (currentMode == AUTONOMOUS) {
    doc["autoState"] = (int)simpleAutoState;
    doc["autoStatus"] = autoStatus;
    
    // Send comprehensive scan results
    if (scanData.scanComplete) {
      JsonArray scanResults = doc.createNestedArray("scanResults");
      scanResults.add(scanData.extremeLeft);   // 165°
      scanResults.add(scanData.slightLeft);    // 135°
      scanResults.add(distance);               // 90° (current)
      scanResults.add(scanData.slightRight);   // 45°
      scanResults.add(scanData.extremeRight);  // 15°
    }
  }
  
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

// ----------------- REST: other sensors ----------------
void updateOtherSensors() {
  static float tempSum = 0, humSum = 0;
  static int readCount = 0;
  
  float tempReading = dht.readTemperature();
  float humReading = dht.readHumidity();
  
  if (!isnan(tempReading) && tempReading > -40 && tempReading < 80) {
    tempSum += tempReading;
    readCount++;
  }
  if (!isnan(humReading) && humReading >= 0 && humReading <= 100) {
    humSum += humReading;
  }
  
  if (readCount > 0) {
    sensors.temp = tempSum / readCount;
    sensors.humidity = humSum / readCount;
    tempSum = humSum = 0;
    readCount = 0;
  }

  int ldrRaw = analogRead(LDR_PIN);
  sensors.light = map(constrain(ldrRaw, 0, 4095), 0, 4095, 100, 0);

  int hallRaw = analogRead(HALL_SENSOR_PIN);
  sensors.hall = map(constrain(hallRaw, 0, 4095), 0, 4095, 0, 100);
  
  int gasRaw = analogRead(MQ135_PIN);
  sensors.gas = (gasRaw < 100) ? 0 : map(constrain(gasRaw, 100, 4095), 100, 4095, 10, 1000);
}

// ----------------- LED helpers ----------------
void setLEDs(MovementState state) {
  static MovementState lastState = STOP;
  if (state == lastState) return;
  
  clearAllLEDs();
  
  switch (state) {
    case FORWARD:
      setStripColor(frontStrip, WHITE_COLOR);
      setStripColor(backStrip, RED_COLOR);
      setStripColor(leftStrip, TEAL_COLOR);
      setStripColor(rightStrip, TEAL_COLOR);
      break;
    case BACKWARD:
      setStripColor(frontStrip, RED_COLOR);
      setStripColor(backStrip, WHITE_COLOR);
      setStripColor(leftStrip, TEAL_COLOR);
      setStripColor(rightStrip, TEAL_COLOR);
      break;
    case LEFT:
      setStripColor(frontStrip, WHITE_COLOR);
      setStripColor(backStrip, RED_COLOR);
      setStripColor(leftStrip, WHITE_COLOR);
      setStripColor(rightStrip, RED_COLOR);
      break;
    case RIGHT:
      setStripColor(frontStrip, WHITE_COLOR);
      setStripColor(backStrip, RED_COLOR);
      setStripColor(leftStrip, RED_COLOR);
      setStripColor(rightStrip, WHITE_COLOR);
      break;
    case STOP:
    default:
      setStripColor(frontStrip, adjustBrightness(WHITE_COLOR, 127));
      setStripColor(backStrip, adjustBrightness(RED_COLOR, 127));
      setStripColor(leftStrip, adjustBrightness(TEAL_COLOR, 127));
      setStripColor(rightStrip, adjustBrightness(TEAL_COLOR, 127));
      break;
  }
  
  showAllLEDs();
  lastState = state;
}

void setStripColor(Adafruit_NeoPixel& strip, uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
}

void clearAllLEDs() {
  frontStrip.clear();
  rightStrip.clear();
  leftStrip.clear();
  backStrip.clear();
}

void showAllLEDs() {
  frontStrip.show();
  rightStrip.show();
  leftStrip.show();
  backStrip.show();
}

uint32_t adjustBrightness(uint32_t color, int brightness) {
  uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
  uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
  uint8_t b = (color & 0xFF) * brightness / 255;
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// ----------------- COMMAND HANDLER ----------------
void handleCommand(const String &msg) {
  if (msg.length() == 0) return;
  
  lastClientActivity = millis();
  
  if (msg.startsWith("mode:")) {
    String mode = msg.substring(5);
    mode.trim();
    
    if (mode == "manual") {
      currentMode = MANUAL;
      stopMotors();
      autoStatus = "";
      simpleAutoState = AUTO_FORWARD; // Reset to forward state
      Serial.println("Manual mode activated");
    } else if (mode == "auto") {
      currentMode = AUTONOMOUS;
      simpleAutoState = AUTO_FORWARD; // Start with forward movement
      autoStatus = "Fixed autonomous mode starting...";
      Serial.println("Fixed Autonomous mode activated");
    }
  }
  else if (currentMode == MANUAL) {
    if (msg.startsWith("move:")) {
      String direction = msg.substring(5);
      direction.trim();
      if (direction == "forward") moveForward();
      else if (direction == "backward") moveBackward();
      else if (direction == "left") turnLeft();
      else if (direction == "right") turnRight();
      else if (direction == "stop") stopMotors();
    }
    else if (msg.startsWith("scanner:")) {
      int angle = constrain(msg.substring(8).toInt(), 0, 180);
      scanServo.write(angle);
      scanServoAngle = angle;
    }
    else if (msg.startsWith("camera:")) {
      int commaIndex = msg.indexOf(',');
      if (commaIndex > 0) {
        int panCmd = constrain(msg.substring(7, commaIndex).toInt(), 0, 180);
        int tiltCmd = constrain(msg.substring(commaIndex + 1).toInt(), 0, 180);
        panAngle = 180 - panCmd;
        tiltAngle = 180 - tiltCmd;
        servoPan.write(panAngle);
        servoTilt.write(tiltAngle);
      }
    }
    else if (msg.startsWith("moisture:")) {
      int angle = constrain(msg.substring(9).toInt(), 0, 180);
      moistureServoControl.write(angle);
    }
  } else {
    // If in AUTONOMOUS, still allow emergency stop and scanner/camera updates
    if (msg == "move:stop") {
      stopMotors();
      Serial.println("Manual stop received during autonomous - stopping motors");
    } else if (msg.startsWith("scanner:")) {
      int angle = constrain(msg.substring(8).toInt(), 0, 180);
      scanServo.write(angle);
      scanServoAngle = angle;
    } else if (msg.startsWith("camera:")) {
      int commaIndex = msg.indexOf(',');
      if (commaIndex > 0) {
        int panCmd = constrain(msg.substring(7, commaIndex).toInt(), 0, 180);
        int tiltCmd = constrain(msg.substring(commaIndex + 1).toInt(), 0, 180);
        panAngle = 180 - panCmd;
        tiltAngle = 180 - tiltCmd;
        servoPan.write(panAngle);
        servoTilt.write(tiltAngle);
      }
    }
  }
}

// ----------------- WEB SOCKET HANDLER --------------
void wsEvent(AsyncWebSocket *serverWS, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    lastClientActivity = millis();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->len == len && info->opcode == WS_TEXT) {
      String msg = "";
      for (size_t i = 0; i < len; i++) msg += (char) data[i];
      if (msg == "__ping__") {
        return;
      }
      handleCommand(msg);
    }
  }
}

// ----------------- SERVER & INIT ------------------
void initializeSystem() {
  // Pins & servos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(DHT_PIN, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  analogWriteResolution(8);

  scanServo.attach(SCAN_SERVO_PIN);
  moistureServoControl.attach(MOISTURE_SERVO);
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);

  scanServo.write(90);
  moistureServoControl.write(90);
  servoPan.write(90);
  servoTilt.write(90);

  dht.begin();

  frontStrip.begin();
  rightStrip.begin();
  leftStrip.begin();
  backStrip.begin();
  frontStrip.setBrightness(80);
  rightStrip.setBrightness(80);
  leftStrip.setBrightness(80);
  backStrip.setBrightness(80);
  showAllLEDs();


  // LittleFS to serve static UI if desired
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS mounted");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    wifiConnected = true;
  } else {
    Serial.println("\nWiFi not connected at setup time.");
  }

  // Webserver: serve index.html from LittleFS if exists
  if (LittleFS.exists("/index.html")) {
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    Serial.println("Serving UI from LittleFS");
  } else {
    // otherwise minimal page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "Enhanced Rover ready. Web UI not found on device; serve your index.html separately.");
    });
  }

  // Websocket
  ws.onEvent(wsEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  Serial.println("Enhanced Server initialized with comprehensive navigation");
}

// ----------------- Movement helpers ----------------
void moveForward(int spd) {
  // Example differential drive with motor pins - adapt to your motor driver wiring
  analogWrite(MOTOR_ENA, spd);
  analogWrite(MOTOR_ENB, spd);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  currentMovement = FORWARD;
}

void moveBackward(int spd) {
  analogWrite(MOTOR_ENA, spd);
  analogWrite(MOTOR_ENB, spd);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  currentMovement = BACKWARD;
}

void turnRight(int spd) {
  // Turn right: left motor forward, right motor backward
  analogWrite(MOTOR_ENA, spd);
  analogWrite(MOTOR_ENB, spd);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  currentMovement = RIGHT;
}

void turnLeft(int spd) {
  // Turn left: left motor backward, right motor forward
  analogWrite(MOTOR_ENA, spd);
  analogWrite(MOTOR_ENB, spd);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  currentMovement = LEFT;
}

void stopMotors() {
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  currentMovement = STOP;
}
