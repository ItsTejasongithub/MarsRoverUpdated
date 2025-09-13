/***********************************************************************************************************************
 * ----------------------------- Mars Rover with Autonomous Navigation ---------------------------------- *
 ***********************************************************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include "LittleFS.h"
#include <DHT.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

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

// ----------------- GLOBAL VARIABLES ----------------
const char *ssid = "10xTC-AP2";
const char *password = "10xTechClub#";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DHT dht(DHT_PIN, DHT_TYPE);
Servo scanServo, moistureServoControl, servoPan, servoTilt;

// Control modes
enum ControlMode { MANUAL, AUTONOMOUS };
ControlMode currentMode = MANUAL;

// Autonomous navigation states
enum AutoState { AUTO_MOVING, AUTO_BACKUP, AUTO_SCANNING, AUTO_TURNING };
AutoState autoState = AUTO_MOVING;

// Movement states
enum MovementState { STOP, FORWARD, RIGHT, LEFT, BACKWARD };
MovementState currentMovement = STOP;

// Navigation variables
int panAngle = 90, tiltAngle = 90, scanServoAngle = 90, moisturePos = 90;
const int OBSTACLE_THRESHOLD = 25;  // cm
const int BACKUP_STEPS = 3;
const int SCAN_DELAY = 500;
int backupCounter = 0;
int scanStep = 0;
int leftDistance = 0, rightDistance = 0;
unsigned long autoTimer = 0;
String autoStatus = "";

// Sensor cache & timing
String cachedStatusJSON = "";
unsigned long lastStatusUpdate = 0;
unsigned long lastWiFiCheck = 0;
const int defaultSpeed = 50;
const int turnSpeed = 100;
const unsigned long wifiCheckInterval = 10000;

// ----------------- FUNCTION DECLARATIONS -----------
void updateSensors();
void sendStatus();
void handleCommand(const String &msg);
void moveForward(int speed = defaultSpeed);
void moveBackward(int speed = defaultSpeed);
void turnLeft(int speed = turnSpeed);
void turnRight(int speed = turnSpeed);
void stopMotors();
void initWiFi();
void checkWiFiConnection();
void initPins();
void initServos();
int convertMQ135ToPPM(int rawValue);
int getDistance();
void autonomousNavigate();
void setAutoStatus(const String& status);
void scanForSpace();

// ----------------- SETUP ---------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Mars Rover - Autonomous Version");
  Serial.println("======================================================");

  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Mount Failed");
    return;
  }
  Serial.println("✅ LittleFS initialized");

  initPins();
  Serial.println("✅ Pins initialized");
  
  initWiFi();
  
  // Initialize OTA (minimal setup)
  ArduinoOTA.setHostname("MarsRover");
  ArduinoOTA.setPassword("10xTechClub");
  ArduinoOTA.onStart([]() { stopMotors(); });
  ArduinoOTA.begin();
  Serial.println("✅ OTA initialized");

  initServos();
  Serial.println("✅ Servos initialized");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  dht.begin();
  Serial.println("✅ Sensors initialized");

  stopMotors();
  Serial.println("✅ Motors initialized");

  // Web server setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Not found");
  });

  // WebSocket handler
  ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
                AwsEventType type, void * arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        Serial.printf("🔗 Client #%u connected\n", client->id());
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("🔌 Client #%u disconnected\n", client->id());
        currentMode = MANUAL; // Safety: return to manual
        stopMotors();
        break;
      case WS_EVT_DATA: {
        AwsFrameInfo * info = (AwsFrameInfo*)arg;
        if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          data[len] = 0;
          String msg = String((char*)data);
          handleCommand(msg);
        }
        break;
      }
      default: break;
    }
  });
  
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("✅ Server started");
  Serial.printf("🌐 Access at: http://%s/\n", WiFi.localIP().toString().c_str());
  Serial.println("🚀 Mars Rover ready!");
}

// ----------------- MAIN LOOP -----------------------
void loop() {
  ArduinoOTA.handle();
  checkWiFiConnection();

  // Autonomous navigation logic
  if (currentMode == AUTONOMOUS) {
    autonomousNavigate();
  }

  // Update sensors and send status
  if (millis() - lastStatusUpdate > 500) {
    updateSensors();
    sendStatus();
    lastStatusUpdate = millis();
  }

  delay(1);
}

// ----------------- AUTONOMOUS NAVIGATION -----------
void autonomousNavigate() {
  int distance = getDistance();
  
  switch (autoState) {
    case AUTO_MOVING:
      if (distance > OBSTACLE_THRESHOLD) {
        if (currentMovement != FORWARD) {
          moveForward();
          setAutoStatus("Moving forward - scanning ahead");
        }
      } else {
        setAutoStatus("Obstacle detected! Backing up");
        autoState = AUTO_BACKUP;
        backupCounter = 0;
        autoTimer = millis();
      }
      break;

    case AUTO_BACKUP:
      if (backupCounter < BACKUP_STEPS) {
        if (millis() - autoTimer > 3000) { // Backup for 3000ms steps
          if (backupCounter == 0) moveBackward();
          backupCounter++;
          autoTimer = millis();
        }
      } else {
        stopMotors();
        setAutoStatus("Scanning left and right for space");
        autoState = AUTO_SCANNING;
        scanStep = 0;
        autoTimer = millis();
      }
      break;

    case AUTO_SCANNING:
      if (millis() - autoTimer > SCAN_DELAY) {
        scanForSpace();
        autoTimer = millis();
      }
      break;

    case AUTO_TURNING:
      if (millis() - autoTimer > 800) { // Turn for 800ms
        stopMotors();
        autoState = AUTO_MOVING;
        setAutoStatus("Turn complete - moving forward");
        // Reset camera to center
        servoPan.write(90);
        servoTilt.write(90);
        scanServo.write(90);
      }
      break;
  }
}

void scanForSpace() {
  switch (scanStep) {
    case 0: // Scan left
      scanServo.write(150);
      servoPan.write(150);
      scanStep++;
      break;
    case 1: // Measure left
      leftDistance = getDistance();
      scanStep++;
      break;
    case 2: // Scan right  
      scanServo.write(30);
      servoPan.write(30);
      scanStep++;
      break;
    case 3: // Measure right
      rightDistance = getDistance();
      scanStep++;
      break;
    case 4: // Decide direction
      if (leftDistance > rightDistance && leftDistance > OBSTACLE_THRESHOLD) {
        setAutoStatus("More space found towards LEFT - turning left");
        turnLeft();
        autoState = AUTO_TURNING;
        autoTimer = millis();
      } else if (rightDistance > leftDistance && rightDistance > OBSTACLE_THRESHOLD) {
        setAutoStatus("More space found towards RIGHT - turning right");
        turnRight();
        autoState = AUTO_TURNING;
        autoTimer = millis();
      } else {
        setAutoStatus("No clear path found - backing up more");
        autoState = AUTO_BACKUP;
        backupCounter = 0;
      }
      scanStep = 0;
      break;
  }
}

void setAutoStatus(const String& status) {
  autoStatus = status;
  Serial.println("🤖 " + status);
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration == 0) ? 999 : constrain(duration * 0.034 / 2, 0, 999);
}

// ----------------- INITIALIZATION FUNCTIONS --------
void initPins() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
}

void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  scanServo.attach(SCAN_SERVO_PIN, 1000, 2000);
  moistureServoControl.attach(MOISTURE_SERVO, 1000, 2000);
  servoPan.attach(SERVO_PAN_PIN, 1000, 2000);
  servoTilt.attach(SERVO_TILT_PIN, 1000, 2000);
  
  scanServo.write(scanServoAngle);
  moistureServoControl.write(0);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);
  
  delay(500);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  Serial.print("🔄 Connecting to WiFi");
  unsigned long startTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("✅ WiFi connected");
    Serial.printf("🌐 IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi failed!");
  }
}

void checkWiFiConnection() {
  if (millis() - lastWiFiCheck > wifiCheckInterval) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("❌ WiFi disconnected, reconnecting...");
      WiFi.begin(ssid, password);
    }
    lastWiFiCheck = millis();
  }
}

// ----------------- SENSOR FUNCTIONS ----------------
void updateSensors() {
  int distance = getDistance();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temperature)) temperature = 0;
  if (isnan(humidity)) humidity = 0;

  int ldrRaw = analogRead(LDR_PIN);
  int lightPercent = map(constrain(ldrRaw, 0, 4095), 0, 4095, 100, 0);

  int hallRaw = analogRead(HALL_SENSOR_PIN);
  int hallPercent = map(constrain(hallRaw, 0, 4095), 0, 4095, 0, 100);
  
  int gasRaw = analogRead(MQ135_PIN);
  int gasPPM = convertMQ135ToPPM(gasRaw);

  StaticJsonDocument<512> doc;
  doc["distance"] = distance;
  doc["temp"] = (int)temperature;
  doc["humidity"] = (int)humidity;
  doc["light"] = lightPercent;
  doc["hall"] = hallPercent;
  doc["gas"] = gasPPM;
  doc["servoAngle"] = scanServoAngle;
  doc["pan"] = panAngle;
  doc["tilt"] = tiltAngle;
  doc["moisture"] = moisturePos;
  doc["mode"] = (currentMode == MANUAL) ? "manual" : "auto";
  
  if (currentMode == AUTONOMOUS && autoStatus.length() > 0) {
    doc["autoStatus"] = autoStatus;
  }
  
  cachedStatusJSON = "";
  serializeJson(doc, cachedStatusJSON);
}

int convertMQ135ToPPM(int rawValue) {
  if (rawValue < 100) return 0;
  int ppm = map(constrain(rawValue, 100, 4095), 100, 4095, 10, 1000);
  return ppm;
}

void sendStatus() {
  if (ws.count() > 0) {
    ws.textAll(cachedStatusJSON);
  }
}

// ----------------- COMMAND HANDLER -----------------
void handleCommand(const String &msg) {
  if (msg.length() == 0) return;
  
  Serial.println("📨 Command: " + msg);

  if (msg.startsWith("mode:")) {
    String mode = msg.substring(5);
    mode.trim();
    
    if (mode == "manual") {
      currentMode = MANUAL;
      stopMotors();
      setAutoStatus("");
      Serial.println("🎮 Switched to MANUAL mode");
    } else if (mode == "auto") {
      currentMode = AUTONOMOUS;
      autoState = AUTO_MOVING;
      setAutoStatus("Autonomous mode activated - starting navigation");
      Serial.println("🤖 Switched to AUTONOMOUS mode");
    }
  }
  else if (currentMode == MANUAL) { // Only process manual commands in manual mode
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
      int angle = msg.substring(8).toInt();
      angle = constrain(angle, 0, 180);
      scanServo.write(angle);
      scanServoAngle = angle;
    }
    else if (msg.startsWith("moisture:")) {
      int pos = msg.substring(9).toInt();
      moisturePos = constrain(pos, 0, 180);
      int servoAngle = map(moisturePos, 0, 180, 20, 160);
      moistureServoControl.write(servoAngle);
    }
    else if (msg.startsWith("camera:")) {
      int commaIndex = msg.indexOf(',');
      if (commaIndex > 0 && commaIndex < msg.length() - 1) {
        int pan = constrain(msg.substring(7, commaIndex).toInt(), 0, 180);
        int tilt = constrain(msg.substring(commaIndex + 1).toInt(), 0, 180);
        
        panAngle = 180 - pan;
        tiltAngle = 180 - tilt;

        servoPan.write(panAngle);
        servoTilt.write(tiltAngle);
      }
    }
  }
}

// ----------------- MOVEMENT FUNCTIONS --------------
void moveForward(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, speed);
  analogWrite(MOTOR_ENB, speed);
  currentMovement = FORWARD;
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, speed);
  analogWrite(MOTOR_ENB, speed);
  currentMovement = BACKWARD;
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, speed);
  analogWrite(MOTOR_ENB, speed);
  currentMovement = LEFT;
}

void turnRight(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(MOTOR_ENA, speed);
  analogWrite(MOTOR_ENB, speed);
  currentMovement = RIGHT;
}

void stopMotors() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
  currentMovement = STOP;
}