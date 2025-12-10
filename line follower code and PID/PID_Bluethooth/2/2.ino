#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

// Motor pins
#define AIN1 8
#define BIN1 5
#define AIN2 9
#define BIN2 4
#define PWMA 6
#define PWMB 3
#define STBY 10

const int offsetA = 1;
const int offsetB = 1;

// Controls - REORGANIZED
#define SW_START 10      // SW1 - Start/Stop line following
#define SW_CALIBRATE 11  // SW2 - Calibration
#define LED 7

// Sensors
#define NUM_SENSORS 8
#define SENSOR_TIMEOUT 2500
unsigned int sensors1[NUM_SENSORS];
QTRSensors qtra;

// PID Parameters - Adjustable via Bluetooth
float kp = 0.5;
float ki = 0.0;
float kd = 10;

// Speed settings
int maxSpeed = 180;
int baseSpeed = 180;

// Line tracking
bool followBlackLine = false;
int centerPosition = 3500;

// PID variables
int lastError = 0;
float integral = 0.0;
float integral_limit = 3000.0;

// Turn detection - CHANGED TO VARIABLES INSTEAD OF CONST
int turnThreshold = 1000; // Error threshold for turn detection
unsigned long turnDuration = 800; // ms for 90 degree turn
int turnSpeed = 150; // Speed during turning

bool turnDetected = false;
unsigned long turnStartTime = 0;

// State management
enum State { IDLE, CALIBRATING, READY, RUNNING, TURNING };
State currentState = IDLE;

String btBuffer = "";

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ========== SETUP ==========
void setup() {
  // Pre-allocate buffer for better performance
  btBuffer.reserve(32);
  
  // Configure sensors
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);
  qtra.setTimeout(SENSOR_TIMEOUT);

  // Configure pins
  pinMode(SW_START, INPUT_PULLUP);
  pinMode(SW_CALIBRATE, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // Initialize serial
  Serial.begin(9600);
  printWelcome();
  
  currentState = IDLE;
  blinkLED(2, 200);  // Ready indication
}

// ========== MAIN LOOP ==========
void loop() {
  checkBluetooth();
  
  static bool lastSW1 = HIGH;
  static bool lastSW2 = HIGH;
  bool sw1 = digitalRead(SW_START);
  bool sw2 = digitalRead(SW_CALIBRATE);
  
  // SW2 - Calibration (pressed)
  if (sw2 == LOW && lastSW2 == HIGH) {
    delay(50);  // Debounce
    if (digitalRead(SW_CALIBRATE) == LOW) {
      handleCalibration();
    }
  }
  
  // SW1 - Start/Stop (pressed)
  if (sw1 == LOW && lastSW1 == HIGH) {
    delay(50);  // Debounce
    if (digitalRead(SW_START) == LOW) {
      handleStartStop();
    }
  }
  
  // Execute current state
  if (currentState == RUNNING) {
    runLineFollowing();
  } else if (currentState == TURNING) {
    executeTurn();
  }
  
  lastSW1 = sw1;
  lastSW2 = sw2;
}

// ========== BUTTON HANDLERS ==========
void handleCalibration() {
  if (currentState == RUNNING || currentState == TURNING) {
    Serial.println("Stop robot before calibration!");
    blinkLED(3, 100);
    return;
  }
  
  currentState = CALIBRATING;
  calibration();
  currentState = READY;
}

void handleStartStop() {
  if (currentState == IDLE) {
    Serial.println("Calibrate first (press SW2)!");
    blinkLED(3, 100);
    return;
  }
  
  if (currentState == RUNNING || currentState == TURNING) {
    // Stop
    currentState = READY;
    brake(motor1, motor2);
    integral = 0;  // Reset integral
    turnDetected = false;
    Serial.println("STOPPED");
    blinkLED(1, 500);
  } else if (currentState == READY) {
    // Start
    currentState = RUNNING;
    Serial.println("STARTING...");
    blinkLED(2, 100);
    // Ramp start for smooth acceleration
    forward(motor1, motor2, 80);
    delay(100);
  }
}

// ========== CALIBRATION ==========
void calibration() {
  Serial.println("=== CALIBRATING ===");
  Serial.println("Move robot left-right over line...");
  digitalWrite(LED, HIGH);
  
  for (int i = 0; i <= 100; i++) {
    if (i < 25 || i >= 75) {
      left(motor1, motor2, 200);
    } else {
      right(motor1, motor2, 200);
    }
    qtra.calibrate();
    delay(10);
  }
  
  brake(motor1, motor2);
  digitalWrite(LED, LOW);
  Serial.println("Calibration Complete!");
  Serial.println("Press SW1 to start");
  blinkLED(2, 200);
}

// ========== TURN DETECTION & EXECUTION ==========
bool detectTurn(int error) {
  // Detect sharp turn when error exceeds threshold
  return abs(error) > turnThreshold;
}

void executeTurn() {
  // Check if turn duration completed
  if (millis() - turnStartTime >= turnDuration) {
    // Return to line following
    currentState = RUNNING;
    turnDetected = false;
    integral = 0; // Reset integral after turn
    Serial.println("Turn completed, resuming line following");
    return;
  }
  
  // Continue turning - one motor CW, other CCW
  // For right turns (positive error), turn right
  // For left turns (negative error), turn left
  
  if (lastError > 0) {
    // Right turn - motor1 forward, motor2 backward
    motor1.drive(turnSpeed);
    motor2.drive(-turnSpeed);
  } else {
    // Left turn - motor1 backward, motor2 forward
    motor1.drive(-turnSpeed);
    motor2.drive(turnSpeed);
  }
}

// ========== LINE FOLLOWING ==========
void runLineFollowing() {
  checkBluetooth();
  
  // Read sensor position
  int position = followBlackLine ? qtra.readLineBlack(sensors1) : qtra.readLineWhite(sensors1);
  
  // Calculate error
  int error = centerPosition - position;
  
  // Check for 90-degree turn
  if (detectTurn(error) && !turnDetected) {
    turnDetected = true;
    currentState = TURNING;
    turnStartTime = millis();
    Serial.println("90-degree turn detected! Executing turn...");
    return; // Exit line following to start turn
  }
  
  // If no turn detected, continue normal PID line following
  integral += error;
  integral = constrain(integral, -integral_limit, integral_limit);
  
  int derivative = error - lastError;
  lastError = error;
  
  float correction = kp * error + ki * integral + kd * derivative;
  correction = constrain(correction, -1000.0, 1000.0);
  
  int rightSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
  int leftSpeed = constrain(baseSpeed + correction, 0, maxSpeed);
  
  motor1.drive(rightSpeed);
  motor2.drive(leftSpeed);
}

// ========== BLUETOOTH PROCESSING ==========
void checkBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else if (btBuffer.length() < 31) {  // Prevent overflow
      btBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  bool validCommand = true;
  
  // PID parameters
  if (cmd.startsWith("KP:")) {
    kp = cmd.substring(3).toFloat();
    Serial.print("P="); Serial.println(kp, 6);
  }
  else if (cmd.startsWith("KI:")) {
    ki = cmd.substring(3).toFloat();
    Serial.print("I="); Serial.println(ki, 6);
  }
  else if (cmd.startsWith("KD:")) {
    kd = cmd.substring(3).toFloat();
    Serial.print("D="); Serial.println(kd, 6);
  }
  else if (cmd.startsWith("ILIMIT:")) {
    integral_limit = cmd.substring(7).toFloat();
    Serial.print("I_Limit="); Serial.println(integral_limit, 2);
  }
  
  // Speed settings
  else if (cmd.startsWith("MAX:")) {
    maxSpeed = constrain(cmd.substring(4).toInt(), 0, 255);
    Serial.print("MaxSpeed="); Serial.println(maxSpeed);
  }
  else if (cmd.startsWith("BASE:")) {
    baseSpeed = constrain(cmd.substring(5).toInt(), 0, 255);
    Serial.print("BaseSpeed="); Serial.println(baseSpeed);
  }
  
  // Turn parameters - FIXED: Now using variables instead of const
  else if (cmd.startsWith("TURN_THRESH:")) {
    int newThreshold = cmd.substring(12).toInt();
    if (newThreshold > 0) {
      turnThreshold = newThreshold;
      Serial.print("TurnThreshold="); Serial.println(turnThreshold);
    }
  }
  else if (cmd.startsWith("TURN_DUR:")) {
    unsigned long newDuration = cmd.substring(9).toInt();
    if (newDuration > 0) {
      turnDuration = newDuration;
      Serial.print("TurnDuration="); Serial.println(turnDuration);
    }
  }
  else if (cmd.startsWith("TURN_SPEED:")) {
    int newSpeed = constrain(cmd.substring(11).toInt(), 0, 255);
    turnSpeed = newSpeed;
    Serial.print("TurnSpeed="); Serial.println(turnSpeed);
  }
  
  // Line type
  else if (cmd == "BLACK") {
    followBlackLine = true;
    Serial.println("Line=BLACK");
  }
  else if (cmd == "WHITE") {
    followBlackLine = false;
    Serial.println("Line=WHITE");
  }
  else if (cmd.startsWith("CENTER:")) {
    centerPosition = cmd.substring(7).toInt();
    Serial.print("Center="); Serial.println(centerPosition);
  }
  
  // System commands
  else if (cmd == "STATUS" || cmd == "S") {
    showStatus();
  }
  else if (cmd == "HELP" || cmd == "H") {
    showHelp();
  }
  else if (cmd == "STOP") {
    currentState = READY;
    brake(motor1, motor2);
    integral = 0;
    turnDetected = false;
    Serial.println("STOPPED");
  }
  else if (cmd == "START") {
    if (currentState == READY) {
      currentState = RUNNING;
      Serial.println("STARTED");
    } else {
      Serial.println("Not ready! Calibrate first");
    }
  }
  else if (cmd == "RESETI") {
    integral = 0;
    Serial.println("Integral=0");
  }
  else if (cmd == "RESET") {
    resetToDefaults();
  }
  else if (cmd == "SENSORS") {
    showSensors();
  }
  else if (cmd == "TEST_TURN") {
    if (currentState == READY) {
      Serial.println("Testing turn - simulating right turn");
      lastError = turnThreshold + 100; // Simulate right turn detection
      turnDetected = true;
      currentState = TURNING;
      turnStartTime = millis();
    }
  }
  else {
    validCommand = false;
    Serial.println("Unknown command. Type HELP");
  }
  
  // Visual feedback
  if (validCommand) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
  }
}

// ========== DISPLAY FUNCTIONS ==========
void printWelcome() {
  Serial.println("\n=== LINE FOLLOWER READY ===");
  Serial.println("SW1: Start/Stop");
  Serial.println("SW2: Calibrate");
  Serial.println("Commands: HELP, STATUS, KP:val, KI:val, KD:val");
  Serial.println("90-degree turn detection: ENABLED");
  Serial.println("===========================\n");
}

void showStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.print("State: ");
  switch(currentState) {
    case IDLE: Serial.println("IDLE"); break;
    case CALIBRATING: Serial.println("CALIBRATING"); break;
    case READY: Serial.println("READY"); break;
    case RUNNING: Serial.println("RUNNING"); break;
    case TURNING: Serial.println("TURNING"); break;
  }
  
  Serial.print("PID: P="); Serial.print(kp, 4);
  Serial.print(" I="); Serial.print(ki, 6);
  Serial.print(" D="); Serial.println(kd, 4);
  
  Serial.print("Speed: Base="); Serial.print(baseSpeed);
  Serial.print(" Max="); Serial.println(maxSpeed);
  
  Serial.print("Line: "); Serial.print(followBlackLine ? "BLACK" : "WHITE");
  Serial.print(" | Center="); Serial.println(centerPosition);
  
  Serial.print("Integral: "); Serial.print(integral, 2);
  Serial.print(" | Limit: "); Serial.println(integral_limit, 2);
  
  Serial.println("TURN SETTINGS:");
  Serial.print("  Threshold="); Serial.print(turnThreshold);
  Serial.print(" Duration="); Serial.print(turnDuration);
  Serial.print(" Speed="); Serial.println(turnSpeed);
  Serial.print("  TurnDetected="); Serial.println(turnDetected ? "YES" : "NO");
  Serial.println("==============\n");
}

void showHelp() {
  Serial.println("\n=== COMMANDS ===");
  Serial.println("PID:    KP:0.15  KI:0.001  KD:5.0  ILIMIT:1000");
  Serial.println("SPEED:  BASE:150  MAX:200");
  Serial.println("LINE:   BLACK | WHITE | CENTER:3500");
  Serial.println("TURN:   TURN_THRESH:1000  TURN_DUR:800  TURN_SPEED:150");
  Serial.println("        TEST_TURN");
  Serial.println("SYSTEM: STATUS (S) | HELP (H) | START | STOP");
  Serial.println("        RESETI | RESET | SENSORS");
  Serial.println("================\n");
}

void showSensors() {
  int position = followBlackLine ? qtra.readLineBlack(sensors1) : qtra.readLineWhite(sensors1);
  int error = centerPosition - position;
  
  Serial.print("Pos="); Serial.print(position);
  Serial.print(" | Error="); Serial.print(error);
  Serial.print(" | TurnDetect="); Serial.println(detectTurn(error) ? "YES" : "NO");
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors1[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }
  Serial.println();
}

void resetToDefaults() {
  kp = 0.15;
  ki = 0.0;
  kd = 0.0;
  integral_limit = 1000.0;
  maxSpeed = 150;
  baseSpeed = 150;
  followBlackLine = true;
  centerPosition = 3500;
  integral = 0;
  turnDetected = false;
  
  // Reset turn parameters to defaults
  turnThreshold = 1000;
  turnDuration = 800;
  turnSpeed = 150;
  
  Serial.println("\n=== RESET TO DEFAULTS ===");
  showStatus();
}

// ========== UTILITY FUNCTIONS ==========
void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(delayMs);
    digitalWrite(LED, LOW);
    if (i < times - 1) delay(delayMs);
  }
}