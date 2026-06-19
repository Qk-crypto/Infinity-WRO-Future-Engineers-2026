/*
  Infinity WRO Future Engineers 2026
  Obstacle Challenge - Arduino Uno

  Hardware:
    Start button D7 -> GND
    Servo D9
    Encoder A D2, Encoder B D3
    BTS7960:
      RPWM D5
      LPWM D6
      REN  D4
      LEN  D8 
    OpenMV P4 TX -> Arduino D10 RX
    Common GND

  Strategy adapted from Nerdvana's:
    heading PID -> follow cube -> pass cube -> recover -> heading PID

  This version completes 3 laps and stops after 12 corner turns.
  Parking is intentionally not enabled yet.
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// OBJECTS
// ============================================================

Servo steering;
SoftwareSerial openmvSerial(10, 11);  // RX, TX (D11 is not connected)

// ============================================================
// PINS
// ============================================================

const int START_BUTTON_PIN = 7;
const int SERVO_PIN = 9;

const int RPWM_PIN = 5;
const int LPWM_PIN = 6;
const int REN_PIN  = 4;
const int LEN_PIN  = 8;

const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

// ============================================================
// MOTOR / SERVO CALIBRATION
// ============================================================

const bool MOTOR_REVERSED = true;

const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 122;
const int SERVO_LEFT = 58;

const int PID_SERVO_MIN = 68;
const int PID_SERVO_MAX = 112;

// ============================================================
// SPEEDS
// ============================================================

const int START_SPEED = 52;
const int CRUISE_SPEED = 68;
const int CAMERA_FOLLOW_SPEED = 60;

const int LINE_OFFSET_SPEED = 55;
const int CORNER_TURN_SPEED = 50;

const int PASS_SPEED = 52;
const int PASS_SETTLE_SPEED = 56;

// ============================================================
// HEADING CONTROL
// ============================================================

const float HEADING_KP = 1.20f;
const float HEADING_KD = 0.65f;

const float YAW_DEADBAND = 0.50f;
const int MAX_HEADING_CORRECTION = 18;

const int MAX_CAMERA_CORRECTION = 16;

// ============================================================
// CORNER SETTINGS
// ============================================================

const long LINE_OFFSET_TICKS = 300;
const float CORNER_STOP_ANGLE = 87.5f;

const unsigned long CORNER_RECENTER_MS = 260;
const unsigned long LINE_EVENT_GUARD_MS = 700;

const int TOTAL_TURNS = 12;

// ============================================================
// CUBE PASS SETTINGS
//
// Red  -> pass on the RIGHT  -> +1
// Green-> pass on the LEFT   -> -1
// ============================================================

const float PASS_OUT_ANGLE = 24.0f;

const int PASS_OUT_RIGHT_SERVO = 112;
const int PASS_OUT_LEFT_SERVO = 68;

const long PASS_ALONG_TICKS = 220;
const long PASS_SETTLE_TICKS = 170;

const float PASS_RECOVER_TOLERANCE = 3.0f;
const unsigned long PASS_RECOVER_STABLE_MS = 130;
const unsigned long CUBE_EVENT_GUARD_MS = 850;

// ============================================================
// START ACCELERATION
// ============================================================

const unsigned long ACCEL_TIME_MS = 550;

// ============================================================
// STATES
// ============================================================

enum RobotState {
  WAIT_START,
  DRIVE,
  LINE_OFFSET,
  CORNER_TURN,
  CORNER_RECENTER,
  PASS_OUT,
  PASS_ALONG,
  PASS_RECOVER,
  PASS_SETTLE,
  FINISHED
};

RobotState state = WAIT_START;
bool robotRunning = false;

// ============================================================
// CAMERA DATA
// ============================================================

char cameraDirection = 'U';   // C, A, U
int turnSign = 0;             // +1 clockwise, -1 anticlockwise

int cameraLineEvent = 0;
int lastHandledLineEvent = 0;
char cameraLineColor = 'N';

int cameraCubeEvent = 0;
int lastHandledCubeEvent = 0;
char cameraCubeEventColor = 'N';

char cameraCubeColor = 'N';
int cameraSteer = 0;          // -100 .. +100
int cameraCubeHeight = 0;
int cameraCubeX = 0;

bool pendingLineEvent = false;

// ============================================================
// YAW
// ============================================================

float filteredYaw = 0.0f;
float targetYaw = 0.0f;
float turnStartYaw = 0.0f;

float previousHeadingError = 0.0f;

bool yawReceived = false;

unsigned long lastYawTime = 0;
unsigned long lastControlTime = 0;

// ============================================================
// TURN / PASS DATA
// ============================================================

int turnCount = 0;

char passColor = 'N';
int passSign = 0;             // red +1/right, green -1/left
float passBaseYaw = 0.0f;
float passStartYaw = 0.0f;
float passSideYaw = 0.0f;

unsigned long recoverStableStarted = 0;

// ============================================================
// ENCODER
// ============================================================

volatile long encoderTicks = 0;
long stateStartTicks = 0;

// ============================================================
// TIMERS
// ============================================================

unsigned long stateStartTime = 0;
unsigned long driveStartTime = 0;
unsigned long lastLineActionTime = 0;
unsigned long lastCubeActionTime = 0;
unsigned long lastPrintTime = 0;

// ============================================================
// UART BUFFER
// ============================================================

char uartBuffer[128];
uint8_t uartIndex = 0;

// ============================================================
// BUTTON
// ============================================================

bool previousButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long buttonDebounceStarted = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 60;

// ============================================================
// ANGLE HELPERS
// ============================================================

float wrap180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}

float absoluteFloat(float value) {
  return value < 0.0f ? -value : value;
}

// ============================================================
// ENCODER
// ============================================================

void encoderISR() {
  int a = digitalRead(ENC_A_PIN);
  int b = digitalRead(ENC_B_PIN);

  if (a == b) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

long readEncoderTicks() {
  noInterrupts();
  long value = encoderTicks;
  interrupts();
  return value;
}

long stateTravelledTicks() {
  long difference = readEncoderTicks() - stateStartTicks;

  if (difference < 0) {
    difference = -difference;
  }

  return difference;
}

void startEncoderState() {
  stateStartTicks = readEncoderTicks();
}

// ============================================================
// BUTTON
// ============================================================

bool buttonPressed() {
  bool reading = digitalRead(START_BUTTON_PIN);

  if (reading != previousButtonReading) {
    previousButtonReading = reading;
    buttonDebounceStarted = millis();
  }

  if (millis() - buttonDebounceStarted >= BUTTON_DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        return true;
      }
    }
  }

  return false;
}

// ============================================================
// MOTOR
// ============================================================

void motorStop() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void motorForward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);

  if (MOTOR_REVERSED) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwm);
  } else {
    analogWrite(RPWM_PIN, pwm);
    analogWrite(LPWM_PIN, 0);
  }
}

// ============================================================
// MESSAGE PARSING
// ============================================================

int parseIntegerField(
  const char* text,
  const char* key,
  int defaultValue
) {
  const char* position = strstr(text, key);

  if (position == NULL) {
    return defaultValue;
  }

  position += strlen(key);
  return atoi(position);
}

char parseCharField(
  const char* text,
  const char* key,
  char defaultValue
) {
  const char* position = strstr(text, key);

  if (position == NULL) {
    return defaultValue;
  }

  position += strlen(key);
  return position[0];
}

void processOpenMVMessage(const char* message) {
  if (strncmp(message, "Y:", 2) != 0) {
    return;
  }

  float receivedYaw = atof(message + 2);

  if (!yawReceived) {
    filteredYaw = receivedYaw;
    yawReceived = true;
  } else {
    float difference = wrap180(receivedYaw - filteredYaw);

    filteredYaw = wrap180(
      filteredYaw + difference * 0.35f
    );
  }

  lastYawTime = millis();

  cameraDirection = parseCharField(
    message,
    ",D:",
    cameraDirection
  );

  cameraLineEvent = parseIntegerField(
    message,
    ",LE:",
    cameraLineEvent
  );

  cameraLineColor = parseCharField(
    message,
    ",LC:",
    cameraLineColor
  );

  cameraCubeEvent = parseIntegerField(
    message,
    ",CE:",
    cameraCubeEvent
  );

  cameraCubeEventColor = parseCharField(
    message,
    ",EC:",
    cameraCubeEventColor
  );

  cameraCubeColor = parseCharField(
    message,
    ",C:",
    cameraCubeColor
  );

  cameraSteer = parseIntegerField(
    message,
    ",S:",
    cameraSteer
  );

  cameraCubeHeight = parseIntegerField(
    message,
    ",H:",
    cameraCubeHeight
  );

  cameraCubeX = parseIntegerField(
    message,
    ",X:",
    cameraCubeX
  );

  if (turnSign == 0) {
    if (cameraDirection == 'C') {
      turnSign = 1;
      Serial.println(F("DIRECTION: CLOCKWISE"));
    } else if (cameraDirection == 'A') {
      turnSign = -1;
      Serial.println(F("DIRECTION: ANTICLOCKWISE"));
    }
  }
}

void readOpenMV() {
  while (openmvSerial.available()) {
    char character = openmvSerial.read();

    if (character == '\n') {
      uartBuffer[uartIndex] = '\0';

      if (uartIndex > 0) {
        processOpenMVMessage(uartBuffer);
      }

      uartIndex = 0;
    } else if (character != '\r') {
      if (uartIndex < sizeof(uartBuffer) - 1) {
        uartBuffer[uartIndex++] = character;
      } else {
        uartIndex = 0;
      }
    }
  }
}

// ============================================================
// SPEED
// ============================================================

int currentDriveSpeed() {
  unsigned long elapsed = millis() - driveStartTime;

  if (elapsed >= ACCEL_TIME_MS) {
    return CRUISE_SPEED;
  }

  return map(
    elapsed,
    0,
    ACCEL_TIME_MS,
    START_SPEED,
    CRUISE_SPEED
  );
}

// ============================================================
// HEADING CONTROL
// ============================================================

void updateHeadingTo(
  float desiredYaw,
  int motorSpeed,
  int additionalServoCorrection,
  int maxHeadingCorrection
) {
  if (millis() - lastControlTime < 20) {
    return;
  }

  lastControlTime = millis();

  float error = wrap180(desiredYaw - filteredYaw);

  if (
    error > -YAW_DEADBAND &&
    error < YAW_DEADBAND
  ) {
    error = 0.0f;
  }

  float derivative = error - previousHeadingError;
  previousHeadingError = error;

  float control =
    HEADING_KP * error +
    HEADING_KD * derivative;

  int headingCorrection = (int)control;

  headingCorrection = constrain(
    headingCorrection,
    -maxHeadingCorrection,
    maxHeadingCorrection
  );

  int servoAngle =
    SERVO_CENTER +
    headingCorrection +
    additionalServoCorrection;

  servoAngle = constrain(
    servoAngle,
    PID_SERVO_MIN,
    PID_SERVO_MAX
  );

  steering.write(servoAngle);
  motorForward(motorSpeed);

  if (millis() - lastPrintTime >= 120) {
    lastPrintTime = millis();

    Serial.print(F("STATE="));
    Serial.print(state);

    Serial.print(F(" TARGET="));
    Serial.print(desiredYaw, 2);

    Serial.print(F(" YAW="));
    Serial.print(filteredYaw, 2);

    Serial.print(F(" ERR="));
    Serial.print(error, 2);

    Serial.print(F(" SERVO="));
    Serial.print(servoAngle);

    Serial.print(F(" CUBE="));
    Serial.print(cameraCubeColor);

    Serial.print(F(" CAM="));
    Serial.print(cameraSteer);

    Serial.print(F(" LE="));
    Serial.print(cameraLineEvent);

    Serial.print(F(" CE="));
    Serial.println(cameraCubeEvent);
  }
}

// ============================================================
// STATE ENTER FUNCTIONS
// ============================================================

void enterDrive() {
  state = DRIVE;
  driveStartTime = millis();
  previousHeadingError = 0.0f;

  lastHandledLineEvent = cameraLineEvent;
  lastHandledCubeEvent = cameraCubeEvent;

  steering.write(SERVO_CENTER);
  motorForward(START_SPEED);

  Serial.println(F("-------------------------"));
  Serial.print(F("DRIVE target="));
  Serial.print(targetYaw, 2);
  Serial.print(F(" turns="));
  Serial.print(turnCount);
  Serial.print('/');
  Serial.println(TOTAL_TURNS);
}

void enterLineOffset() {
  state = LINE_OFFSET;
  startEncoderState();
  previousHeadingError = 0.0f;
  pendingLineEvent = false;

  Serial.print(F("LINE OFFSET event="));
  Serial.println(cameraLineEvent);
}

void enterCornerTurn() {
  if (turnSign == 0) {
    Serial.println(F("ERROR: UNKNOWN DIRECTION"));
    motorStop();
    robotRunning = false;
    state = WAIT_START;
    return;
  }

  state = CORNER_TURN;
  turnStartYaw = filteredYaw;

  targetYaw = wrap180(
    targetYaw + turnSign * 90.0f
  );

  steering.write(
    turnSign > 0
    ? SERVO_RIGHT
    : SERVO_LEFT
  );

  motorForward(CORNER_TURN_SPEED);

  Serial.print(F("CORNER TURN sign="));
  Serial.print(turnSign);
  Serial.print(F(" target="));
  Serial.println(targetYaw, 2);
}

void enterCornerRecenter() {
  state = CORNER_RECENTER;
  stateStartTime = millis();

  motorStop();
  steering.write(SERVO_CENTER);

  previousHeadingError = 0.0f;

  lastHandledLineEvent = cameraLineEvent;
  lastHandledCubeEvent = cameraCubeEvent;
  lastLineActionTime = millis();

  Serial.print(F("CORNER COMPLETE: "));
  Serial.println(turnCount);
}

void startPassCube(char color) {
  passColor = color;

  // Official rule:
  // red -> pass right, green -> pass left.
  passSign = (color == 'R') ? 1 : -1;

  passBaseYaw = targetYaw;
  passStartYaw = filteredYaw;
  passSideYaw = wrap180(
    passBaseYaw + passSign * PASS_OUT_ANGLE
  );

  state = PASS_OUT;
  previousHeadingError = 0.0f;
  lastCubeActionTime = millis();

  steering.write(
    passSign > 0
    ? PASS_OUT_RIGHT_SERVO
    : PASS_OUT_LEFT_SERVO
  );

  motorForward(PASS_SPEED);

  Serial.print(F("PASS CUBE "));
  Serial.print(passColor);
  Serial.print(F(" sign="));
  Serial.println(passSign);
}

void enterPassAlong() {
  state = PASS_ALONG;
  startEncoderState();
  previousHeadingError = 0.0f;

  Serial.println(F("PASS ALONG"));
}

void enterPassRecover() {
  state = PASS_RECOVER;
  recoverStableStarted = 0;
  previousHeadingError = 0.0f;

  Serial.println(F("PASS RECOVER"));
}

void enterPassSettle() {
  state = PASS_SETTLE;
  startEncoderState();
  previousHeadingError = 0.0f;

  lastHandledCubeEvent = cameraCubeEvent;

  Serial.println(F("PASS SETTLE"));
}

void finishRobot() {
  state = FINISHED;
  robotRunning = false;

  motorStop();
  steering.write(SERVO_CENTER);

  Serial.println(F("========================="));
  Serial.println(F("FINISHED: 3 LAPS"));
  Serial.println(F("PARKING NOT ENABLED"));
  Serial.println(F("========================="));
}

// ============================================================
// START / STOP
// ============================================================

void startRobot() {
  if (!yawReceived) {
    Serial.println(F("ERROR: NO BNO08X YAW"));
    Serial.println(F("CHECK OPENMV P4 -> UNO D10"));
    return;
  }

  robotRunning = true;
  turnCount = 0;
  turnSign = 0;
  pendingLineEvent = false;

  noInterrupts();
  encoderTicks = 0;
  interrupts();

  targetYaw = filteredYaw;
  previousHeadingError = 0.0f;

  lastHandledLineEvent = cameraLineEvent;
  lastHandledCubeEvent = cameraCubeEvent;

  Serial.println(F("========================="));
  Serial.println(F("OBSTACLE RUN START"));
  Serial.print(F("INITIAL YAW="));
  Serial.println(targetYaw, 2);
  Serial.println(F("========================="));

  enterDrive();
}

void stopRobot() {
  robotRunning = false;
  state = WAIT_START;

  motorStop();
  steering.write(SERVO_CENTER);

  Serial.println(F("MANUAL STOP"));
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  openmvSerial.begin(38400);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);

  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A_PIN),
    encoderISR,
    CHANGE
  );

  steering.attach(SERVO_PIN);
  steering.write(SERVO_CENTER);

  motorStop();

  delay(500);

  Serial.println(F("========================="));
  Serial.println(F("INFINITY OBSTACLE V1"));
  Serial.println(F("D7 = START / STOP"));
  Serial.println(F("WAITING FOR OPENMV YAW"));
  Serial.println(F("========================="));
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  readOpenMV();

  if (buttonPressed()) {
    if (robotRunning) {
      stopRobot();
    } else {
      startRobot();
    }
  }

  if (!robotRunning) {
    motorStop();
    steering.write(SERVO_CENTER);

    static unsigned long readyPrintTime = 0;

    if (
      yawReceived &&
      millis() - readyPrintTime >= 500
    ) {
      readyPrintTime = millis();

      Serial.print(F("READY YAW="));
      Serial.print(filteredYaw, 2);

      Serial.print(F(" DIR="));
      Serial.print(cameraDirection);

      Serial.print(F(" CUBE="));
      Serial.print(cameraCubeColor);

      Serial.print(F(" H="));
      Serial.print(cameraCubeHeight);

      Serial.print(F(" LE="));
      Serial.print(cameraLineEvent);

      Serial.print(F(" CE="));
      Serial.println(cameraCubeEvent);
    }

    return;
  }

  if (millis() - lastYawTime > 400) {
    Serial.println(F("IMU DATA LOST - STOP"));
    stopRobot();
    return;
  }

  // Save a line event seen while passing a cube.
  if (cameraLineEvent > lastHandledLineEvent) {
    if (
      state == PASS_OUT ||
      state == PASS_ALONG ||
      state == PASS_RECOVER ||
      state == PASS_SETTLE
    ) {
      pendingLineEvent = true;
    }
  }

  // ==========================================================
  // DRIVE
  // ==========================================================

  if (state == DRIVE) {
    int speedPWM = currentDriveSpeed();

    int cameraCorrection = 0;

    if (
      cameraCubeColor == 'R' ||
      cameraCubeColor == 'G'
    ) {
      cameraCorrection = map(
        constrain(cameraSteer, -100, 100),
        -100,
        100,
        -MAX_CAMERA_CORRECTION,
        MAX_CAMERA_CORRECTION
      );

      speedPWM = CAMERA_FOLLOW_SPEED;
    }

    updateHeadingTo(
      targetYaw,
      speedPWM,
      cameraCorrection,
      MAX_HEADING_CORRECTION
    );

    bool cubeGuardFinished =
      millis() - lastCubeActionTime >=
      CUBE_EVENT_GUARD_MS;

    bool lineGuardFinished =
      millis() - lastLineActionTime >=
      LINE_EVENT_GUARD_MS;

    if (
      cubeGuardFinished &&
      cameraCubeEvent > lastHandledCubeEvent
    ) {
      lastHandledCubeEvent = cameraCubeEvent;
      startPassCube(cameraCubeEventColor);
    } else if (
      lineGuardFinished &&
      turnSign != 0 &&
      cameraLineEvent > lastHandledLineEvent
    ) {
      lastHandledLineEvent = cameraLineEvent;
      enterLineOffset();
    }
  }

  // ==========================================================
  // LINE OFFSET
  // ==========================================================

  else if (state == LINE_OFFSET) {
    updateHeadingTo(
      targetYaw,
      LINE_OFFSET_SPEED,
      0,
      MAX_HEADING_CORRECTION
    );

    if (stateTravelledTicks() >= LINE_OFFSET_TICKS) {
      enterCornerTurn();
    }
  }

  // ==========================================================
  // CORNER TURN
  // ==========================================================

  else if (state == CORNER_TURN) {
    steering.write(
      turnSign > 0
      ? SERVO_RIGHT
      : SERVO_LEFT
    );

    motorForward(CORNER_TURN_SPEED);

    float turnedAngle =
      wrap180(filteredYaw - turnStartYaw);

    if (turnSign < 0) {
      turnedAngle = -turnedAngle;
    }

    if (turnedAngle < 0.0f) {
      turnedAngle = 0.0f;
    }

    if (turnedAngle >= CORNER_STOP_ANGLE) {
      turnCount++;

      if (turnCount >= TOTAL_TURNS) {
        finishRobot();
      } else {
        enterCornerRecenter();
      }
    }
  }

  // ==========================================================
  // CORNER RECENTER
  // ==========================================================

  else if (state == CORNER_RECENTER) {
    motorStop();
    steering.write(SERVO_CENTER);

    lastHandledLineEvent = cameraLineEvent;
    lastHandledCubeEvent = cameraCubeEvent;

    if (
      millis() - stateStartTime >=
      CORNER_RECENTER_MS
    ) {
      enterDrive();
    }
  }

  // ==========================================================
  // PASS OUT
  // ==========================================================

  else if (state == PASS_OUT) {
    steering.write(
      passSign > 0
      ? PASS_OUT_RIGHT_SERVO
      : PASS_OUT_LEFT_SERVO
    );

    motorForward(PASS_SPEED);

    float turnedOut =
      wrap180(filteredYaw - passStartYaw);

    if (passSign < 0) {
      turnedOut = -turnedOut;
    }

    if (turnedOut < 0.0f) {
      turnedOut = 0.0f;
    }

    if (turnedOut >= PASS_OUT_ANGLE) {
      enterPassAlong();
    }
  }

  // ==========================================================
  // PASS ALONG
  // ==========================================================

  else if (state == PASS_ALONG) {
    updateHeadingTo(
      passSideYaw,
      PASS_SPEED,
      0,
      22
    );

    if (stateTravelledTicks() >= PASS_ALONG_TICKS) {
      enterPassRecover();
    }
  }

  // ==========================================================
  // PASS RECOVER
  // ==========================================================

  else if (state == PASS_RECOVER) {
    updateHeadingTo(
      passBaseYaw,
      PASS_SPEED,
      0,
      24
    );

    float recoverError =
      absoluteFloat(
        wrap180(passBaseYaw - filteredYaw)
      );

    if (recoverError <= PASS_RECOVER_TOLERANCE) {
      if (recoverStableStarted == 0) {
        recoverStableStarted = millis();
      }

      if (
        millis() - recoverStableStarted >=
        PASS_RECOVER_STABLE_MS
      ) {
        enterPassSettle();
      }
    } else {
      recoverStableStarted = 0;
    }
  }

  // ==========================================================
  // PASS SETTLE
  // ==========================================================

  else if (state == PASS_SETTLE) {
    updateHeadingTo(
      targetYaw,
      PASS_SETTLE_SPEED,
      0,
      MAX_HEADING_CORRECTION
    );

    if (stateTravelledTicks() >= PASS_SETTLE_TICKS) {
      lastCubeActionTime = millis();
      lastHandledCubeEvent = cameraCubeEvent;

      if (
        pendingLineEvent &&
        turnSign != 0
      ) {
        lastHandledLineEvent = cameraLineEvent;
        pendingLineEvent = false;
        enterLineOffset();
      } else {
        enterDrive();
      }
    }
  }
}
