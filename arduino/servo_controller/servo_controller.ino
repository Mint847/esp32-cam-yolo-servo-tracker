#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

/*
  Servo controller for ESP32-CAM AI object tracking project

  Hardware:
  - ESP32-S3 DevKit
  - PCA9685 servo driver (Adafruit library - same as pan_tilt_control_3)
  - Pan  servo on PCA9685 channel 0
  - Tilt servo on PCA9685 channel 1
  - I2C: default pins (GPIO8 = SDA, GPIO9 = SCL on ESP32-S3)

  Commands from Python:
  - C = center servos
  - D,panDelta,tiltDelta = move pan/tilt servos
*/

// PCA9685 using Adafruit library (proven working on this hardware)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define PAN_CH  0
#define TILT_CH 1

// SERVO CENTER

int panCenter = 300;
int tiltCenter = 410;

float panPos = panCenter;
float tiltPos = tiltCenter;

float targetPan = panCenter;
float targetTilt = tiltCenter;

// CENTER ANIMATION STATE

bool centeringMode = false;

float centerStartPan = 0;
float centerStartTilt = 0;

unsigned long centerStartTime = 0;

// Higher = slower
unsigned long centerDurationMs = 1150;

int centerWriteDeadband = 1;

// SERVO LIMITS

// PAN = left/right
int panMin = 180;
int panMax = 430;

// TILT = up/down
int tiltMin = 230;
int tiltMax = 470;

int PAN_DIR = 1;
int TILT_DIR = 1;

// TRACKING SETTINGS

unsigned long servoUpdateMs = 6;
unsigned long lastServoUpdate = 0;

int panMaxTargetLead = 25;   // reduced from 45: limits overshoot when target disappears at edge
int tiltMaxTargetLead = 25;  // reduced from 40

unsigned long commandTimeoutMs = 2000;
unsigned long lastCommandTime = 0;

float arriveThreshold = 2;

// PAN tuning
float panMinStep = 0.8;
float panMaxStep = 6.2;
float panGain = 0.15;

// TILT tuning
float tiltMinStep = 0.5;
float tiltMaxStep = 4.0;
float tiltGain = 0.09;

int tinyPanDeltaIgnore = 1;
int tinyTiltDeltaIgnore = 0;

int panWriteDeadband = 2;
int tiltWriteDeadband = 2;

int lastPanPWM = -9999;
int lastTiltPWM = -9999;

// SERIAL LINE BUFFER

char serialBuffer[48];
int serialIndex = 0;

// SERVO HELPERS

void moveServo(byte ch, int pwmVal) {
  pwm.setPWM(ch, 0, pwmVal);
}

void moveServoIfChanged(byte ch, int pwmVal, int &lastPWM, int deadband) {
  if (lastPWM == -9999 || abs(pwmVal - lastPWM) >= deadband) {
    moveServo(ch, pwmVal);
    lastPWM = pwmVal;
  }
}

// HELPER FUNCTIONS

float absFloat(float value) {
  if (value < 0) return -value;
  return value;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float easeInOutCos(float t) {
  t = clampFloat(t, 0.0, 1.0);
  return 0.5 - 0.5 * cos(t * 3.14159265);
}

float adaptiveStep(float diff, float minStep, float maxStep, float gain) {
  float absDiff = absFloat(diff);
  if (absDiff <= arriveThreshold) return absDiff;
  float step = minStep + absDiff * gain;
  if (step > maxStep) step = maxStep;
  if (step < minStep) step = minStep;
  return step;
}

void freezeTargetsToCurrent() {
  centeringMode = false;
  targetPan = panPos;
  targetTilt = tiltPos;
}

void centerServos() {
  centerStartPan = panPos;
  centerStartTilt = tiltPos;
  centerStartTime = millis();
  targetPan = panCenter;
  targetTilt = tiltCenter;
  centeringMode = true;
}

void applyTargetLeadLimit() {
  targetPan = clampFloat(targetPan, panPos - panMaxTargetLead, panPos + panMaxTargetLead);
  targetTilt = clampFloat(targetTilt, tiltPos - tiltMaxTargetLead, tiltPos + tiltMaxTargetLead);
  targetPan = clampFloat(targetPan, panMin, panMax);
  targetTilt = clampFloat(targetTilt, tiltMin, tiltMax);
}

// SERVO UPDATE

void updateSmoothServos() {
  unsigned long now = millis();

  if (now - lastServoUpdate < servoUpdateMs) return;
  lastServoUpdate = now;

  if (centeringMode) {
    float t = (now - centerStartTime) / (float)centerDurationMs;

    if (t >= 1.0) {
      panPos = panCenter;
      tiltPos = tiltCenter;
      targetPan = panCenter;
      targetTilt = tiltCenter;
      centeringMode = false;
    } else {
      float e = easeInOutCos(t);
      panPos = centerStartPan + (panCenter - centerStartPan) * e;
      tiltPos = centerStartTilt + (tiltCenter - centerStartTilt) * e;
    }

    panPos  = clampFloat(panPos,  panMin,  panMax);
    tiltPos = clampFloat(tiltPos, tiltMin, tiltMax);

    moveServoIfChanged(PAN_CH,  (int)round(panPos),  lastPanPWM,  centerWriteDeadband);
    moveServoIfChanged(TILT_CH, (int)round(tiltPos), lastTiltPWM, centerWriteDeadband);
    return;
  }

  if (now - lastCommandTime > commandTimeoutMs) {
    freezeTargetsToCurrent();
  }

  applyTargetLeadLimit();

  float panDiff  = targetPan  - panPos;
  float tiltDiff = targetTilt - tiltPos;

  // PAN
  if (absFloat(panDiff) <= arriveThreshold) {
    panPos = targetPan;
  } else {
    float step = adaptiveStep(panDiff, panMinStep, panMaxStep, panGain);
    if (panDiff > 0) { panPos += step; if (panPos > targetPan) panPos = targetPan; }
    else             { panPos -= step; if (panPos < targetPan) panPos = targetPan; }
  }

  // TILT
  if (absFloat(tiltDiff) <= arriveThreshold) {
    tiltPos = targetTilt;
  } else {
    float step = adaptiveStep(tiltDiff, tiltMinStep, tiltMaxStep, tiltGain);
    if (tiltDiff > 0) { tiltPos += step; if (tiltPos > targetTilt) tiltPos = targetTilt; }
    else              { tiltPos -= step; if (tiltPos < targetTilt) tiltPos = targetTilt; }
  }

  panPos  = clampFloat(panPos,  panMin,  panMax);
  tiltPos = clampFloat(tiltPos, tiltMin, tiltMax);

  moveServoIfChanged(PAN_CH,  (int)round(panPos),  lastPanPWM,  panWriteDeadband);
  moveServoIfChanged(TILT_CH, (int)round(tiltPos), lastTiltPWM, tiltWriteDeadband);
}

// COMMAND HANDLING

void handleCommand(char *cmd) {
  if (cmd[0] == 'C' || cmd[0] == 'c') {
    centerServos();
    lastCommandTime = millis();
    Serial.println("CENTER");
    return;
  }

  int panDelta = 0;
  int tiltDelta = 0;

  if (sscanf(cmd, "D,%d,%d", &panDelta, &tiltDelta) == 2 ||
      sscanf(cmd, "d,%d,%d", &panDelta, &tiltDelta) == 2) {

    Serial.print("GOT D,"); Serial.print(panDelta); Serial.print(","); Serial.println(tiltDelta);

    lastCommandTime = millis();
    centeringMode = false;

    if (panDelta == 0 && tiltDelta == 0) { freezeTargetsToCurrent(); return; }

    if (abs(panDelta)  <= tinyPanDeltaIgnore)  panDelta  = 0;
    if (abs(tiltDelta) <= tinyTiltDeltaIgnore) tiltDelta = 0;
    if (panDelta == 0 && tiltDelta == 0) return;

    panDelta  *= PAN_DIR;
    tiltDelta *= TILT_DIR;

    targetPan  += panDelta;
    targetTilt += tiltDelta;

    targetPan  = clampFloat(targetPan,  panMin,  panMax);
    targetTilt = clampFloat(targetTilt, tiltMin, tiltMax);

    applyTargetLeadLimit();

    Serial.print("TGT pan="); Serial.print(targetPan); Serial.print(" tilt="); Serial.println(targetTilt);
  }
}

void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleCommand(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      } else {
        serialIndex = 0;
      }
    }
  }
}

// SETUP / LOOP

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  // Init PCA9685 using Adafruit library (same as pan_tilt_control_3 - proven working)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // match working sketch
  pwm.setPWMFreq(50);                    // 50 Hz for servos
  delay(10);

  // Move to center position on boot
  moveServo(PAN_CH,  panCenter);
  moveServo(TILT_CH, tiltCenter);

  lastPanPWM  = panCenter;
  lastTiltPWM = tiltCenter;

  lastCommandTime = millis();

  Serial.println("READY");
}

void loop() {
  readSerialNonBlocking();
  updateSmoothServos();
}