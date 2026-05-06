#include "odometry.h"
#include "config.h"

// Encoders
volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile int lastEncodedLeft = 0;
volatile int lastEncodedRight = 0;

// Timing & State
long lastTicksL = 0;
long lastTicksR = 0;

// Control targets
float targetLeftVel = 0;
float targetRightVel = 0;
unsigned long lastCmdTime = 0;
bool brakeEnabled = false;

// Motor control
WheelPID* leftPID;
WheelPID* rightPID;
float lastPwmLeft = 0;
float lastPwmRight = 0;

// Measurements
float vL_meas = 0;
float vR_meas = 0;

// Pose (odometry frame — pure encoder + IMU)
float robotX = 5.0;
float robotY = 5.0;
float robotTheta = -PI/2;
float robotDistance = 0;
float encoderTheta = 0;
float gyroTheta = 0;
float fusedTheta = 0;

// Map-frame pose (computed from odom + TF)
float mapX = 5.0;
float mapY = 5.0;
float mapTheta = -PI/2;

// TF map→odom transform (accumulated scan matching corrections)
float tfDx = 0;
float tfDy = 0;
float tfDTheta = 0;

// Battery
float filteredVBatt = 12.0f;

/**
 * Apply TF transform: mapPose = odomPose ⊕ tfMapOdom
 * Call this after odometry update or after TF update.
 */
void applyTf() {
    // Proper 2D rigid-body transform: map = Rot(tfDTheta) * odom + (tfDx, tfDy)
    float cosT = cosf(tfDTheta);
    float sinT = sinf(tfDTheta);
    mapX     = cosT * robotX - sinT * robotY + tfDx;
    mapY     = sinT * robotX + cosT * robotY + tfDy;
    mapTheta = atan2f(sinf(robotTheta + tfDTheta), cosf(robotTheta + tfDTheta));
}

/**
 * Update the TF transform given a map-frame correction (e.g. from CSM).
 * Applies correction to map pose and back-solves for the required tf params.
 */
void updateTf(float dx, float dy, float dtheta, float weight) {
    // 1. Calculate target map pose
    float targetMapX = mapX + weight * dx;
    float targetMapY = mapY + weight * dy;
    float targetMapTheta = mapTheta + weight * dtheta;

    // 2. Compute new TF rotation
    tfDTheta = targetMapTheta - robotTheta;
    tfDTheta = atan2f(sinf(tfDTheta), cosf(tfDTheta)); // Normalize

    // 3. Compute new TF translation
    float cosT = cosf(tfDTheta);
    float sinT = sinf(tfDTheta);
    tfDx = targetMapX - (cosT * robotX - sinT * robotY);
    tfDy = targetMapY - (sinT * robotX + cosT * robotY);

    // 4. Update map pose with new TF
    applyTf();
}

// ============================================================
//   ENCODER ISRs
// ============================================================
void IRAM_ATTR leftISR() {
  int MSB = digitalRead(ENCODER_LEFT_A);
  int LSB = digitalRead(ENCODER_LEFT_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedLeft << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    INVERT_LEFT_ENCODER ? leftTicks-- : leftTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    INVERT_LEFT_ENCODER ? leftTicks++ : leftTicks--;
  lastEncodedLeft = encoded;
}

void IRAM_ATTR rightISR() {
  int MSB = digitalRead(ENCODER_RIGHT_A);
  int LSB = digitalRead(ENCODER_RIGHT_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedRight << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    INVERT_RIGHT_ENCODER ? rightTicks-- : rightTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    INVERT_RIGHT_ENCODER ? rightTicks++ : rightTicks--;
  lastEncodedRight = encoded;
}

void init_encoders() {
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), rightISR, CHANGE);
}

// ============================================================
//   MOTOR CONTROL
// ============================================================
void init_motors() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(MOTOR_LEFT_EN, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_RIGHT_EN, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void setMotor(int pinIN1, int pinIN2, int pwmCh, float u) {
  int pwr = (int)fabs(u);
  if (pwr > 255) pwr = 255;

  if (u > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (u < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    if (brakeEnabled) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, HIGH);
      pwr = 255;
    } else {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      pwr = 0;
    }
  }
  ledcWrite(pwmCh, pwr);
}
