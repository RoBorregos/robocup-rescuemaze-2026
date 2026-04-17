// ============================================================
// motors.cpp — versión corregida
// Cambios aplicados (busca "FIX" para encontrarlos todos):
//   FIX 2 — eliminado delay(20) de PID_speed, lazo más fino
//   FIX 3 — Serial.println comentados en PID_Wheel (activa con DEBUG_SERIAL)
//   FIX 4 — ticsSpeed inicializado a 0.0f en getTicsSpeed
//   FIX 5 — timeout en ahead() y moveDistance() con millis()
//   FIX 6 — variable i inicializada a 0 en writeServo()
// ============================================================

#include "motors.h"
#include "Encoder.h"
#include "Pins_ID.h"
#include <WiFi.h>

// Descomenta para depurar; déjalo comentado en carrera real
// #define DEBUG_SERIAL

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

motors::motors() {}

void motors::setupMotors() {
  Wire.begin();
  delay(10);
  Wire.setClock(400000);
  setupVlx(vlxID::left);
  setupVlx(vlxID::frontRight);
  setupVlx(vlxID::right);
  setupVlx(vlxID::frontLeft);
  setupVlx(vlxID::back);
  //setupTCS();
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].initialize(Pins::digitalOne[i], Pins::digitalTwo[i],
                        Pins::pwmPin[i], i);
    myPID[i].changeConstants(kP_mov, kI_mov, kD_mov, movTime);
  }
  rampUpPID.changeConstants(kP_RampUp, kI_RampUp, kD_RampUp, rampTime);
  rampDownPID.changeConstants(kP_RampDown, kI_RampDown, kD_RampDown, rampTime);
  limitSwitch_[LimitSwitchID::kLeft].initLimitSwitch(Pins::limitSwitchPins[LimitSwitchID::kLeft]);
  limitSwitch_[LimitSwitchID::kRight].initLimitSwitch(Pins::limitSwitchPins[LimitSwitchID::kRight]);
  targetAngle = 0;
  delay(500);
  bno.setupBNO();
}

void motors::printAngle() {
  double angulo = bno.getOrientationX();
  Serial.println(angulo);
}

void motors::PID_speed(float setpoint, float angle, uint16_t reference_speed) {
  PID PID;
  double output = PID.calculate_PID(setpoint, angle);
  int right_speed = reference_speed - output;
  int left_speed  = reference_speed + output;
  right_speed = constrain(right_speed, 30, 255);
  left_speed  = constrain(left_speed,  30, 255);
  for (int i = 0; i < 4; i++) {
    motor[i].setSpeed((i % 2 == 0) ? left_speed : right_speed);
  }
  // FIX 2: eliminado delay(20) — la cadencia la controla el PID con calculate_time
}

void motors::PID_Wheel(int targetSpeed, int i) {
  int speed_setpoint = targetSpeed;
  int reference_pwm  = motor[i].getSpeed();
  int speedTics      = motor[i].getTicsSpeed();

  // FIX 3: Serial solo activo si DEBUG_SERIAL está definido
#ifdef DEBUG_SERIAL
  Serial.print("PWM["); Serial.print(i); Serial.print("]=");
  Serial.print(reference_pwm);
  Serial.print(" tics="); Serial.println(speedTics);
#endif

  float error = myPID[i].calculate_PID(speed_setpoint, speedTics);
  int speed   = reference_pwm + error;
  speed       = constrain(speed, 0, 255);
  motor[i].setSpeed(speed);
}

void motors::PID_AllWheels(int targetSpeed) {
  for (uint8_t i = 0; i < 4; i++) {
    PID_Wheel(targetSpeed, i);
  }
}

void motors::pidEncoders(int speedReference, bool ahead) {
  bno.getOrientationX();
  //static PID pidBno(0.78, 0.00, 0.3, 1);
  static PID pidBno(1.458, 0.025, 0.168, 12);
  if (rampState != 0) changeAngle = 0;
  float AngleError = pidBno.calculate_PID(
      targetAngle + changeAngle, (targetAngle == 0 ? z_rotation : angle));
  AngleError = constrain(AngleError, -18, 18);
  if (!ahead) AngleError = -AngleError;
  PID_Wheel(speedReference + AngleError, MotorID::kFrontLeft);
  PID_Wheel(speedReference + AngleError, MotorID::kBackLeft);
  PID_Wheel(speedReference - AngleError, MotorID::kFrontRight);
  PID_Wheel(speedReference - AngleError, MotorID::kBackRight);
}

// FIX 5: timeout máximo en ahead() — si los encoders fallan no se queda colgado
static const unsigned long kAheadTimeoutMs = 5000; // 5 s por tile

void motors::ahead() {
  passObstacle();
  passObstacle();
  nearWall();
  resetTics();
  int offset = 0;
  float distance;
  bool encoder, frontVlx;
  bool rampCaution = false;

  float frontDistance = max(vlx[vlxID::frontLeft].getDistance(),
                            vlx[vlxID::frontRight].getDistance());
  float backDistance;

  bool frontVlxValid = (frontDistance >= 1 && frontDistance <= maxVlxDistance);
  if (frontVlxValid) {
    distance = frontDistance; encoder = false; frontVlx = true;
  } else {
    backDistance = vlx[vlxID::back].getDistance();
    bool backVlxValid = (backDistance >= 1 &&
                         backDistance <= (maxVlxDistance - kTileLength));
    if (backVlxValid) {
      distance = backDistance; encoder = false; frontVlx = false;
    } else {
      encoder = true;
    }
  }

  //if (frontDistance > 800) rampCaution = true;
  if (abs(bno.getOrientationY()) > 15) { encoder = true; offset = kTicsPerTile / 6; }

  unsigned long tStart = millis(); // FIX 5: referencia de tiempo para timeout

  if (!encoder) {
    float targetDistance = findNearest(
        distance, frontVlx ? targetDistances : targetDistancesB, 2, frontVlx);
    while (frontVlx ? (distance >= targetDistance)
                    : (distance <= targetDistance)) {
      // FIX 5: salida por timeout
      if ((millis() - tStart) > kAheadTimeoutMs) { stop(); resetTics(); return; }
      limitCrash();
      setahead();
      if (blackTile)  break;
      if (buttonPressed) break;
      if (isRamp())   break;

      distance = (frontVlx
                  ? max(vlx[vlxID::frontLeft].getDistance(), vlx[vlxID::frontRight].getDistance())
                  : vlx[vlxID::back].getDistance());

      // Si VLX sale del rango util en movimiento, continuar por encoder.
      if (distance < 1 || distance > maxVlxDistance) {
        encoder = true;
        break;
      }

      float missingDistance = abs(distance - targetDistance);
      float speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard, kMinSpeedFormard);

      if (slope) {
        missingDistance = kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
        speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard, kMinSpeedFormard);
        if (getAvergeTics() > kTicsPerTile) break;
      }
      if (rampCaution)
        speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard / 3, kMinSpeedFormard);
      speed = constrain(speed, kMinSpeedFormard, kMaxSpeedFormard);
      pidEncoders(speed, true);
    }

    if (encoder) {
      while (getAvergeTics() < kTicsPerTile - offset) {
        if ((millis() - tStart) > kAheadTimeoutMs) { stop(); resetTics(); return; }

        setahead();
        if (blackTile)    return;
        if (buttonPressed) break;
        if (isRamp())     break;

        float missingDistance = kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
        float speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard, kMinSpeedFormard);
        speed = constrain(speed, kMinSpeedFormard, kMaxSpeedFormard);
        if (rampCaution)
          speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard / 3, kMinSpeedFormard);
        pidEncoders(speed, true);
      }
    }
  } else {
    while (getAvergeTics() < kTicsPerTile - offset) {
      // FIX 5: salida por timeout
      if ((millis() - tStart) > kAheadTimeoutMs) { stop(); resetTics(); return; }
      limitCrash();
      if (BothLimits) { stop(); return; }
      setahead();
      if (blackTile)    return;
      if (buttonPressed) break;
      if (isRamp())     break;

      float missingDistance = kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
      float speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard, kMinSpeedFormard);
      speed = constrain(speed, kMinSpeedFormard, kMaxSpeedFormard);
      if (rampCaution)
        speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard / 3, kMinSpeedFormard);
      pidEncoders(speed, true);
    }
  }
  slope = false;
  stop();
  resetTics();
}

void motors::checkTileColor() {
  char tileColor = tcs_.getColor();
  Serial.println(tileColor);
  if (tileColor == kBlackColor && inMotion == true) {
    inMotion = false;
    resetTics();
    setback();
    blackTile = true;
    while (getAvergeTics() < kTicsPerTile / 2) {
      setback();
      if (buttonPressed) break;
      int speed = map(getAvergeTics(), 0, kTicsPerTile / 2, kMaxSpeedFormard, kMinSpeedFormard);
      pidEncoders(speed, false);
    }
    stop();
    resetTics();
  } else if (tileColor == kBlueColor && inMotion == false) {
    Serial.println("Blue Tile");
    blueTile = true;
    wait(5500);
  } else if (tileColor == kCheckpointColor && inMotion == false) {
    checkpoint = true;
    Serial.println("Checkpoint Tile");
  }
}

float motors::nearWall() {
  if (vlx[vlxID::left].getDistance() < minDisToLateralWall) {
    changeAngle = maxChangeAngle;
  } else if (vlx[vlxID::right].getDistance() < minDisToLateralWall) {
    changeAngle = -maxChangeAngle;
  } else {
    changeAngle = 0;
  }
  return changeAngle;
}

void motors::passObstacle() {
  float targetAngle_ = targetAngle;
  float angle        = bno.getOrientationX();
  if (abs(targetAngle_ - angle) >= 2) rotate(targetAngle_);
  bool leftBlocked  = vlx[vlxID::frontLeft].getDistance()  < kDistanceToObstacle;
  bool rightBlocked = vlx[vlxID::frontRight].getDistance() < kDistanceToObstacle;
  if (!leftBlocked && !rightBlocked) return;
  if (leftBlocked  && rightBlocked)  return;
  if(!vlx[vlxID::back].isWall()) {
    moveDistance(kTileLength / 5, false);
  }
  limitColition = true;
  if (leftBlocked || rightBlocked) {
    float sideAngle = targetAngle + (leftBlocked ? 25 : -25);
    if (sideAngle >= 360) sideAngle -= 360;
    if (sideAngle <  0)   sideAngle += 360;
    rotate(sideAngle);
    moveDistance(3 * kTileLength / 10, true);
  }
  rotate(targetAngle_);
  limitColition = false;
}

uint8_t motors::findNearest(float number, const uint8_t numbers[], uint8_t size, bool frontVlx) {
  if (frontVlx)       number -= kTileLength;
  else if (!frontVlx) number += kTileLength;
  uint8_t nearest       = numbers[0];
  float   minDifference = abs(number - numbers[0]);
  for (uint8_t i = 1; i < size; i++) {
    float currentDifference = abs(number - numbers[i]);
    if (currentDifference < minDifference) {
      nearest       = numbers[i];
      minDifference = currentDifference;
    }
  }
  return nearest;
}

void motors::back() { setback(); }

void motors::right() {
  Serial.println("right");
  targetAngle = targetAngle + 90;
  rotate(targetAngle);
  if (targetAngle == 360) targetAngle = 0;
}

void motors::left() {
  Serial.println("left");
  if (targetAngle == 0) targetAngle = 360;
  targetAngle = targetAngle - 90;
  rotate(targetAngle);
}

float motors::calculateAngularDistance() {
  float rightAngularDistance, leftAngularDistance;
  if (targetAngle >= angle) {
    rightAngularDistance = targetAngle - angle;
    leftAngularDistance  = angle + (360 - targetAngle);
  } else {
    rightAngularDistance = (360 - angle) + targetAngle;
    leftAngularDistance  = angle - targetAngle;
  }
  return (rightAngularDistance <= leftAngularDistance) ? rightAngularDistance : -leftAngularDistance;
}

void motors::rotate(float deltaAngle) {
  targetAngle = deltaAngle;
  delayMicroseconds(1);
  bno.getOrientationX();
  float currentAngle, rightAngularDistance, leftAngularDistance,
        minInterval, maxInterval, tolerance = 2;
  bool hexadecimal;
  if (targetAngle >= angle) {
    rightAngularDistance = targetAngle - angle;
    leftAngularDistance  = angle + (360 - targetAngle);
  } else {
    rightAngularDistance = (360 - angle) + targetAngle;
    leftAngularDistance  = angle - targetAngle;
  }
  if (targetAngle != 360 && targetAngle != 0) {
    minInterval = targetAngle - tolerance;
    maxInterval = targetAngle + tolerance;
    hexadecimal = true;
  } else {
    targetAngle = 0;
    minInterval = -tolerance; maxInterval = tolerance;
    hexadecimal = false;
  }
  (rightAngularDistance <= leftAngularDistance) ? setright() : setleft();
  currentAngle = hexadecimal ? angle : z_rotation;
  while (currentAngle < minInterval || currentAngle > maxInterval) {
    changeSpeedMove(false, true, 0, false);
    bno.getOrientationX();
    currentAngle = hexadecimal ? angle : z_rotation;
    if (buttonPressed) break;
  }
  stop();
  inMotion = true;
}

float motors::changeSpeedMove(bool encoders, bool rotate, int targetDistance, bool frontVlx) {
  float speed, missingDistance, missingAngle;
  if (rotate) {
    missingAngle = abs(targetAngle - (targetAngle == 0 ? z_rotation : angle));
    speed = map(missingAngle, 90, 0, kMaxSpeedRotate, kMinSpeedRotate);
    speed = constrain(speed, kMinSpeedRotate, kMaxSpeedRotate);
    PID_AllWheels(speed);
    return 0;
  } else {
    if (encoders) {
      speed = map(getAvergeTics(), 0, kTicsPerTile, kMaxPwmFormard, kMinPwmFormard);
      missingDistance = kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
      return speed;
    } else {
      missingDistance = abs((frontVlx ? vlx[vlxID::frontLeft].getDistance()
                                      : vlx[vlxID::back].getDistance()) - targetDistance);
      speed = map(missingDistance, kTileLength, 0, kMaxPwmFormard, kMinPwmFormard);
      speed = constrain(speed, kMinPwmFormard, kMaxPwmFormard);
      return speed;
    }
  }
}

void motors::limitCrash() {
  if (slope) return;
  float targetAngle_ = targetAngle;
  bool leftState = limitSwitch_[LimitSwitchID::kLeft].getState();
  bool rightState = limitSwitch_[LimitSwitchID::kRight].getState();

    if (!leftState && !rightState) return;
    delay(50);
    if(leftState && rightState) {
      BothLimits = true;
      stop();
      resetTics();
      return;
    }

    if (rampState != 0) {
      if (leftState || rightState) limitColition = true;
      return;
    }

    moveDistance(kTileLength / 5, false);

    float sideAngle = targetAngle + (leftState ? 25 : -25);
    if (sideAngle >= 360) sideAngle -= 360;
    if (sideAngle < 0) sideAngle += 360;
    rotate(sideAngle);
    moveDistance(3 * kTileLength / 10, true);

    rotate(targetAngle_);
    limitColition = false;
    resetTics();
    setahead();
}

void motors::setSpeed(uint16_t speed) {
  for (uint8_t i = 0; i < 4; i++) motor[i].setSpeed(speed);
}

void motors::setahead() {
  for (int i = 0; i < 4; i++) motor[i].ahead();
  inMotion = true;
}

void motors::setback() {
  for (int i = 0; i < 4; i++) motor[i].back();
}

void motors::setright() {
  motor[MotorID::kBackLeft].ahead();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].ahead();
  motor[MotorID::kFrontRight].back();
}

void motors::setleft() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].ahead();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].ahead();
}

void motors::setrightTraslation() {
  motor[MotorID::kBackLeft].stop();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].stop();
  motor[MotorID::kFrontRight].back();
}

void motors::setleftTraslation() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].stop();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].stop();
}

void motors::stop() {
  for (uint8_t i = 0; i < 4; i++) motor[i].stop();
  setSpeed(0);
  inMotion = false;
}

void motors::printSpeeds() {
  Serial.println("speed:");
  for (int i = 0; i < 4; i++) {
    Serial.print("Motor"); Serial.print(i + 1);
    Serial.print(":"); Serial.println(motor[i].getSpeed());
  }
}

void motors::resetTics() {
  for (uint8_t i = 0; i < 4; i++) motor[i].resetTics();
}

double motors::getAvergeTics() {
  float totalTics = 0;
  for (int i = 0; i < 4; i++) totalTics += motor[i].tics;
  return totalTics / 4;
}

double motors::getTicsSpeed() {
  // FIX 4: inicializar ticsSpeed antes de acumular — evita valor basura
  float ticsSpeed = 0.0f;
  for (int i = 0; i < 4; i++) ticsSpeed += motor[i].getTicsSpeed();
  return ticsSpeed / 4.0;
}

void motors::setupVlx(const uint8_t index) {
  vlx[index].setMux(index);
  vlx[index].begin();
}

void motors::resetVlx() {
  screenBegin();
  setupVlx(vlxID::frontLeft);
  setupVlx(vlxID::left);
  setupVlx(vlxID::frontRight);
  setupVlx(vlxID::right);
  setupVlx(vlxID::back);
}

bool motors::isWall(uint8_t direction) {
  uint8_t relativeDir = 0;
  int deltaTargetAngle = static_cast<int>(targetAngle);
  switch (deltaTargetAngle) {
    case 0: case 360: relativeDir = 0; break;
    case 90:          relativeDir = 1; break;
    case 180:         relativeDir = 2; break;
    case 270:         relativeDir = 3; break;
    default:          relativeDir = 0; break;
  }
  uint8_t realPos   = rulet[relativeDir][direction];
  bool frontLeft    = vlx[vlxID::frontLeft].isWall();
  bool frontRight   = vlx[vlxID::frontRight].isWall();
  bool right        = vlx[vlxID::right].isWall();
  bool back         = vlx[vlxID::back].isWall();
  bool left         = vlx[vlxID::left].isWall();
  switch (realPos) {
    case 0: return frontLeft && frontRight;
    case 1: return right;
    case 2: return back;
    case 3: return left;
    default: return false;
  }
}

bool motors::rampInFront() {
  return ((vlx[vlxID::frontLeft].getDistance() -
           vlx[vlxID::frontRight].getDistance()) >= 2);
}

bool motors::isRamp() {
  float currentOrientationY = bno.getOrientationY();
  if (abs(currentOrientationY) > 7) slope = true;
  if (currentOrientationY >= kMinRampOrientation ||
      currentOrientationY <= -kMinRampOrientation) {
    ramp(); return true;
  }
  return false;
}

void motors::ramp() {
  resetTics();
  setahead();
  while (bno.getOrientationY() > 7) {
    if (buttonPressed)    break;
    if (limitColition)    { stop(); break; }
    float error;
    vlx[vlxID::right].getDistance();
    vlx[vlxID::left].getDistance();
    if ((vlx[vlxID::right].distance < vlx[vlxID::right].kDistanceToWall &&
         vlx[vlxID::left].distance  < vlx[vlxID::left].kDistanceToWall) &&
        (vlx[vlxID::right].distance < 6 || vlx[vlxID::left].distance < 6)) {
      error = rampUpPID.calculate_PID(
          0, (vlx[vlxID::right].distance - vlx[vlxID::left].distance));
      error = constrain(error, -15, 15);
      PID_Wheel(kSpeedRampUp - error, MotorID::kFrontLeft);
      PID_Wheel(kSpeedRampUp - error, MotorID::kBackLeft);
      PID_Wheel(kSpeedRampUp + error, MotorID::kFrontRight);
      PID_Wheel(kSpeedRampUp + error, MotorID::kBackRight);
    } else {
      pidEncoders(kSpeedRampUp, true);
    }
    rampState = 1;
    screenPrint("rampUp");
  }
  while (bno.getOrientationY() < -7) {
    if (buttonPressed)    break;
    if (limitColition)    break;
    float error;
    vlx[vlxID::right].getDistance();
    vlx[vlxID::left].getDistance();
    if ((vlx[vlxID::right].distance < vlx[vlxID::right].kDistanceToWall &&
         vlx[vlxID::left].distance  < vlx[vlxID::left].kDistanceToWall) &&
        (vlx[vlxID::right].distance < 6 || vlx[vlxID::left].distance < 6)) {
      error = rampDownPID.calculate_PID(
          0, (vlx[vlxID::right].distance - vlx[vlxID::left].distance));
      error = constrain(error, -6, 6);
      PID_Wheel(kSpeedRampDown - error, MotorID::kFrontLeft);
      PID_Wheel(kSpeedRampDown - error, MotorID::kBackLeft);
      PID_Wheel(kSpeedRampDown + error, MotorID::kFrontRight);
      PID_Wheel(kSpeedRampDown + error, MotorID::kBackRight);
    } else {
      pidEncoders(kSpeedRampDown, true);
    }
    rampState = 2;
  }
  if (getAvergeTics() > 1 * kTicsPerTile && rampState == 1) {
    moveDistance(kTileLength / 3, true);
    rotate(targetAngle);
  } else if (getAvergeTics() > 0.8 * kTicsPerTile && rampState == 2) {
    moveDistance(kTileLength / 2, true);
    rotate(targetAngle);
  } else {
    rampState = 0;
  }
  limitColition = false;
  resetTics();
  stop();
  wait(100);
}

// FIX 5: timeout en moveDistance — si el encoder falla no se bloquea
void motors::moveDistance(uint8_t targetDistance, bool ahead) {
  ahead ? setahead() : setback();
  resetTics();
  unsigned long tStart = millis();
  while (getCurrentDistanceCm() < targetDistance) {
    if (buttonPressed) break;
    if ((millis() - tStart) > kAheadTimeoutMs) break; // FIX 5
    pidEncoders((kMinSpeedFormard + kMaxSpeedFormard) / 2, ahead);
  }
  stop();
}

float motors::getCurrentDistanceCm() {
  return getAvergeTics() * kTileLength / kTicsPerTile;
}

float motors::getAngleOrientation() {
  float currentAngle = bno.getOrientationX();
  if ((currentAngle > 315 && currentAngle <= 360) || (currentAngle >= 0 && currentAngle <= 45))
    return 0;
  else if (currentAngle > 45  && currentAngle <= 135) return 90;
  else if (currentAngle > 135 && currentAngle <= 225) return 180;
  else if (currentAngle > 225 && currentAngle <= 315) return 270;
  else return currentAngle;
}

void motors::resetOrientation() {
  bno.resetOrientation();
  targetAngle = 0;
}

void motors::checkpointElection() {
  float angleOrientation = getAngleOrientation();
  Serial.println(angleOrientation);
  uint8_t angleThreshold = 10;
  float currentAngle = (angleOrientation == 0) ? z_rotation : angle;
  if (abs(currentAngle - angleOrientation) < angleThreshold) return;
  rotate(angleOrientation);
  if ((currentAngle - angleOrientation) < -angleThreshold) {
    ahead(); left(); ahead();
  } else if ((currentAngle - angleOrientation) > angleThreshold) {
    ahead(); right(); ahead();
  }
}

void motors::victimSequency() {
  float current = millis();
  while ((millis() - current) < 5100) {
    screenPrint("Victim");
    delay(500);
    screenPrint(" ");
    delay(500);
  }
}

void motors::harmedVictim() {
  victimSequency();
  if      (kitState == kitID::kRight) kitRight(2);
  else if (kitState == kitID::kLeft)  kitLeft(2);
}

void motors::stableVictim() {
  victimSequency();
  if      (kitState == kitID::kRight) kitRight(1);
  else if (kitState == kitID::kLeft)  kitLeft(1);
}

void motors::unharmedVictim() {
  victimSequency();
}

void motors::kitLeft(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosRight);
    writeServo(servoPosLeft);
    writeServo(10);
    writeServo(servoPosLeft);
    writeServo(90);
  }
}

void motors::writeServo(uint16_t servoAngle) {
  servo.write(servoAngle);
  delay(200);
  if ((servoAngle != 10 && servoAngle != 90) && servoAngle != 170) {
    // FIX 6: variable i inicializada a 0 — antes era basura
    for (uint8_t i = 0; i < 4; i++) {
      servo.write(servoAngle - 6);
      delay(40);
      servo.write(servoAngle + 6);
      delay(40);
    }
  }
}

void motors::kitRight(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosLeft);
    writeServo(servoPosRight);
    writeServo(170);
    writeServo(servoPosRight);
    writeServo(90);
  }
}

void motors::reloadKits() {
  writeServo(servoPosLeft);
  writeServo(servoPosRight);
  writeServo(90);
}

void motors::wait(unsigned long targetTime) {
  unsigned long initialTime = millis();
  while ((millis() - initialTime) < targetTime) {
    if (buttonPressed) break;
  }
}

void motors::wifiPrint(String message, float i) {}

void motors::setupTCS() {
  tcs_.setMux(Pins::tcsPins[0]);
  tcs_.init();
  bno.setPhaseCorrection(bno.getOrientationX());
  bno.setPhaseCorrectionY(bno.getOrientationY());
}

void motors::screenBegin() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println("Screen initialized");
}

void motors::screenPrint(String output) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(output);
  display.display();
}

void motors::calibrateColors() {
  uint16_t dt = 10000;
  Serial.println("Blue Tile");   delay(dt); tcs_.updateRGBC(); tcs_.printRGB();
  float redInBlue   = tcs_.red_; float greenInBlue  = tcs_.green_; float blueInBlue  = tcs_.blue_;
  Serial.println("Black Tile");  delay(dt); tcs_.updateRGBC(); tcs_.printRGB();
  float redInBlack  = tcs_.red_; float greenInBlack = tcs_.green_; float blueInBlack = tcs_.blue_;
  Serial.println("Checkpoint");  delay(dt); tcs_.updateRGBC(); tcs_.printRGB();
  float redInCheck  = tcs_.red_; float greenInCheck = tcs_.green_; float blueInCheck = tcs_.blue_;
  Serial.println("White Tile");  delay(dt); tcs_.updateRGBC(); tcs_.printRGB();
  float redInWhite  = tcs_.red_; float greenInWhite = tcs_.green_; float blueInWhite = tcs_.blue_;
  while (true) {
    if (buttonPressed) break;
    dt = 2000;
    auto printColor = [&](const char* name, float r, float g, float b) {
      Serial.print(name);
      delay(800);
      Serial.println(String(r) + "," + String(g) + "," + String(b));
      delay(dt);
    };
    printColor("RGB Blue",       redInBlue,  greenInBlue,  blueInBlue);
    printColor("RGB Black",      redInBlack, greenInBlack, blueInBlack);
    printColor("RGB Checkpoint", redInCheck, greenInCheck, blueInCheck);
    printColor("RGB White",      redInWhite, greenInWhite, blueInWhite);
  }
}