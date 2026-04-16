#include "motors.h"
#include "Encoder.h"
#include "Pins_ID.h"
#include <WiFi.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
motors::motors() {}

namespace {
float normalize360(float angleValue) {
  while (angleValue >= 360.0f) {
    angleValue -= 360.0f;
  }
  while (angleValue < 0.0f) {
    angleValue += 360.0f;
  }
  return angleValue;
}

float signedAngleError(float target, float current) {
  float error = normalize360(target) - normalize360(current);
  if (error > 180.0f) {
    error -= 360.0f;
  } else if (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

float frontMinDistance(float leftDistance, float rightDistance) {
  const bool leftValid = leftDistance >= 1.0f && leftDistance < 819.0f;
  const bool rightValid = rightDistance >= 1.0f && rightDistance < 819.0f;

  if (leftValid && rightValid) {
    return min(leftDistance, rightDistance);
  }
  if (leftValid) {
    return leftDistance;
  }
  if (rightValid) {
    return rightDistance;
  }
  return 819.0f;
}

float stableDistance(VLX &sensor) {
  float first = sensor.getDistance();
  delay(2);
  float second = sensor.getDistance();

  if (first >= 819.0f) {
    return second;
  }
  if (second >= 819.0f) {
    return first;
  }
  return (first + second) * 0.5f;
}
} // namespace

void motors::setupMotors() {
  Wire.begin();
  delay(10);
  Wire.setClock(400000);
  screenBegin();
  bno.setupBNO();
  setupVlx(vlxID::left);
  setupVlx(vlxID::frontRight);
  setupVlx(vlxID::right);
  setupVlx(vlxID::frontLeft);
  setupVlx(vlxID::back);
  leds.setupLeds();
  setupTCS();
  servo[servosID::kLeft].attach(Pins::servos[servosID::kLeft]);
  Serial.println("Servo Left attached");
  servo[servosID::kRight].attach(Pins::servos[servosID::kRight]);
  Serial.println("Servo Right attached");
  servo[servosID::kLeft].write(servoPos);
  servo[servosID::kRight].write(servoPosLeft);

  for (uint8_t i = 0; i < 4; i++) {
    motor[i].initialize(Pins::digitalOne[i], Pins::digitalTwo[i],
                        Pins::pwmPin[i], i);
    myPID[i].changeConstants(kP_mov, kI_mov, kD_mov, movTime);
  }

  turnPID_.configure(2.4f, 0.0f, 0.22f);
  turnPID_.setOutputLimits(kMinSpeedRotate, kMaxSpeedRotate);
  turnPID_.setSettleCriteria(1.2f, 140);

  rampUpPID.changeConstants(kP_RampUp, kI_RampUp, kD_RampUp, rampTime);
  rampDownPID.changeConstants(kP_RampDown, kI_RampDown, kD_RampDown, rampTime);
  limitSwitch_[LimitSwitchID::kLeft].initLimitSwitch(Pins::limitSwitchPins[LimitSwitchID::kLeft]);
  limitSwitch_[LimitSwitchID::kRight].initLimitSwitch(Pins::limitSwitchPins[LimitSwitchID::kRight]);
  targetAngle = 0;
  delay(500);
}
void motors::printAngle() {
  double angulo = bno.getOrientationX();
  Serial.println(angulo);
}

void motors::PID_speed(float setpoint, float angle, uint16_t reference_speed) {
  PID PID;
  double output = PID.calculate_PID(setpoint, angle);
  int right_speed = reference_speed - output;
  int left_speed = reference_speed + output;
  right_speed = constrain(right_speed, 30, 255);
  left_speed = constrain(left_speed, 30, 255);
  for (int i = 0; i < 4; i++) {
    motor[i].setSpeed((i % 2 == 0) ? left_speed : right_speed);
  }
  delay(20);
}
void motors::PID_Wheel(int targetSpeed, int i) {
  int speed_setpoint = targetSpeed;
  int reference_pwm;
  reference_pwm = motor[i].getSpeed();
  int speedTics = motor[i].getTicsSpeed();
  float error = myPID[i].calculate_PID(speed_setpoint, speedTics);
  int speed = reference_pwm + error;
  const int minEffectivePwm = 50;
  if (speed_setpoint > 0 && speed > 0 && speed < minEffectivePwm) {
    speed = minEffectivePwm;
  }
  speed = constrain(speed, 0, 255);
  motor[i].setSpeed(speed);
}
void motors::PID_AllWheels(int targetSpeed) {
  for (uint8_t i = 0; i < 4; i++) {
    PID_Wheel(targetSpeed, i);
  }
}

void motors::pidEncoders(int speedReference, bool ahead) {
  bno.getOrientationX();
  static PID pidBno(0.5, 0.1, 0.01, 1);
  static PID pidWheelBalance(0.6, 0.0, 0.03, 20);
  static float smoothedHeadingTarget = 0.0f;
  static bool headingInitialized = false;
  static float lastTotalCorrection = 0.0f;

  if (rampState != 0)
    changeAngle = 0;

  const float currentHeading = (targetAngle == 0 ? z_rotation : angle);
  const float desiredHeading = normalize360(targetAngle + changeAngle);

  if (!headingInitialized) {
    smoothedHeadingTarget = desiredHeading;
    headingInitialized = true;
  }

  const float kHeadingTargetMaxStepDeg = 0.8f;
  float headingTargetStep =
      signedAngleError(desiredHeading, smoothedHeadingTarget);
  headingTargetStep = constrain(headingTargetStep, -kHeadingTargetMaxStepDeg,
                                kHeadingTargetMaxStepDeg);
  smoothedHeadingTarget = normalize360(smoothedHeadingTarget + headingTargetStep);

  float AngleError = pidBno.calculate_PID(smoothedHeadingTarget, currentHeading);
  AngleError = constrain(AngleError, -12, 12);

  const float leftMeasured =
      (motor[MotorID::kFrontLeft].getTicsSpeed() +
       motor[MotorID::kBackLeft].getTicsSpeed()) /
      2.0f;
  const float rightMeasured =
      (motor[MotorID::kFrontRight].getTicsSpeed() +
       motor[MotorID::kBackRight].getTicsSpeed()) /
      2.0f;

  const float wheelDiff = leftMeasured - rightMeasured;
  float balanceError = pidWheelBalance.calculate_PID(0.0f, wheelDiff);
  balanceError = constrain(balanceError, -6.0f, 6.0f);

  if (!ahead)
    AngleError = -AngleError;

  float totalCorrection = AngleError - balanceError;
  totalCorrection = constrain(totalCorrection, -9.0f, 9.0f);

  const float kCorrectionSlewPerCycle = 1.2f;
  float correctionDelta = totalCorrection - lastTotalCorrection;
  correctionDelta = constrain(correctionDelta, -kCorrectionSlewPerCycle,
                              kCorrectionSlewPerCycle);
  totalCorrection = lastTotalCorrection + correctionDelta;
  lastTotalCorrection = totalCorrection;

  const int leftReference = static_cast<int>(speedReference + totalCorrection);
  const int rightReference = static_cast<int>(speedReference - totalCorrection);

  PID_Wheel(leftReference, MotorID::kFrontLeft);
  PID_Wheel(leftReference, MotorID::kBackLeft);
  PID_Wheel(rightReference, MotorID::kFrontRight);
  PID_Wheel(rightReference, MotorID::kBackRight);
}

void motors::ahead() {
  passObstacle();
  passObstacle();

  nearWall();
  resetTics();
  int offset = 0;
  ;
  float distance;
  bool encoder, frontVlx;
  bool rampCaution = false;
  float frontLeftDistance = vlx[vlxID::frontLeft].getDistance();
  float frontRightDistance = vlx[vlxID::frontRight].getDistance();
  float frontDistance = max(frontLeftDistance, frontRightDistance);
  const float kFrontStopDistance = 4.0f;

  if (frontDistance <= kFrontStopDistance) {
    stop();
    return;
  }

  float backDistance;
  if (frontDistance < maxVlxDistance && frontDistance >= 1) {
    distance = frontDistance;
    encoder = false;
    frontVlx = true;
  } /*else {
    backDistance = vlx[vlxID::back].getDistance();
    if ((backDistance < maxVlxDistance - kTileLength) && frontDistance >= 1) {
      distance = backDistance;
      encoder = false;
      frontVlx = false;
      
    } */else{
      encoder = true;
  }

  String print = static_cast<String>(frontDistance);
  // robot.screenPrint(print);
  if (frontDistance > 800)
    rampCaution = true;
  // if (abs(bno.getOrientationY()) > 15) {
  //   encoder = true;
  //   offset = kTicsPerTile / 6;
  // }
  if (!encoder) {
    float targetDistance = findNearest(
        distance, frontVlx ? targetDistances : targetDistancesB, 2, frontVlx);
    targetDistance = targetDistance;
    while (frontVlx ? (distance >= targetDistance)
                    : (distance <= targetDistance)) {
      setahead();
      // checkTileColor();
      if (blackTile)
        break;
      if (buttonPressed)
        break;
      if (isRamp())
        break;
      limitCrash();
      if (frontVlx) {
        frontLeftDistance = vlx[vlxID::frontLeft].getDistance();
        frontRightDistance = vlx[vlxID::frontRight].getDistance();
        distance = max(frontLeftDistance, frontRightDistance);
        if (distance <= kFrontStopDistance) {
          stop();
          break;
        }
      } else {
        distance = vlx[vlxID::back].getDistance();
      }
      float missingDistance = abs(distance - targetDistance);
      float speed;
      speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard,
                  kMinSpeedFormard);
      if (slope == true) {
        missingDistance =
            kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
        speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard,
                    kMinSpeedFormard);
        if (getAvergeTics() > kTicsPerTile)
          break;
      }
      if (rampCaution)
        speed = map(missingDistance, kTileLength, 0, (kMaxSpeedFormard / 3),
                    kMinSpeedFormard);
      speed = constrain(speed, kMinSpeedFormard, kMaxSpeedFormard);
      pidEncoders(speed, true);
      screenPrint(String(distance));
    }
  } else if (encoder) {
    while (getAvergeTics() < kTicsPerTile - offset) {
      setahead();
      limitCrash();
      // checkTileColor();
      if (blackTile)
        return;
      if (buttonPressed)
        break;
      if (isRamp())
        break;
      frontLeftDistance = vlx[vlxID::frontLeft].getDistance();
      frontRightDistance = vlx[vlxID::frontRight].getDistance();
      frontDistance = max(frontLeftDistance, frontRightDistance);
      if (frontDistance <= kFrontStopDistance) {
        stop();
        break;
      }
      float missingDistance =
          kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
      float speed = map(missingDistance, kTileLength, 0, kMaxSpeedFormard,
                        kMinSpeedFormard);
      speed = constrain(speed, kMinSpeedFormard, kMaxSpeedFormard);
      if (rampCaution)
        speed = map(missingDistance, kTileLength, 0, (kMaxSpeedFormard / 3),
                    kMinSpeedFormard);
      pidEncoders(speed, true);
    }
  }
  slope = false;
  stop();
  resetTics();
  // checkTileColor();
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
      if (buttonPressed)
        break;
      int speed = map(getAvergeTics(), 0, kTicsPerTile / 2, kMaxSpeedFormard,
                      kMinSpeedFormard);
      pidEncoders(speed, false);
      Serial.println("Black Tile");
      // screenPrint("Black Tile");
    }
    stop();
    resetTics();
  } else if (tileColor == kBlueColor && inMotion == false) {
    // screenPrint("Blue Tile");
    Serial.println("Blue Tile");
    blueTile = true;
    wait(5500);
  } else if (tileColor == kCheckpointColor && inMotion == false) {
    checkpoint = true;
    Serial.println("Checkpoint Tile");
    // screenPrint("Checkpoint Tile");
  }
}

float motors::nearWall() {
  // float left = AverageLeftDistance();
  // float right = AverageRightDistance();
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
  float sideAngle = targetAngle_;
  float angle = bno.getOrientationX();
  if (abs(targetAngle_ - angle) >= 2)
    rotate(targetAngle_);
  bool leftBlocked = vlx[vlxID::frontLeft].getDistance() < kDistanceToObstacle;
  bool rightBlocked =
      vlx[vlxID::frontRight].getDistance() < kDistanceToObstacle;

  if (!leftBlocked && !rightBlocked)
    return; // No obstacle, do nothing
  if (leftBlocked && rightBlocked) {
    stop();
    return; // Completely blocked, can't pass
  }

  moveDistance(kTileLength / 5, false);
  limitColition = true;
  sideAngle = targetAngle + (leftBlocked ? 25 : -25);
  if (sideAngle >= 360)
    sideAngle -= 360;
  if (sideAngle < 0)
    sideAngle += 360;

  rotate(sideAngle);
  moveDistance(3 * kTileLength / 10, true);
  rotate(targetAngle_);
  limitColition = false;
}
void motors::limitCrash() {
  if (slope) return;
  float targetAngle_ = targetAngle;
  bool leftState = limitSwitch_[LimitSwitchID::kLeft].getState();
  bool rightState = limitSwitch_[LimitSwitchID::kRight].getState();

    if (!leftState && !rightState) return;

    if (rampState != 0) {
      if (leftState || rightState) limitColition = true;
      return;
    }

    moveDistance(kTileLength / 5, false);

    float sideAngle = targetAngle + (leftState ? 25 : -25);
    if (sideAngle >= 360) sideAngle -= 360;
    if (sideAngle < 0) sideAngle += 360;
    moveDistance(kTileLength / 5, false);
    rotate(sideAngle);
    moveDistance(3 * kTileLength / 10, true);

    rotate(targetAngle_);
    limitColition = false;
    resetTics();   // <-- reset so encoder loop restarts correctly
    setahead();    // <-- resume forward motionz
}

uint8_t motors::findNearest(float number, const uint8_t numbers[], uint8_t size,
                            bool frontVlx) {
  if (frontVlx)
    number -= kTileLength;
  else if (!frontVlx)
    number += kTileLength;
  uint8_t nearest = numbers[0];
  float minDifference = abs(number - numbers[0]);
  for (uint8_t i = 1; i < size; i++) {
    float currentDifference = abs(number - numbers[i]);
    if (currentDifference < minDifference) {
      nearest = numbers[i];
      minDifference = currentDifference;
    }
  }
  return nearest;
}
void motors::back() { setback(); }
void motors::right() {
  Serial.println("right");
  targetAngle = normalize360(targetAngle + 90.0f);
  rotate(targetAngle);
}
void motors::left() {
  Serial.println("left");
  targetAngle = normalize360(targetAngle - 90.0f);
  rotate(targetAngle);
}

float motors::calculateAngularDistance() {
  float rightAngularDistance, leftAngularDistance;
  if (targetAngle >= angle) {
    rightAngularDistance = targetAngle - angle;
    leftAngularDistance = angle + (360 - targetAngle);
  } else {
    rightAngularDistance = (360 - angle) + targetAngle;
    leftAngularDistance = angle - targetAngle;
  }
  return (rightAngularDistance <= leftAngularDistance) ? rightAngularDistance
                                                       : -leftAngularDistance;
}
void motors::rotate(float deltaAngle) {
  targetAngle = normalize360(deltaAngle);
  const unsigned long kTurnTimeoutMs = 2800;
  const unsigned long startTime = millis();

  bno.getOrientationX();
  turnPID_.reset(angle, targetAngle, millis());

  while (true) {
    bno.getOrientationX();
    const unsigned long now = millis();

    if (turnPID_.isSettled(targetAngle, angle, now)) {
      break;
    }

    const int16_t turnCommand = turnPID_.update(targetAngle, angle, now);

    if (turnCommand > 0) {
      setright();
    } else if (turnCommand < 0) {
      setleft();
    } else {
      stop();
      continue;
    }

    int turnSpeed = abs(turnCommand);
    PID_AllWheels(turnSpeed);

    if (buttonPressed || (now - startTime) > kTurnTimeoutMs) {
      break;
    }
  }

  stop();
  justRotatedAfterTurn_ = true;
  inMotion = true;
}

float motors::changeSpeedMove(bool encoders, bool rotate, int targetDistance,
                              bool frontVlx) {
  float speed;
  float missingDistance, missingAngle;
  if (rotate == true) {
    missingAngle = abs(targetAngle - (targetAngle == 0 ? z_rotation : angle));
    speed = map(missingAngle, 90, 0, kMaxSpeedRotate, kMinSpeedRotate);
    speed = constrain(speed, kMinSpeedRotate, kMaxSpeedRotate);
    PID_AllWheels(speed);
    return 0;
  } else {
    if (encoders == true) {
      speed =
          map(getAvergeTics(), 0, kTicsPerTile, kMaxPwmFormard, kMinPwmFormard);
      missingDistance =
          kTileLength - (getAvergeTics() * kTileLength / kTicsPerTile);
      return speed;
    } else {
      missingDistance = abs((frontVlx ? vlx[vlxID::frontLeft].getDistance()
                                      : vlx[vlxID::back].getDistance()) -
                            targetDistance); // intercambiar vlx
      speed = frontVlx ? map(missingDistance, kTileLength, 0, kMaxPwmFormard,
                             kMinPwmFormard)
                       : map(missingDistance, kTileLength, 0, kMaxPwmFormard,
                             kMinPwmFormard);
      speed = constrain(speed, kMinPwmFormard, kMaxPwmFormard);
      return speed;
    }
  }
}

void motors::setSpeed(uint16_t speed) {
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].setSpeed(speed);
  }
}
void motors::setahead() {
  for (int i = 0; i < 4; i++) {
    motor[i].ahead();
  }
  inMotion = true;
}
void motors::setback() {
  for (int i = 0; i < 4; i++) {
    motor[i].back();
  }
  // inMotion=true;
}
void motors::setright() {
  motor[MotorID::kBackLeft].ahead();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].ahead();
  motor[MotorID::kFrontRight].back();
  // inMotion=true;
}
void motors::setleft() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].ahead();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].ahead();
  // inMotion=true;
}
void motors::setrightTraslation() {
  motor[MotorID::kBackLeft].stop();
  motor[MotorID::kBackRight].back();
  motor[MotorID::kFrontLeft].stop();
  motor[MotorID::kFrontRight].back();
  // inMotion=true;
}
void motors::setleftTraslation() {
  motor[MotorID::kBackLeft].back();
  motor[MotorID::kBackRight].stop();
  motor[MotorID::kFrontLeft].back();
  motor[MotorID::kFrontRight].stop();
  // inMotion=true;
}

void motors::stop() {
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].stop();
  }
  setSpeed(0);
  inMotion = false;
}
void motors::printSpeeds() {
  float speedM1 = motor[0].getSpeed();
  float speedM2 = motor[1].getSpeed();
  float speedM3 = motor[2].getSpeed();
  float speedM4 = motor[3].getSpeed();
  Serial.println("speed:");
  Serial.print("Motor1:");
  Serial.print(speedM1);
  Serial.print("Motor2:");
  Serial.print(speedM2);
  Serial.print("Motor3:");
  Serial.print(speedM3);
  Serial.print("Motor4:");
  Serial.println(speedM4);
}
void motors::resetTics() {
  for (uint8_t i = 0; i < 4; i++) {
    motor[i].resetTics();
  }
}
double motors::getAvergeTics() {
  float totalTics = 0;
    totalTics += motor[MotorID::kBackLeft].tics;
    totalTics += motor[MotorID::kFrontRight].tics;
  return totalTics / 2;
}
double motors::getTicsSpeed() {
  float ticsSpeed = 0;
  for (int i = 0; i < 4; i++) {
    ticsSpeed += motor[i].getTicsSpeed();
  }
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
  //|setupVlx(vlxID::leftDown);
  // setupVlx(vlxID::front);
  setupVlx(vlxID::frontRight);
  setupVlx(vlxID::right);
  setupVlx(vlxID::back);
}

/*
float motors::AverageLeftDistance(){
    float frontLeftDistance=vlx[vlxID::frontLeft].getDistance();
    float leftDistance=vlx[vlxID::left].getDistance();

    if (frontLeftDistance <= maxVlxDistance && leftDistance <= maxVlxDistance) {
        return (frontLeftDistance+leftDistance)/2;
    } else if (frontLeftDistance > maxVlxDistance && leftDistance <=
maxVlxDistance) { return leftDistance; } else if (frontLeftDistance <=
maxVlxDistance && leftDistance > maxVlxDistance) { return frontLeftDistance;
    }
    else {
        return maxVlxDistance;
    }
}
float motors::AverageRightDistance(){
    float rightDownDistance=vlx[vlxID::rightDown].getDistance();
    float rightDistance=vlx[vlxID::right].getDistance();

    if (rightDownDistance <= maxVlxDistance && rightDistance <= maxVlxDistance)
{ return (rightDownDistance+rightDistance)/2; } else if (rightDownDistance >
maxVlxDistance && rightDistance <= maxVlxDistance) { return rightDistance; }
else if (rightDownDistance <= maxVlxDistance && rightDistance > maxVlxDistance)
{ return rightDownDistance;
    }
    else {
        return maxVlxDistance;
    }
}
*/
bool motors::isWall(uint8_t direction) {
  uint8_t relativeDir = 0;
  int deltaTargetAngle = static_cast<int>(targetAngle);
  switch (deltaTargetAngle) {
  case 0:
  case 360:
    relativeDir = 0;
    break;
  case 90:
    relativeDir = 1;
    break;
  case 180:
    relativeDir = 2;
    break;
  case 270:
    relativeDir = 3;
    break;
  default:
    // Unexpected orientation, keep default (front)
    relativeDir = 0;
    break;
  }
  uint8_t realPos = rulet[relativeDir][direction];
  constexpr float kWallThresholdCm = 22.0f;

  const float frontLeft = stableDistance(vlx[vlxID::frontLeft]);
  const float frontRight = stableDistance(vlx[vlxID::frontRight]);
  const float right = stableDistance(vlx[vlxID::right]);
  const float back = stableDistance(vlx[vlxID::back]);
  const float left = stableDistance(vlx[vlxID::left]);

  switch (realPos) {
  case 0:
    // front wall if any front sensor sees obstacle, using the closer stable read
    return frontLeft < kWallThresholdCm && frontRight < kWallThresholdCm;
  case 1:
    return right < kWallThresholdCm;
  case 2:
    return back < kWallThresholdCm;
  case 3:
    return left < kWallThresholdCm;
  default:
    return false;
  }
}

bool motors::rampInFront() {
  if ((vlx[vlxID::frontLeft].getDistance() -
       vlx[vlxID::frontRight].getDistance()) >= 2) {
    return true;
  } else {
    return false;
  }
}
bool motors::isRamp() {
  float currentOrientationY = bno.getOrientationY();
  if (abs(currentOrientationY) > 7)
    slope = true;
  if (currentOrientationY >= kMinRampOrientation ||
      currentOrientationY <= -kMinRampOrientation) {

    if (currentOrientationY <= -kMinRampOrientation) {
      screenPrint("Ramp detected");
      ramp();
      return true;
    } else if (currentOrientationY > kMinRampOrientation) {
      screenPrint("Ramp detected");
      ramp();
      return true;
    }
  }
  return false;
}
void motors::ramp() {
  resetTics();
  setahead();
  while (bno.getOrientationY() < -7) {
    if (buttonPressed == true)
      break;
    // limitCrash();
    if (limitColition == true) {
      stop();
      break;
    }
    float error;
    vlx[vlxID::right].getDistance();
    vlx[vlxID::left].getDistance();
    if ((vlx[vlxID::right].distance < vlx[vlxID::right].kDistanceToWall &&
         vlx[vlxID::left].distance < vlx[vlxID::left].kDistanceToWall) &&
        (vlx[vlxID::right].distance < 6 || vlx[vlxID::left].distance < 6)) {
      error = rampUpPID.calculate_PID(
          0, (vlx[vlxID::right].distance - vlx[vlxID::left].distance));
      error = constrain(error, -40, 40);
      PID_Wheel(kSpeedRampUp - error, MotorID::kFrontLeft);
      PID_Wheel(kSpeedRampUp - error, MotorID::kBackLeft);
      PID_Wheel(kSpeedRampUp + error, MotorID::kFrontRight);
      PID_Wheel(kSpeedRampUp + error, MotorID::kBackRight);
    } else {
      pidEncoders(kSpeedRampUp, true);
    }
    rampState = 1;
    screenPrint(String(bno.getOrientationY()));
  }
  while (bno.getOrientationY() > 7) {
    if (buttonPressed == true)
      break;
    // limitCrash();
    if (limitColition == true)
      break;
    float error;
    vlx[vlxID::right].getDistance();
    vlx[vlxID::left].getDistance();
    if ((vlx[vlxID::right].distance < vlx[vlxID::right].kDistanceToWall &&
         vlx[vlxID::left].distance < vlx[vlxID::left].kDistanceToWall) &&
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
    // stop();
    // bno.resetOrientationX();
    // setahead();
    moveDistance(kTileLength / 3, true);
    rotate(targetAngle);
  } else if (getAvergeTics() > 0.8 * kTicsPerTile && rampState == 2) {
    // stop();
    // bno.resetOrientationX();
    // setahead();
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

void motors::moveDistance(uint8_t targetDistance, bool ahead) {
  // screenPrint("leaving");
  ahead ? setahead() : setback();
  resetTics();
  while (getCurrentDistanceCm() < targetDistance) {
    if (buttonPressed)
      break;
    pidEncoders((kMinSpeedFormard + kMaxSpeedFormard) / 2, ahead);
  }
  stop();
}

float motors::getCurrentDistanceCm() {
  return getAvergeTics() * kTileLength / kTicsPerTile;
}

float motors::getAngleOrientation() {
  float currentAngle = bno.getOrientationX();
  if ((currentAngle > 315 && currentAngle <= 360) ||
      (currentAngle >= 0 && currentAngle <= 45))
    return 0;
  else if (currentAngle > 45 && currentAngle <= 135)
    return 90;
  else if (currentAngle > 135 && currentAngle <= 225)
    return 180;
  else if (currentAngle > 225 && currentAngle <= 315)
    return 270;
  else
    return currentAngle;
}

void motors::resetOrientation() {
  bno.resetOrientation();
  targetAngle = 0;
}
void motors::checkpointElection() {
  float angleOrientation = getAngleOrientation();
  Serial.println("angle");
  String print;
  print = static_cast<String>(angleOrientation);
  // screenPrint(print);
  Serial.println(angleOrientation);
  uint8_t angleThreshold = 10;
  float currentAngle = (angleOrientation == 0) ? z_rotation : angle;
  if (abs(currentAngle - angleOrientation) < angleThreshold) {
    return;
  }
  rotate(angleOrientation);
  int turn;
  if ((currentAngle - angleOrientation) < -angleThreshold) {
    turn = -1;
    ahead();
    left();
    ahead();
  } else if ((currentAngle - angleOrientation) > angleThreshold) {
    turn = 1;
    ahead();
    right();
    ahead();
  }
  return;
}

void motors::victimSequency(const char *label) {
  float current = millis();
  while ((millis() - current) < 5100) {
    screenPrint(String(label));
    leds.setWhite();
    delay(500);
    leds.turnOff();
    screenPrint(" ");
    delay(500);

  }
  leds.turnOff();
}

void motors::harmedVictim() {
  victimSequency("HARMED");
  if (kitState == kitID::kRight) {
    kitRight(2);
  } else if (kitState == kitID::kLeft) {
    kitLeft(2);
  }
}

void motors::stableVictim() {
  victimSequency("STABLE");
  if (kitState == kitID::kRight) {
    kitRight(1);
  } else if (kitState == kitID::kLeft) {
    kitLeft(1);
  }
}

void motors::unharmedVictim() {
  victimSequency("UNHARMED");
}

void motors::phiVictim() {
  victimSequency("PHI");
  if (kitState == kitID::kRight) {
    kitRight(2);
  } else if (kitState == kitID::kLeft) {
    kitLeft(2);
  }
}

void motors::psiVictim() {
  victimSequency("PSI");
  if (kitState == kitID::kRight) {
    kitRight(1);
  } else if (kitState == kitID::kLeft) {
    kitLeft(1);
  }
}

void motors::omegaVictim() { victimSequency("OMEGA"); }

void motors::kitLeft(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosLeft, servosID::kLeft);
    delay(10);
    writeServo(servoPosRight, servosID::kLeft);
  }
}
void motors::writeServo(uint16_t servoAngle, uint8_t servoID) {
  int currentAngle = servo[servoID].read();
  int targetAngle = servoAngle;
  if (currentAngle < targetAngle) {
    for (int i = currentAngle; i <= targetAngle; i++) {
      servo[servoID].write(i);
      delay(5);
    }
  } else if (currentAngle > targetAngle) {
    for (int i = currentAngle; i >= targetAngle; i--) {
      servo[servoID].write(i);
      delay(5);
    }
  } 
}
void motors::kitRight(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    writeServo(servoPosRight, servosID::kRight);
    delay(50);
    writeServo(servoPosLeft, servosID::kRight);
  }
}
void motors::reloadKits() {
  writeServo(servoPosLeft, servosID::kLeft);
  writeServo(servoPosRight, servosID::kRight);
}

void motors::wait(unsigned long targetTime) {
  unsigned long initialTime = millis();
  while ((millis() - initialTime) < targetTime) {
    if (buttonPressed)
      break;
  }
}
void motors::wifiPrint(String message, float i) {
  // client.print(message);
  // client.println(i);
  // Serial.println("Enviado: ");
}

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
  // screenPrint("Blue Tile");
  Serial.println("Blue Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInBlue = tcs_.red_;
  float greenInBlue = tcs_.green_;
  float blueInBlue = tcs_.blue_;

  // screenPrint("Black Tile");
  Serial.println("Black Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInBlack = tcs_.red_;
  float greenInBlack = tcs_.green_;
  float blueInBlack = tcs_.blue_;

  // screenPrint("Checkpoint Tile");
  Serial.println("Checkpoint Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInCheck = tcs_.red_;
  float greenInCheck = tcs_.green_;
  float blueInCheck = tcs_.blue_;
  float clearInCheck = tcs_.clear_;

  // screenPrint("white Tile");
  Serial.println("White Tile");
  delay(dt);
  tcs_.updateRGBC();
  tcs_.printRGB();
  float redInWhite = tcs_.red_;
  float greenInWhite = tcs_.green_;
  float blueInWhite = tcs_.blue_;

  while (true) {
    if (buttonPressed)
      break;
    String print;
    dt = 2000;
    Serial.print("RGB Blue");
    delay(800);
    print = static_cast<String>(redInBlue) + "," +
            static_cast<String>(greenInBlue) + "," +
            static_cast<String>(blueInBlue);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB Black");
    Serial.print("RGB Black");
    delay(800);
    print = static_cast<String>(redInBlack) + "," +
            static_cast<String>(greenInBlack) + "," +
            static_cast<String>(blueInBlack);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB Check");
    Serial.print("RGB Checkpoint");
    delay(800);
    print = static_cast<String>(redInCheck) + "," +
            static_cast<String>(greenInCheck) + "," +
            static_cast<String>(blueInCheck);
    Serial.println(print);
    delay(dt);

    // screenPrint("RGB white");
    Serial.print("RGB White");
    delay(800);
    print = static_cast<String>(redInWhite) + "," +
            static_cast<String>(greenInWhite) + "," +
            static_cast<String>(blueInWhite);
    Serial.println(print);
    delay(dt);
  }
}
