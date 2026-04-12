#include "TurnPID.h"

float TurnPID::normalize360(float angle) {
  while (angle >= 360.0f) {
    angle -= 360.0f;
  }
  while (angle < 0.0f) {
    angle += 360.0f;
  }
  return angle;
}

float TurnPID::signedError(float target, float current) {
  float error = normalize360(target) - normalize360(current);
  if (error > 180.0f) {
    error -= 360.0f;
  } else if (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

void TurnPID::configure(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void TurnPID::setOutputLimits(uint8_t minCommand, uint8_t maxCommand) {
  minCommand_ = minCommand;
  maxCommand_ = maxCommand;
}

void TurnPID::setSettleCriteria(float toleranceDeg, uint16_t settleTimeMs) {
  settleToleranceDeg_ = toleranceDeg;
  settleTimeMs_ = settleTimeMs;
}

void TurnPID::reset(float currentHeading, float targetHeading, uint32_t nowMs) {
  integral_ = 0.0f;
  lastError_ = signedError(targetHeading, currentHeading);
  lastTimeMs_ = nowMs;
  inToleranceSinceMs_ = 0;
}

int16_t TurnPID::update(float targetHeading, float currentHeading, uint32_t nowMs) {
  const float error = signedError(targetHeading, currentHeading);

  uint32_t dtMs = nowMs - lastTimeMs_;
  if (lastTimeMs_ == 0 || dtMs == 0) {
    lastTimeMs_ = nowMs;
    lastError_ = error;
    return 0;
  }

  const float dt = static_cast<float>(dtMs) / 1000.0f;
  integral_ += error * dt;
  integral_ = constrain(integral_, -maxIntegral_, maxIntegral_);

  const float derivative = (error - lastError_) / dt;
  float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

  lastError_ = error;
  lastTimeMs_ = nowMs;

  if (abs(error) <= settleToleranceDeg_) {
    integral_ *= 0.8f;
    return 0;
  }

  output = constrain(output, -static_cast<float>(maxCommand_),
                     static_cast<float>(maxCommand_));

  if (output > 0.0f && output < minCommand_) {
    output = minCommand_;
  } else if (output < 0.0f && output > -static_cast<float>(minCommand_)) {
    output = -static_cast<float>(minCommand_);
  }

  return static_cast<int16_t>(output);
}

bool TurnPID::isSettled(float targetHeading, float currentHeading, uint32_t nowMs) {
  const float error = abs(signedError(targetHeading, currentHeading));

  if (error <= settleToleranceDeg_) {
    if (inToleranceSinceMs_ == 0) {
      inToleranceSinceMs_ = nowMs;
    }
    return (nowMs - inToleranceSinceMs_) >= settleTimeMs_;
  }

  inToleranceSinceMs_ = 0;
  return false;
}
