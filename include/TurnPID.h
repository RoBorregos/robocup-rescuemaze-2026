#ifndef TURN_PID_H
#define TURN_PID_H

#include <Arduino.h>

class TurnPID {
private:
  float kp_ = 2.2f;
  float ki_ = 0.0f;
  float kd_ = 0.2f;
  float integral_ = 0.0f;
  float lastError_ = 0.0f;
  uint32_t lastTimeMs_ = 0;
  uint8_t minCommand_ = 30;
  uint8_t maxCommand_ = 120;
  float maxIntegral_ = 30.0f;
  float settleToleranceDeg_ = 1.2f;
  uint16_t settleTimeMs_ = 120;
  uint32_t inToleranceSinceMs_ = 0;

  static float normalize360(float angle);
  static float signedError(float target, float current);

public:
  void configure(float kp, float ki, float kd);
  void setOutputLimits(uint8_t minCommand, uint8_t maxCommand);
  void setSettleCriteria(float toleranceDeg, uint16_t settleTimeMs);
  void reset(float currentHeading, float targetHeading, uint32_t nowMs);
  int16_t update(float targetHeading, float currentHeading, uint32_t nowMs);
  bool isSettled(float targetHeading, float currentHeading, uint32_t nowMs);
};

#endif
