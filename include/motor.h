#ifndef MOTOR_H
#define MOTOR_H
#include "Pins_ID.h"
#include <Arduino.h>

class Motor {
private:
  uint8_t in1 = 0;
  uint8_t in2 = 0;
  uint8_t enable = 0;
  uint16_t speed = 0;
  int ticsSpeed = 0;
  unsigned long last_time = 0;

public:
  volatile int tics = 0;
  volatile int deltaTics = 0;
  void initialize(uint8_t, uint8_t, uint8_t, uint8_t);
  Motor();
  void updateTics();
  void setSpeed(uint16_t);
  void ahead();
  void back();
  void stop();
  double getSpeed();
  void resetTics();
  int getTics();
  int getTicsSpeed();
};
#endif
