#ifndef VLX_H
#define VLX_H
#include "MUX.h"
#include "Pins_ID.h"
#include "SingleEMAFilter.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

class VLX {
private:
  MUX mux_;
  static constexpr uint8_t kMaxInitAttempts_ = 5;
  // High-precision profile: larger timing budget and larger smoothing window.
  static constexpr uint32_t kTimingBudget = 50000;
  static constexpr uint8_t kFilterSize = 5;
  static constexpr uint8_t kTrimSamples = 1;
  static constexpr float kOutOfRangeCm = 819.0f;
  static constexpr uint8_t kMaxInvalidReads = 2;
  float filter_[kFilterSize];
  uint8_t filterIdx_ = 0;
  uint8_t invalidReads_ = 0;

public:
  static constexpr uint8_t kDistanceToWall = 15;
  bool initialized = false;
  float distance = 0.0f;
  Adafruit_VL53L0X VLX_ = Adafruit_VL53L0X();
  VL53L0X_RangingMeasurementData_t measure;
  void updateDistance();
  float getDistance();
  void printDistance();
  VLX();
  VLX(int, int);
  bool isWall();
  VLX(const uint8_t);
  void setMux(const uint8_t);
  void begin();
};
#endif
