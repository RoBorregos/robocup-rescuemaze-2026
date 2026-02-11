#ifndef VLX_H
#define VLX_H
#include "Pins_ID.h"
#include <Adafruit_VL53L0X.h>
#include <Adafruit_Sensor.h>
#include "SingleEMAFilter.h"
#include <Wire.h>
#include <Arduino.h>
#include "MUX.h"

class VLX{
private:
    MUX mux_;
    static constexpr uint8_t kMaxInitAttempts_ = 5;
    static constexpr uint32_t kTimingBudget = 33000;
    static constexpr uint32_t vDelay = 33;

public:
    static constexpr uint8_t kDistanceToWall=15;
    float distance = 0.0f;
    bool initialized = false;
    Adafruit_VL53L0X VLX_ = Adafruit_VL53L0X();
    VL53L0X_RangingMeasurementData_t measure;
    void updateDistance();
    float getDistance();
    VLX();
    VLX(int,int);
    bool isWall();
    VLX(const uint8_t);
    void setMux(const uint8_t);
    void begin();
};
#endif
