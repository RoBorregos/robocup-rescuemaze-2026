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
    static constexpr uint8_t kMaxInitAttempts_ = 5;
    float lastDistance = 0.0f;
    SemaphoreHandle_t i2cSemaphore;
    static MUX mux;
    static constexpr uint32_t kTimingBudget = 33000;
    static constexpr uint32_t vDelay = 20;

public:
    static constexpr uint8_t kDistanceToWall=15;
    Adafruit_VL53L0X VLX_ = Adafruit_VL53L0X();
    VL53L0X_RangingMeasurementData_t measure;
    float getDistance();
    VLX();
    VLX(const uint8_t);
    void begin();
    VLX(int,int);
    bool isWall();
};
#endif
