#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

VLX::VLX() {}

VLX::VLX(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::begin() {
    initialized = false;
    mux_.selectChannel();
    if (!VLX_.begin()) {
        Serial.println("Error initializing VL53L0X sensor.");
        Serial.println(MUX_ADDR);
        while (1);
    }
    Serial.println("VL53L0X initialized correctly.");
    VLX_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget);
    VLX_.startRangeContinuous();
    initialized = true;
}

void VLX::setMux(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::updateDistance() {
    mux_.selectChannel();

    if (VLX_.isRangeComplete()) {
        distance = (float)(VLX_.readRange()) / 10.0f;
        }
}

float VLX::getDistance() {
    if (initialized) {
        return distance;
    }
    else {
        return -1.0f;
    }
}

bool VLX::isWall() {
    return (getDistance() < kDistanceToWall);
}
