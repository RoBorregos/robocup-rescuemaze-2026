#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

VLX::VLX() {}

VLX::VLX(const uint8_t posMux) { mux_.setNewChannel(posMux); }

void VLX::begin() {
  mux_.selectChannel();
  VLX_.begin();
  if (!VLX_.begin()) {
    Serial.println("Error on sensor VL53L0X!");
    while (1)
      ;
  }
  VLX_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget);
  VLX_.startRangeContinuous(100);
  hasFilteredDistance_ = false;
  Serial.println("VL53L0X inicilaized properly.");
}

void VLX::setMux(const uint8_t posMux) { mux_.setNewChannel(posMux); }

float VLX::getDistance() {
  mux_.selectChannel();

  uint16_t rawRange = VLX_.readRange();
  uint16_t status = VLX_.readRangeStatus();
  if (VLX_.timeoutOccurred()) {
    // preserve previous valid distance if timeout
    return distance;
  }

  if (status != 4) {
    const float distanceCm = static_cast<float>(rawRange) / 10.0f;
    if (!hasFilteredDistance_) {
      distance = distanceCm;
      hasFilteredDistance_ = true;
    } else {
      distance = distanceFilter_.addValue(distanceCm);
    }
    return distance;
  } else {
    rawRange = VLX_.readRange();
    status = VLX_.readRangeStatus();
    if (status != 4) {
      const float distanceCm = static_cast<float>(rawRange) / 10.0f;
      if (!hasFilteredDistance_) {
        distance = distanceCm;
        hasFilteredDistance_ = true;
      } else {
        distance = distanceFilter_.addValue(distanceCm);
      }
      return distance;
    }
  }
  return hasFilteredDistance_ ? distance : 819.0f;
}

void VLX::printDistance() {
  float printedDistance = getDistance();
  if (printedDistance != 819.0) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range.");
  }
}

bool VLX::isWall() { return (getDistance() < kDistanceToWall); }
