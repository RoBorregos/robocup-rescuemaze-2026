#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

VLX::VLX() {
  for (uint8_t i = 0; i < kFilterSize; ++i) {
    filter_[i] = kOutOfRangeCm;
  }
  filterIdx_ = 0;
}

VLX::VLX(const uint8_t posMux) {
  mux_.setNewChannel(posMux);
  for (uint8_t i = 0; i < kFilterSize; ++i) {
    filter_[i] = kOutOfRangeCm;
  }
  filterIdx_ = 0;
}

void VLX::begin() {
  mux_.selectChannel();
  VLX_.begin();
  if (!VLX_.begin()) {
    Serial.println("Error on sensor VL53L0X!");
    while (1);
  }
  VLX_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget);
  VLX_.startRangeContinuous(100);
  Serial.println("VL53L0X initialized properly.");
}

void VLX::setMux(const uint8_t posMux) {
  mux_.setNewChannel(posMux);
}

float VLX::getDistance() {
  mux_.selectChannel();
  uint16_t rawRange = VLX_.readRange();
  uint16_t status   = VLX_.readRangeStatus();

  bool timeout = VLX_.timeoutOccurred();
  bool valid = (!timeout && status != 4);
  float newReading = valid ? ((float)(rawRange) / 10.0f) : kOutOfRangeCm;

  if (!valid) {
    // segundo intento si el primero falla
    rawRange = VLX_.readRange();
    status   = VLX_.readRangeStatus();
    timeout  = VLX_.timeoutOccurred();
    valid    = (!timeout && status != 4);
    if (valid) {
      newReading = (float)(rawRange) / 10.0f;
    }
  }

  // High-precision filter: ring buffer + trimmed mean to reject spikes.
  if (newReading < kOutOfRangeCm) {
    invalidReads_ = 0;
    filter_[filterIdx_] = newReading;
    filterIdx_ = (filterIdx_ + 1) % kFilterSize;

    float valid[kFilterSize];
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < kFilterSize; ++i) {
      if (filter_[i] < kOutOfRangeCm) {
        valid[validCount++] = filter_[i];
      }
    }

    if (validCount == 0) {
      distance = newReading;
      return distance;
    }

    // Insertion sort is fine here because validCount is small (<= 9).
    for (uint8_t i = 1; i < validCount; ++i) {
      float key = valid[i];
      int j = i - 1;
      while (j >= 0 && valid[j] > key) {
        valid[j + 1] = valid[j];
        --j;
      }
      valid[j + 1] = key;
    }

    uint8_t trim = (validCount > (2 * kTrimSamples + 2)) ? kTrimSamples : 0;
    uint8_t start = trim;
    uint8_t endExclusive = validCount - trim;
    float sum = 0.0f;
    uint8_t used = 0;
    for (uint8_t i = start; i < endExclusive; ++i) {
      sum += valid[i];
      ++used;
    }
    distance = (used > 0) ? (sum / used) : valid[validCount / 2];
  } else {
    // En pasillos libres no queremos quedarnos pegados al ultimo valor valido.
    ++invalidReads_;
    if (invalidReads_ >= kMaxInvalidReads) {
      distance = kOutOfRangeCm;
      for (uint8_t i = 0; i < kFilterSize; ++i) {
        filter_[i] = kOutOfRangeCm;
      }
      filterIdx_ = 0;
    }
  }

  return distance;
}

void VLX::printDistance() {
  float d = getDistance();
  if (d < kOutOfRangeCm) {
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range.");
  }
}

bool VLX::isWall() {
  return (getDistance() < kDistanceToWall);
}