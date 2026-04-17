#include "LimitSwitch.h"
#define DEBUG_LIMIT_SWITCH 0

LimitSwitch::LimitSwitch() {
    this->state_ = false;
    this->pin_ = 0;
}

void LimitSwitch::initLimitSwitch(const uint8_t pin) {
    id_ = pin;
    pin_ = pin;
    initLimitSwitchInternal();
}

void LimitSwitch::initLimitSwitchInternal() {
    pinMode(pin_, INPUT_PULLDOWN);
    Serial.println("LimitSwitch initialized");
}

bool LimitSwitch::getState() {
    const uint8_t val = digitalRead(pin_);
    delayMicroseconds(1000); // 1ms (1000
    const uint8_t currentVal = digitalRead(pin_);
    if (val == LOW) {
        state_ = true;
        Serial.println("LimitSwitch is active");
    } else {
        state_ = false;
    }
    return state_;
}

void LimitSwitch::LimitSwitchActive() {
    volatile bool val = digitalRead(pin_);
    if (state_ == LOW) {
        state_ = true;
        Serial.println("LimitSwitch is active");
    } else {
        state_ = false;
        Serial.println("LimitSwitch is not active");
    }
}

void LimitSwitch::printState() {
    Serial.print("LimitSwitch ");
    Serial.print(static_cast<int>(id_));
    Serial.print(" state: ");
    Serial.println(state_);
}