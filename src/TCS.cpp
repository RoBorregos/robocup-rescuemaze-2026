#include "TCS.h"

TCS::TCS()
    : tcs_(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X) {
}

void TCS::setMux(const uint8_t pos) {
    muxPos_ = pos;
    mux_.setNewChannel(pos);
}

bool TCS::init() {
    mux_.selectChannel(muxPos_);
    initialized_ = tcs_.begin();
    return initialized_;
}

void TCS::updateRGBC() {
    if (!initialized_) {
        return;
    }

    mux_.selectChannel(muxPos_);
    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;
    uint16_t c = 0;
    tcs_.getRawData(&r, &g, &b, &c);
    red_ = static_cast<float>(r);
    green_ = static_cast<float>(g);
    blue_ = static_cast<float>(b);
    clear_ = static_cast<float>(c);
}

void TCS::printRGB() {
    Serial.print("R:");
    Serial.print(red_);
    Serial.print(" G:");
    Serial.print(green_);
    Serial.print(" B:");
    Serial.print(blue_);
    Serial.print(" C:");
    Serial.println(clear_);
}

char TCS::getColor() {
    if (!initialized_) {
        return 'U';
    }

    updateRGBC();

    if (clear_ < 80.0f) {
        return 'N';
    }

    const float sum = red_ + green_ + blue_ + 1.0f;
    const float r = red_ / sum;
    const float g = green_ / sum;
    const float b = blue_ / sum;

    if (b > 0.45f) {
        return 'B';
    }
    if (r > 0.45f) {
        return 'R';
    }
    if (clear_ > 800.0f) {
        return 'C';
    }

    return 'W';
}
