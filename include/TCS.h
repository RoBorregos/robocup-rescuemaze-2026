#ifndef TCS_H
#define TCS_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include "MUX.h"

class TCS {
public:
    float red_ = 0.0f;
    float green_ = 0.0f;
    float blue_ = 0.0f;
    float clear_ = 0.0f;

    TCS();

    void setMux(const uint8_t pos);
    bool init();
    void updateRGBC();
    void printRGB();
    char getColor();

private:
    MUX mux_;
    uint8_t muxPos_ = 0;
    bool initialized_ = false;
    Adafruit_TCS34725 tcs_;
};

#endif
