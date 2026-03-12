#include "TCS.h"

#define DEBUG_TCS 0

TCS::TCS() {
    setDefaultValues();
}

TCS::TCS(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
    setDefaultValues();
}

void TCS::init() {
    mux_.selectChannel();
    Serial.println("Attempting to initialize TCS...");
    if (!tcs_.begin()) {
        Serial.println("ERROR: No TCS34725 found ... check your connections");
        while (1) {
#if DEBUG_TCS
            Serial.println("No TCS34725 found ... check your connections");
#endif
        }
    }
    Serial.println("TCS inicializado");
}

void TCS::setDefaultValues() {
    red_ = 0;
    green_ = 0;
    blue_ = 0;
    clear_ = 0;
}

void TCS::updateRGB() {
    mux_.selectChannel();
    tcs_.setInterrupt(false);
    delay(millisToWait_);
    tcs_.getRGB(&red_, &green_, &blue_);
    tcs_.setInterrupt(true);
}

void TCS::updateRGBC() {
    uint16_t redR;
    uint16_t greenR;
    uint16_t blueR;
    uint16_t clearR;
    mux_.selectChannel();
    tcs_.setInterrupt(false);
    delay(millisToWait_);
    tcs_.getRawData(&redR, &greenR, &blueR, &clearR);
    red_ = redR;
    green_ = greenR;
    blue_ = blueR;
    clear_ = clearR;
    tcs_.setInterrupt(true);
}

void TCS::printRGB() {
    updateRGBC();
    Serial.println(red_);
    Serial.println(green_);
    Serial.println(blue_);
}

void TCS::printRGBC() {
    const unsigned long t = millis();
    updateRGBC();
#if DEBUG_TCS
    Serial.print("Time:\t");
    Serial.println(millis() - t);
    Serial.print("R:\t");
    Serial.println(red_);
    Serial.print("G:\t");
    Serial.println(green_);
    Serial.print("B:\t");
    Serial.println(blue_);
    Serial.print("C:\t");
    Serial.println(clear_);
#else
    (void)t;
#endif
}

void TCS::setMux(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void TCS::printColor() {
    Serial.println(getColor());
}

char TCS::getColor() {
    updateRGBC();
    char colorLetter = kUndefinedColor_;

    if (red_ > kMinRedValueInBlue_ && green_ > kMinGreenValueInBlue_ && blue_ > kMinBlueValueInBlue_ &&
        red_ < kMaxRedValueInBlue_ && green_ < kMaxGreenValueInBlue_ && blue_ < kMaxBlueValueInBlue_) {
        colorLetter = kBlueColor_;
#if DEBUG_TCS
        Serial.println("blue");
#endif
    } else if (red_ > kMinRedValueInBlack_ && green_ > kMinGreenValueInBlack_ && blue_ > kMinBlueValueInBlack_ &&
               red_ < kMaxRedValueInBlack_ && green_ < kMaxGreenValueInBlack_ && blue_ < kMaxBlueValueInBlack_) {
        colorLetter = kBlackColor_;
#if DEBUG_TCS
        Serial.println("black");
#endif
    } else if (red_ > kMinRedValueInCheckpoint_ && green_ > kMinGreenValueInCheckpoint_ && blue_ > kMinBlueValueInCheckpoint_ &&
               red_ < kMaxRedValueInCheckpoint_ && green_ < kMaxGreenValueInCheckpoint_ && blue_ < kMaxBlueValueInCheckpoint_) {
        colorLetter = kCheckpointColor_;
#if DEBUG_TCS
        Serial.println("checkpoint");
#endif
    } else {
        colorLetter = kUndefinedColor_;
#if DEBUG_TCS
        Serial.println("unknown");
#endif
    }

#if DEBUG_TCS
    Serial.print("colorLetter: ");
    Serial.println(colorLetter);
#endif
    return colorLetter;
}
