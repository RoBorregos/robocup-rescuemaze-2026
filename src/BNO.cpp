#include "BNO.H"
float z_rotation;
float angle;

BNO::BNO() {
    this->event_ = {0};
    this->bno_ = Adafruit_BNO055(sensorID, I2CAddress, &Wire);
}
void BNO::setupBNO() {
    adafruit_bno055_opmode_t mode = OPERATION_MODE_IMUPLUS;

    if (!bno_.begin()){
        Serial.println("Error initializing BNO055! Check your connections.");
        while(1);
    }
    delay(1000);
    bno_.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully");

}

void BNO::updateBNO(sensors_event_t &event) {
    bno_.getEvent(&event);
}

float BNO::getOrientationX() {
    updateBNO(event_);
    angle=event_.orientation.x - phaseCorrection_;
    if (angle < 0) {
        angle+= 360;
    } else if (angle >= 360) {
        angle-= 360;
    }
    if(angle>180){
        z_rotation=angle-360.0;
    }else{
        z_rotation=angle;
    }
    return angle;
}

float BNO::getOrientationY() {
    updateBNO(event_);
    return event_.orientation.y-phaseCorrectionY_;
}

void BNO::setPhaseCorrection(const float phaseCorrection) {
    phaseCorrection_ = phaseCorrection;
}

void BNO::setPhaseCorrectionY(float phaseCorrectionY) {
    phaseCorrectionY_ = phaseCorrectionY;
}


void BNO::resetOrientation() {
    updateBNO(event_);
    setPhaseCorrection(event_.orientation.x);
    setPhaseCorrectionY(event_.orientation.y);
    bno_.begin();
    delay(10);
    bno_.setExtCrystalUse(true);
    Serial.println("Bno values set to 0 for X and Y axis.");
}

void BNO::resetOrientationX() {
    updateBNO(event_);
    setPhaseCorrection(event_.orientation.x); // Reset the X axis
    bno_.begin();
    delay(10);
    bno_.setExtCrystalUse(true);
    Serial.println("Bno values set to 0 for X axis.");
}