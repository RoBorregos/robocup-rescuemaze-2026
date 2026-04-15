#include "BNO.H"
float z_rotation;
float angle;

static float normalizeAngle360(float angle) {
  if (angle < 0) {
    angle += 360.0f;
  } else if (angle >= 360.0f) {
    angle -= 360.0f;
  }
  return angle;
}

static float filterHeading(float raw, float &filtered, float alpha) {
  float delta = raw - filtered;
  if (delta > 180.0f)
    delta -= 360.0f;
  else if (delta < -180.0f)
    delta += 360.0f;

  filtered += alpha * delta;
  if (filtered < 0.0f)
    filtered += 360.0f;
  else if (filtered >= 360.0f)
    filtered -= 360.0f;
  return filtered;
}

static float filterValue(float raw, float &filtered, float alpha) {
  filtered = alpha * raw + (1.0f - alpha) * filtered;
  return filtered;
}

BNO::BNO() {
  this->event_ = {0};
  this->bno_ = Adafruit_BNO055(sensorID, I2CAddress, &Wire);
}
void BNO::setupBNO() {
  adafruit_bno055_opmode_t mode = OPERATION_MODE_IMUPLUS;

  if (!bno_.begin()) {
    Serial.println("Error initializing BNO055! Check your connections.");
    while (1)
      ;
  }
  delay(1000);
  bno_.setExtCrystalUse(true);
  updateBNO(event_);
  filteredAngleX_ = normalizeAngle360(event_.orientation.x);
  filteredAngleY_ = event_.orientation.z;
  Serial.println("BNO055 initialized successfully");
}

void BNO::updateBNO(sensors_event_t &event) { bno_.getEvent(&event); }

float BNO::getOrientationX() {
  updateBNO(event_);
  float rawAngle = event_.orientation.x - phaseCorrection_;
  rawAngle = normalizeAngle360(rawAngle);
  if (filteredAngleX_ < 0.0f || filteredAngleX_ >= 360.0f) {
    filteredAngleX_ = rawAngle;
  }
  angle = filterHeading(rawAngle, filteredAngleX_, orientationFilterAlpha_);
  z_rotation = (angle > 180.0f) ? angle - 360.0f : angle;
  return angle;
}

float BNO::getOrientationY() {
  updateBNO(event_);
  float rawY = event_.orientation.z - phaseCorrectionY_;
  return filterValue(rawY, filteredAngleY_, orientationFilterAlpha_);
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
  setPhaseCorrectionY(event_.orientation.z);
  filteredAngleX_ = normalizeAngle360(event_.orientation.x - phaseCorrection_);
  filteredAngleY_ = event_.orientation.z - phaseCorrectionY_;
  bno_.begin();
  delay(10);
  bno_.setExtCrystalUse(true);
  Serial.println("Bno values set to 0 for X and Y axis.");
}

void BNO::resetOrientationX() {
  updateBNO(event_);
  setPhaseCorrection(event_.orientation.x); // Reset the X axis
  filteredAngleX_ = normalizeAngle360(event_.orientation.x - phaseCorrection_);
  bno_.begin();
  delay(10);
  bno_.setExtCrystalUse(true);
  Serial.println("Bno values set to 0 for X axis.");
}