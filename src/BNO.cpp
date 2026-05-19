#include "BNO.H"

#define DEBUG_BNO

float z_rotation;
float angle;

BNO::BNO() {
  this->event_ = {0};
  this->bno_   = Adafruit_BNO055(sensorID, I2CAddress, &Wire);
}

void BNO::setupBNO() {
  if (!bno_.begin()) {
    Serial.println("Error initializing BNO055! Check your connections.");
    while (1);
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
  float raw = event_.orientation.x;
  angle = raw - phaseCorrection_;
  if (angle < 0)      angle += 360;
  else if (angle >= 360) angle -= 360;
  z_rotation = (angle > 180) ? (angle - 360.0f) : angle;
#ifdef DEBUG_BNO
  Serial.print("BNO raw: "); Serial.print(raw);
  Serial.print(" phase: "); Serial.print(phaseCorrection_);
  Serial.print(" angle: "); Serial.print(angle);
  Serial.print(" z: "); Serial.println(z_rotation);
#endif
  return angle;
}

float BNO::getOrientationY() {
  // FIX 7: una sola lectura del sensor por llamada
  // Antes llamaba updateBNO dos veces (una aquí y otra heredada del patrón),
  // lo que causaba dos lecturas I2C en el mismo ciclo con valores distintos.
  updateBNO(event_);
  return event_.orientation.z - phaseCorrectionY_;
}

void BNO::setPhaseCorrection(const float phaseCorrection) {
  phaseCorrection_ = phaseCorrection;
}

void BNO::setPhaseCorrectionY(float phaseCorrectionY) {
  phaseCorrectionY_ = phaseCorrectionY;
}

void BNO::resetOrientation() {
  delay(100);
  updateBNO(event_);
  setPhaseCorrection(event_.orientation.x);
  setPhaseCorrectionY(event_.orientation.z);
#ifdef DEBUG_BNO
  Serial.print("resetOrientation rawX: "); Serial.print(event_.orientation.x);
  Serial.print(" rawZ: "); Serial.print(event_.orientation.z);
  Serial.print(" phaseX: "); Serial.print(phaseCorrection_);
  Serial.print(" phaseZ: "); Serial.println(phaseCorrectionY_);
#endif
  Serial.println("Bno values set to 0 for X and z axis.");
}

void BNO::resetOrientationX() {
  delay(100);
  updateBNO(event_);
  setPhaseCorrection(event_.orientation.x);
#ifdef DEBUG_BNO
  Serial.print("resetOrientationX rawX: "); Serial.print(event_.orientation.x);
  Serial.print(" phaseX: "); Serial.println(phaseCorrection_);
#endif
  Serial.println("Bno values set to 0 for X axis.");
}