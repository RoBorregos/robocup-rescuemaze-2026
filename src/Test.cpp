#include "Test.h"
#include "Encoder.h"
#include "Pins_ID.h"

void testEncoder(uint8_t id) {
  robot.motor[id].ahead();
  robot.motor[id].setSpeed(100);
  Serial.println(robot.motor[id].tics);
}
void testEncoders() {
  Serial.println("kBackRight Encoder Tics:");
  Serial.println(robot.motor[MotorID::kBackRight].tics);
  delay(1000);
  Serial.println("kBackLeft Encoder Tics:");
  Serial.println(robot.motor[MotorID::kBackLeft].tics);
  delay(1000);
  Serial.println("kFrontRight Encoder Tics:");
  Serial.println(robot.motor[MotorID::kFrontRight].tics);
  delay(1000);
  Serial.println("kFrontLeft Encoder Tics:");
  Serial.println(robot.motor[MotorID::kFrontLeft].tics);
  delay(1000);
}
void testButton() {
  String print = static_cast<String>(robot.buttonPressed);
  // robot.screenPrint(print);
  Serial.println(print);
}
void testVlx() { robot.screenPrint("FL: " + String(robot.vlx[vlxID::frontLeft].getDistance()) + " FR: " + String(robot.vlx[vlxID::frontRight].getDistance()) + " B: " + String(robot.vlx[vlxID::back].getDistance()));
  delay(1000);
  robot.screenPrint("L: " + String(robot.vlx[vlxID::left].getDistance()) + " R: " + String(robot.vlx[vlxID::right].getDistance()));
  delay(1000);
  robot.screenPrint("FL: " + String(robot.vlx[vlxID::frontLeft].getDistance()) + " FR: " + String(robot.vlx[vlxID::frontRight].getDistance()));
  delay(1000);
}
void testVlxFrontLeft() {
  float distance = robot.vlx[vlxID::frontLeft].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxFrontRigth() {
  float distance = robot.vlx[vlxID::frontRight].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxRight() {
  float distance = robot.vlx[vlxID::right].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxLeft() {
  float distance = robot.vlx[vlxID::left].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxrightDown() {
  float distance = robot.vlx[vlxID::right].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxFront() {
  float distance = robot.vlx[vlxID::frontRight].getDistance();
  Serial.println(distance);
  String print = static_cast<String>(distance);
  // robot.screenPrint(print);
}
void testVlxFrontDistance() {
  robot.vlx[vlxID::frontLeft].getDistance();
  robot.vlx[vlxID::frontRight].getDistance();
  float frontDistance = (robot.vlx[vlxID::frontLeft].distance >
                         robot.vlx[vlxID::frontRight].distance)
                            ? robot.vlx[vlxID::frontLeft].distance
                            : robot.vlx[vlxID::frontRight].distance;
  String print = static_cast<String>(frontDistance);
  // robot.screenPrint(print);
  Serial.println(frontDistance);
}
void testMotors() {
  int dt = 1000;
  robot.setahead();
  delay(dt);
  robot.screenPrint("BackRight");
  robot.motor[MotorID::kBackRight].setSpeed(200);
  delay(dt);
  robot.screenPrint("FrontRight");
  robot.motor[MotorID::kFrontRight].setSpeed(200);
  delay(dt);
  robot.screenPrint("BackLeft");
  robot.motor[MotorID::kBackLeft].setSpeed(200);
  delay(dt);
  robot.screenPrint("FrontLeft");
  robot.motor[MotorID::kFrontLeft].setSpeed(200);
  delay(dt);
  // robot.setSpeed(0);
  while (1)
  {
    delay(1000);
    robot.screenPrint("L: " + String(robot.vlx[vlxID::left].getDistance()) + " R: " + String(robot.vlx[vlxID::right].getDistance()));
    delay(1000);
    robot.screenPrint("FL: " + String(robot.vlx[vlxID::frontLeft].getDistance()) + " FR: " + String(robot.vlx[vlxID::frontRight].getDistance()));
    delay(1000);
  };
}

void testPIDWheel() {
  robot.setahead();
  for (int i = 0; i < 4; i++) {
    robot.PID_Wheel(50, i);
    delay(1000);
    robot.PID_Wheel(255, i);
    delay(1000);
    robot.motor[i].setSpeed(0);
  }
}

void testBnoY() {
  String print = static_cast<String>(robot.bno.getOrientationY());
  // robot.screenPrint(print);
  Serial.println(print);
}

void calibrateColors() { robot.calibrateColors(); }
void pidTest() {
  robot.setahead();
  robot.pidEncoders(20, true);
  robot.PID_Wheel(4, MotorID::kFrontLeft);
}

void testTCS() {
  robot.tcs_.updateRGBC();
  robot.tcs_.printRGB();
  char color = robot.tcs_.getColor();
  String print = static_cast<String>(color);
  // robot.screenPrint(print);
  Serial.println(print);
}

void testVictimSequenceWithLeds() {
  Serial.println("Test HARMED victim with LEDs");
  robot.kitState = kitID::kRight;
  robot.harmedVictim();
  delay(1000);

  Serial.println("Test STABLE victim with LEDs");
  robot.kitState = kitID::kLeft;
  robot.stableVictim();
  delay(1000);

  Serial.println("Test UNHARMED victim with LEDs");
  robot.unharmedVictim();
  delay(1000);
}

void testKitDropWithLeds() {
  Serial.println("Test kit drop right");
  robot.kitState = kitID::kRight;
  robot.harmedVictim();
  delay(1000);

  Serial.println("Test kit drop left");
  robot.kitState = kitID::kLeft;
  robot.stableVictim();
  delay(1000);
}

void testTurn(float angle) { robot.rotate(angle); }
