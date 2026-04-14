#include "Encoder.h"
#include "Test.h"
#include "maze.h"
#include "raspy.h"
#include <Arduino.h>
maze m;

void setup() {
  Serial.begin(115200);
  robot.setupMotors();

  robot.screenBegin();
  robot.screenPrint("INIT");
  Serial.println("=== RoboCup Rescue Maze ===");
  Serial.println("Connecting to Raspberry Pi...");
  //raspy.connect();
  robot.screenPrint("READY");
  Serial.println("Connected. getDetection() ready.");

  pinMode(Pins::checkpointPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]),
                  Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]),
                  Interrups::backLeftEncoder, RISING);

}

void loop() { m.run_algs(); }
