#include <Arduino.h>
#include "Encoder.h"
#include "maze.h"
#include "Test.h"
//#include <FastLED.h>
//#include "Jetson.h"
maze m;
//Jetson jeetson;
int servopos=0;
bool mazeStarted = false;
void setup() {

  pinMode(2,OUTPUT);
  digitalWrite(2,1);
  Serial.begin(115200);
  Serial.println("Setup starting...");
  robot.setupMotors();
  Serial.println("Motors setup done");
  pinMode(Pins::checkpointPin,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), Interrups::backRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::checkpointPin),Interrups::lackOfProgress, RISING);
  Serial.println("Setup complete!");
  // robot.reloadKits();
}

void loop() {
  Serial.println("Loop starting...");
  
  if(!mazeStarted) {
    Serial.println("Starting maze exploration...");
    mazeStarted = true;
    m.run_algs();
    Serial.println("Maze exploration complete!");
  }
  
  delay(1000);
  Serial.println("Loop iteration complete");
}
