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
  // put your setup code here, to run once:
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
  
  // // robot.leds.sequency();

  // robot.kitLeft(1);
  // delay(1000);
  // robot.kitRight(1);
  // delay(1000);

  // robot.ahead();
  // testTCS();
  // robot.setahead();
  // robot.setSpeed(50);

  // jeetson.getDetection();
  // delay(300);
  // testButton();
  // robot.ahead();
  // testMotors();
  // testPIDWheel();

  // pidTest();
  // calibrateColors();
  // robot.checkpointElection(); 
  // robot.buttonPressed=false;
  // testTCS();
  // testLimits();
  // testBnoY();

  // testMotors();
  // testEncoders();
  // testVlxFrontDistance();
  // testVlxFrontLeft();
  // testVlxFrontRigth();
  // testVlxRight();
  // testVlxLeft();
  // testVlxFront();
  // testVlxBack();
}
