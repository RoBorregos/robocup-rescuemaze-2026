#include <Arduino.h>
#include "Encoder.h"
#include "maze.h"
#include "Test.h"

maze m;
constexpr bool kSerialScreenTestMode = true;
String incomingLine;

int servopos=0;
void setup() {
  Serial.begin(115200);
  robot.setupMotors();
  if (kSerialScreenTestMode) {
    robot.screenBegin();
    robot.screenPrint("READY");
    Serial.println("Serial OLED test mode enabled. Send text ending with \\n");
  }
  //pinMode(Pins::checkpointPin,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), Interrups::backRightEncoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(Pins::checkpointPin),Interrups::lackOfProgress, RISING);
  // robot.reloadKits();
}

void loop() {
  if (kSerialScreenTestMode) {
    while (Serial.available() > 0) {
      char current = static_cast<char>(Serial.read());
      if (current == '\n') {
        incomingLine.trim();
        if (incomingLine.length() > 0) {
          Serial.print("RX: ");
          Serial.println(incomingLine);
          robot.screenPrint(incomingLine);
        }
        incomingLine = "";
      } else if (current != '\r') {
        incomingLine += current;
      }
    }
    return;
  }

  m.run_algs();
}
