#include <Arduino.h>
#include "Encoder.h"
#include "maze.h"
#include "Test.h"

maze m;
constexpr bool kSerialScreenTestMode = true;
constexpr bool kEnableSerial2Bridge = false;  // Disabled - using Serial (TX0/RX0) instead
String serialLine;

int servopos=0;
void setup() {
  Serial.begin(115200);
  // robot.setupMotors();
  if (kSerialScreenTestMode) {
    robot.screenBegin();
    robot.screenPrint("READY");
    Serial.println("Serial OLED test mode enabled. Send text ending with \\n");
    Serial.println("UART using TX0/RX0 (connected to Raspberry Pi)");
  }
 /* //pinMode(Pins::checkpointPin,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), Interrups::backRightEncoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(Pins::checkpointPin),Interrups::lackOfProgress, RISING);
  // robot.reloadKits();
  */
}

void loop() {
  auto handleIncoming = [](HardwareSerial& port, String& lineBuffer, const char* source) {
    while (port.available() > 0) {
      char current = static_cast<char>(port.read());
      if (current == '\n') {
        lineBuffer.trim();
        if (lineBuffer.length() > 0) {
          Serial.print("RX(");
          Serial.print(source);
          Serial.print("): ");
          Serial.println(lineBuffer);
          robot.screenPrint(lineBuffer);
        }
        lineBuffer = "";
      } else if (current != '\r') {
        lineBuffer += current;
      }
    }
  };

  if (kSerialScreenTestMode) {
    handleIncoming(Serial, serialLine, "SERIAL");
    return;
  }

  m.run_algs();
}
