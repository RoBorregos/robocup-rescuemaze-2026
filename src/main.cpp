#if 0  // DISABLED: test_esp_comunication.cpp is the main program
#include <Arduino.h>
#include "Encoder.h"
#include "maze.h"
#include "Test.h"

// Include vision packet receiver
extern void updateVisionPacketReceiver(HardwareSerial& port);

maze m;
constexpr bool kSerialScreenTestMode = true;
constexpr bool kEnableSerial2Bridge = false;  // Disabled - using Serial (TX0/RX0) instead
String serialLine;

// Binary packet parsing
enum PacketState {
  WAITING_FF,
  WAITING_AA,
  RECEIVE_LEN,
  RECEIVE_CMD,
  RECEIVE_DATA,
  RECEIVE_CHECKSUM
};

PacketState packet_state = WAITING_FF;
uint8_t packet_len = 0;
uint8_t packet_cmd = 0;
uint8_t packet_data = 0;
uint8_t packet_checksum = 0;
uint8_t bytes_received = 0;

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
    handleIncoming(Serial, serialLine, "TEXT");
    updateVisionPacketReceiver(Serial);  // Also process binary vision packets
    return;
  }

  m.run_algs();
}
#endif  // End disabled main.cpp
