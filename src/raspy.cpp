#include "raspy.h"
#include "Test.h"

Raspy raspy;

Raspy::Raspy() {
  camera = CAM_RIGHT;
  victim = VICTIM_NONE;
  rx_state = WAIT_FF;
  rx_index = 0;
  rx_len = 0;
  rx_checksum = 0;
  packet_received = false;
}

void Raspy::sendRequest(uint8_t camera_id) {
  // Send request: 0xFF 0xAA 0x02 0x01 [CAMERA] [CHECKSUM]
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(0x02);      // Length (command + camera)
  Serial.write(0x01);      // Request command
  Serial.write(camera_id); // Camera: 0=RIGHT, 1=LEFT
  Serial.write((uint8_t)(0x03 + camera_id)); // checksum = len + cmd + camera
  Serial.flush();
}

void Raspy::writeSerial(uint8_t camera_id, uint8_t victim_id) {
  // Send vision packet: 0xFF 0xAA [LEN] [CAMERA] [VICTIM] [CHECKSUM]
  uint8_t packet_len = 2;
  uint8_t checksum = (packet_len + camera_id + victim_id) & 0xFF;
  
  Serial.write(0xFF);
  Serial.write(0xAA);
  Serial.write(packet_len);
  Serial.write(camera_id);
  Serial.write(victim_id);
  Serial.write(checksum);
  Serial.flush();
}

bool Raspy::getDetectionFromCamera(uint8_t camera_id) {
  packet_received = false;
  sendRequest(camera_id);

  uint32_t start_time = millis();
  while ((millis() - start_time) < waitingTime) {
    if (readSerial()) {
      return true;
    }
    delay(1);
  }
  return false;
}

uint8_t Raspy::getDetection() {
  Serial.flush();
  victim = VICTIM_NONE;

  // Ask RIGHT first
  if (getDetectionFromCamera(CAM_RIGHT) && victim != VICTIM_NONE) {
    return victim;
  }

  // Then ask LEFT
  if (getDetectionFromCamera(CAM_LEFT) && victim != VICTIM_NONE) {
    return victim;
  }

  return VICTIM_NONE;
}

uint8_t Raspy::getVictim() const {
  return victim;
}

void Raspy::handlePacket(uint8_t len, const uint8_t* payload) {
  if (len != 2) {
    return;  // Invalid packet size
  }
  
  uint8_t cam_id = payload[0];
  uint8_t victim_id = payload[1];
  
  // Validate camera ID
  if (cam_id != CAM_RIGHT && cam_id != CAM_LEFT) {
    return;
  }
  
  // Validate victim ID
  if (victim_id > VICTIM_OMEGA) {
    return;
  }
  
  // Store detection data
  camera = cam_id;
  victim = victim_id;
  packet_received = true;
  
  if (victim != VICTIM_NONE) {
    char display[20];
    const char* cam_name = (cam_id == CAM_RIGHT) ? "RIGHT" : "LEFT";
    const char* victim_name = "NONE";
    if (victim_id == VICTIM_PHI) victim_name = "PHI";
    else if (victim_id == VICTIM_PSI) victim_name = "PSI";
    else if (victim_id == VICTIM_OMEGA) victim_name = "OMEGA";
    
    snprintf(display, sizeof(display), "%s:%s", cam_name, victim_name);
    robot.screenPrint(String(display));
  }
}

void Raspy::parseIncomingByte(uint8_t b) {
  switch (rx_state) {
    case WAIT_FF:
      if (b == 0xFF) {
        rx_state = WAIT_AA;
      }
      break;
    
    case WAIT_AA:
      if (b == 0xAA) {
        rx_state = WAIT_LEN;
      } else {
        rx_state = WAIT_FF;
      }
      break;
    
    case WAIT_LEN:
      rx_len = b;
      rx_index = 0;
      rx_checksum = b;
      if (rx_len == 0 || rx_len > sizeof(rx_payload)) {
        rx_state = WAIT_FF;
      } else {
        rx_state = WAIT_PAYLOAD;
      }
      break;
    
    case WAIT_PAYLOAD:
      rx_payload[rx_index++] = b;
      rx_checksum = (uint8_t)((rx_checksum + b) & 0xFF);
      if (rx_index >= rx_len) {
        rx_state = WAIT_CHECK;
      }
      break;
    
    case WAIT_CHECK:
      rx_state = WAIT_FF;
      if (b == rx_checksum) {
        handlePacket(rx_len, rx_payload);
      }
      break;
  }
}

bool Raspy::readSerial() {
  while (Serial.available() > 0) {
    uint8_t byte = (uint8_t)Serial.read();
    parseIncomingByte(byte);

    if (packet_received) {
      packet_received = false;
      return true;
    }
  }
  return false;
}

void Raspy::connect() {
  // UART is already initialized by Serial.begin() in setup().
}

void Raspy::update() {
  // Process any incoming serial data
  readSerial();
}
