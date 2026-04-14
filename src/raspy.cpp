#include "raspy.h"
#include "Test.h"

Raspy raspy;

Raspy::Raspy() {
  camera = CAM_RIGHT;
  victim = VICTIM_NONE;
  left_victim = VICTIM_NONE;
  right_victim = VICTIM_NONE;
  rx_state = WAIT_FF;
  rx_index = 0;
  rx_len = 0;
  rx_checksum = 0;
  packet_received = false;
}

void Raspy::sendRequest(uint8_t camera_id) {
  // Send request: 0xFF 0xAA 0x02 0x01 [CAMERA] [CHECKSUM]
  Serial.write(UART_SYNC_BYTE1);
  Serial.write(UART_SYNC_BYTE2);
  Serial.write(UART_PACKET_LEN);
  Serial.write(UART_REQUEST_CMD);
  Serial.write(camera_id);
  Serial.write((uint8_t)(UART_BASE_CHECKSUM + camera_id));
  Serial.flush();
}

void Raspy::writeSerial(uint8_t camera_id, uint8_t victim_id) {
  // Send vision packet: 0xFF 0xAA [LEN] [CAMERA] [VICTIM] [CHECKSUM]
  uint8_t packet_len = UART_PACKET_LEN;
  uint8_t checksum = (packet_len + camera_id + victim_id) & UART_CHECKSUM_MASK;

  Serial.write(UART_SYNC_BYTE1);
  Serial.write(UART_SYNC_BYTE2);
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
  left_victim = VICTIM_NONE;
  right_victim = VICTIM_NONE;
  const int min_consensus_required =
      (DETECTION_MIN_CONSENSUS > DETECTION_ATTEMPTS) ? DETECTION_ATTEMPTS
                                                      : DETECTION_MIN_CONSENSUS;

  // Make DETECTION_ATTEMPTS attempts for RIGHT camera
  uint8_t right_results[DETECTION_ATTEMPTS] = {0};
  for (int i = 0; i < DETECTION_ATTEMPTS; i++) {
    if (getDetectionFromCamera(CAM_RIGHT)) {
      right_results[i] = victim;
    }
    delay(DETECTION_ATTEMPT_DELAY_MS);
  }

  // Make DETECTION_ATTEMPTS attempts for LEFT camera
  uint8_t left_results[DETECTION_ATTEMPTS] = {0};
  for (int i = 0; i < DETECTION_ATTEMPTS; i++) {
    if (getDetectionFromCamera(CAM_LEFT)) {
      left_results[i] = victim;
    }
    delay(DETECTION_ATTEMPT_DELAY_MS);
  }

  // Find consensus for RIGHT: most common result (need at least
  // DETECTION_MIN_CONSENSUS)
  uint8_t right_consensus = VICTIM_NONE;
  int max_count = 0;
  for (uint8_t val = VICTIM_NONE; val <= VICTIM_MAX_ID; val++) {
    int count = 0;
    for (int i = 0; i < DETECTION_ATTEMPTS; i++) {
      if (right_results[i] == val)
        count++;
    }
    if (count > max_count) {
      max_count = count;
      right_consensus = val;
    }
  }
  right_victim =
      (max_count >= min_consensus_required) ? right_consensus : VICTIM_NONE;

  // Find consensus for LEFT: most common result (need at least
  // DETECTION_MIN_CONSENSUS)
  uint8_t left_consensus = VICTIM_NONE;
  max_count = 0;
  for (uint8_t val = VICTIM_NONE; val <= VICTIM_MAX_ID; val++) {
    int count = 0;
    for (int i = 0; i < DETECTION_ATTEMPTS; i++) {
      if (left_results[i] == val)
        count++;
    }
    if (count > max_count) {
      max_count = count;
      left_consensus = val;
    }
  }
  left_victim =
      (max_count >= min_consensus_required) ? left_consensus : VICTIM_NONE;

  // Update display with split screen: LEFT on top, RIGHT on bottom
  updateDisplaySplitScreen();

  // Return first valid victim (LEFT priority, then RIGHT)
  if (left_victim != VICTIM_NONE) {
    victim = left_victim;
    camera = CAM_LEFT;
    return left_victim;
  }
  if (right_victim != VICTIM_NONE) {
    victim = right_victim;
    camera = CAM_RIGHT;
    return right_victim;
  }

  victim = VICTIM_NONE;
  return VICTIM_NONE;
}

uint8_t Raspy::getVictim() const { return victim; }

void Raspy::handlePacket(uint8_t len, const uint8_t *payload) {
  if (len != UART_PACKET_LEN) {
    return; // Invalid packet size
  }

  uint8_t cam_id = payload[0];
  uint8_t protocol_victim_id = payload[1];

  // Validate camera ID
  if (cam_id != CAM_RIGHT && cam_id != CAM_LEFT) {
    return;
  }

  // Validate victim ID
  if (protocol_victim_id > PROTO_VICTIM_MAX_ID) {
    return;
  }

  uint8_t victim_id = mapProtocolVictimToInternal(protocol_victim_id);

  // Store detection data
  camera = cam_id;
  victim = victim_id;
  packet_received = true;
}

uint8_t Raspy::mapProtocolVictimToInternal(uint8_t protocol_victim_id) {
  switch (protocol_victim_id) {
  case PROTO_VICTIM_PHI:
    return VICTIM_LETTER_H;
  case PROTO_VICTIM_PSI:
    return VICTIM_LETTER_S;
  case PROTO_VICTIM_OMEGA:
    return VICTIM_LETTER_U;
  case PROTO_VICTIM_HARMED:
    return VICTIM_HARMED;
  case PROTO_VICTIM_UNHARMED:
    return VICTIM_UNHARMED;
  case PROTO_VICTIM_STABLE:
    return VICTIM_STABLE;
  case PROTO_VICTIM_FAKE_TARGET:
    return VICTIM_FAKE_TARGET;
  default:
    return VICTIM_NONE;
  }
}

const char *Raspy::victimIdToName(uint8_t victim_id) {
  switch (victim_id) {
  case VICTIM_HARMED:
    return "HARMED";
  case VICTIM_STABLE:
    return "STABLE";
  case VICTIM_UNHARMED:
    return "UNHARMED";
  case VICTIM_PHI:
    return "PHI";
  case VICTIM_PSI:
    return "PSI";
  case VICTIM_OMEGA:
    return "OMEGA";
  case VICTIM_FAKE_TARGET:
    return "FAKE";
  default:
    return "NONE";
  }
}

void Raspy::updateDisplaySplitScreen() {
  // Split screen: LEFT on top (line 1), RIGHT on bottom (line 2)
  char line1[DISPLAY_LINE_SIZE];
  char line2[DISPLAY_LINE_SIZE];

  const char *left_name = victimIdToName(left_victim);
  const char *right_name = victimIdToName(right_victim);

  snprintf(line1, sizeof(line1), DISPLAY_FORMAT_LEFT, left_name);
  snprintf(line2, sizeof(line2), DISPLAY_FORMAT_RIGHT, right_name);

  // Combine both lines with newline character
  String display = String(line1) + String(DISPLAY_SEPARATOR) + String(line2);
  robot.screenPrint(display);
}

void Raspy::parseIncomingByte(uint8_t b) {
  switch (rx_state) {
  case WAIT_FF:
    if (b == UART_SYNC_BYTE1) {
      rx_state = WAIT_AA;
    }
    break;

  case WAIT_AA:
    if (b == UART_SYNC_BYTE2) {
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
    rx_checksum = (uint8_t)((rx_checksum + b) & UART_CHECKSUM_MASK);
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
