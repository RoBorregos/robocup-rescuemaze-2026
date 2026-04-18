#ifndef RASPY_H
#define RASPY_H

#include "Arduino.h"

// Camera IDs
#define CAM_RIGHT 1
#define CAM_LEFT 0

// Victim IDs
#define VICTIM_NONE 0x00
#define VICTIM_HARMED 0x01
#define VICTIM_STABLE 0x02
#define VICTIM_UNHARMED 0x03
#define VICTIM_LETTER_H 0x04
#define VICTIM_LETTER_S 0x05
#define VICTIM_LETTER_U 0x06
#define VICTIM_FAKE_TARGET 0x07
#define VICTIM_MAX_ID VICTIM_FAKE_TARGET

// Backward compatibility aliases
#define VICTIM_PHI VICTIM_LETTER_H
#define VICTIM_PSI VICTIM_LETTER_S
#define VICTIM_OMEGA VICTIM_LETTER_U

// Protocol victim IDs received from Raspberry Pi
#define PROTO_VICTIM_NONE 0x00
#define PROTO_VICTIM_PHI 0x01
#define PROTO_VICTIM_PSI 0x02
#define PROTO_VICTIM_OMEGA 0x03
#define PROTO_VICTIM_HARMED 0x04
#define PROTO_VICTIM_UNHARMED 0x05
#define PROTO_VICTIM_STABLE 0x06
#define PROTO_VICTIM_FAKE_TARGET 0x07
#define PROTO_VICTIM_MAX_ID PROTO_VICTIM_FAKE_TARGET

// UART Protocol constants
#define UART_SYNC_BYTE1 0xFF
#define UART_SYNC_BYTE2 0xAA
#define UART_PACKET_LEN 0x02
#define UART_REQUEST_CMD 0x01
#define UART_BASE_CHECKSUM 0x03
#define UART_CHECKSUM_MASK 0xFF

// Detection constants
#define DETECTION_ATTEMPTS 1
#define DETECTION_MIN_CONSENSUS 1
#define DETECTION_ATTEMPT_DELAY_MS 10

// Display constants
#define DISPLAY_LINE_SIZE 20
#define DISPLAY_FORMAT_LEFT "L:%s"
#define DISPLAY_FORMAT_RIGHT "R:%s"
#define DISPLAY_SEPARATOR "\n"

class Raspy {
public:
  Raspy();

  // Request detection from Raspberry Pi (RIGHT then LEFT)
  // Returns victim IDs (target or letter classes)
  uint8_t getDetection();
  uint8_t getVictim() const;

  // Write packet to serial
  void writeSerial(uint8_t camera_id, uint8_t victim_id);

  // Read incoming response from Raspberry Pi
  bool readSerial();

  // Check connection
  void connect();

  // Call periodically to process incoming data
  void update();

  // Data storage
  uint8_t camera;
  uint8_t victim;
  uint8_t left_victim;
  uint8_t right_victim;
  uint32_t waitingTime = 1000; // 5 seconds waiting for response

private:
  // Packet parsing states
  enum RxState { WAIT_FF, WAIT_AA, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECK };

  RxState rx_state;
  uint8_t rx_len;
  uint8_t rx_payload[16];
  uint8_t rx_index;
  uint8_t rx_checksum;
  bool packet_received;

  bool getDetectionFromCamera(uint8_t camera_id);

  void parseIncomingByte(uint8_t b);
  void handlePacket(uint8_t len, const uint8_t *payload);
  void sendRequest(uint8_t camera_id);
  uint8_t mapProtocolVictimToInternal(uint8_t protocol_victim_id);
  const char *victimIdToName(uint8_t victim_id);
  void updateDisplaySplitScreen();
};

extern Raspy raspy;

#endif
