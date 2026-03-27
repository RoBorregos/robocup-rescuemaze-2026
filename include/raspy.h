#ifndef RASPY_H
#define RASPY_H

#include "Arduino.h"

// Camera IDs
#define CAM_RIGHT 0
#define CAM_LEFT 1

// Victim IDs
#define VICTIM_NONE 0x00
#define VICTIM_PHI 0x01
#define VICTIM_PSI 0x02
#define VICTIM_OMEGA 0x03

// UART Protocol constants
#define UART_SYNC_BYTE1 0xFF
#define UART_SYNC_BYTE2 0xAA
#define UART_PACKET_LEN 0x02
#define UART_REQUEST_CMD 0x01
#define UART_BASE_CHECKSUM 0x03
#define UART_CHECKSUM_MASK 0xFF

// Detection constants
#define DETECTION_ATTEMPTS 3
#define DETECTION_MIN_CONSENSUS 2
#define DETECTION_ATTEMPT_DELAY_MS 50

// Display constants
#define DISPLAY_LINE_SIZE 20
#define DISPLAY_FORMAT_LEFT "L:%s"
#define DISPLAY_FORMAT_RIGHT "R:%s"
#define DISPLAY_SEPARATOR "\n"

class Raspy {
public:
  Raspy();

  // Request detection from Raspberry Pi (RIGHT then LEFT)
  // Returns: VICTIM_NONE / VICTIM_PHI / VICTIM_PSI / VICTIM_OMEGA
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
  uint32_t waitingTime = 5000; // 5 seconds waiting for response

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
  const char *victimIdToName(uint8_t victim_id);
  void updateDisplaySplitScreen();
};

extern Raspy raspy;

#endif
