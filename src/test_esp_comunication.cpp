#if 0 // DISABLED: Using raspy.h/raspy.cpp instead
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

static const uint32_t LINK_BAUD = 115200;
static const bool ENABLE_DEBUG_LOGS = true;

static const uint8_t OLED_SDA_PIN = 21;
static const uint8_t OLED_SCL_PIN = 22;
static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t OLED_WIDTH = 128;
static const uint8_t OLED_HEIGHT = 64;

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool oledReady = false;

#ifndef LED_BUILTIN
static const uint8_t STATUS_LED_PIN = 2;
#else
static const uint8_t STATUS_LED_PIN = LED_BUILTIN;
#endif

// ----- Protocol -----
static const uint8_t HEADER0 = 0xFF;
static const uint8_t HEADER1 = 0xAA;

static const uint8_t CAM_RIGHT = 0;
static const uint8_t CAM_LEFT  = 1;

static const uint8_t VICTIM_NONE  = 0x00;
static const uint8_t VICTIM_PHI   = 0x01; // phi (Φ)
static const uint8_t VICTIM_PSI   = 0x02; // psi (Ψ)
static const uint8_t VICTIM_OMEGA = 0x03; // omega (Ω)

static uint8_t lastCamId = CAM_RIGHT;
static uint8_t lastVictimId = VICTIM_NONE;
static uint32_t packetCounter = 0;

// Polling timer
static uint32_t lastRequestMs = 0;
static const uint32_t REQUEST_INTERVAL_MS = 5000;  // Ask every 5 seconds (increased for input time)

// RX parser state
enum RxState : uint8_t {
  WAIT_FF,
  WAIT_AA,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECK,
};

static RxState rxState = WAIT_FF;
static uint8_t rxLen = 0;
static uint8_t rxPayload[16];
static uint8_t rxIndex = 0;
static uint8_t rxChecksum = 0;

const char* cameraLabel(uint8_t camId) {
  if (camId == CAM_RIGHT) return "RIGHT";
  if (camId == CAM_LEFT) return "LEFT";
  return "UNKNOWN";
}

const char* victimLabel(uint8_t victimId) {
  switch (victimId) {
    case VICTIM_NONE:  return "N";
    case VICTIM_PHI:   return "PHI";
    case VICTIM_PSI:   return "PSI";
    case VICTIM_OMEGA: return "OMEGA";
    default:           return "UNKNOWN";
  }
}

void renderOled() {
  if (!oledReady) {
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("CAM: ");
  oled.println(cameraLabel(lastCamId));
  
  oled.setCursor(0, 10);
  oled.print("PKT: ");
  oled.println(packetCounter);

  oled.setTextSize(3);
  oled.setCursor(0, 34);
  oled.println(victimLabel(lastVictimId));
  
  oled.display();
}

void doAction(uint8_t camId, uint8_t victimId) {
  if (ENABLE_DEBUG_LOGS) {
    Serial.print("[ACTION] cam=");
    Serial.print(camId);
    Serial.print(" ");
    Serial.print(cameraLabel(camId));
    Serial.print(" | victim=");
    Serial.print(victimId);
    Serial.print(" ");
    Serial.println(victimLabel(victimId));
  }

  uint8_t blinks = 1;
  if (victimId == VICTIM_PHI) blinks = 2;
  else if (victimId == VICTIM_PSI) blinks = 3;
  else if (victimId == VICTIM_OMEGA) blinks = 4;

  for (uint8_t i = 0; i < blinks; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(60);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
  }

  renderOled();
}

void handlePacket(uint8_t len, const uint8_t* payload) {
  if (len != 2) {
    if (ENABLE_DEBUG_LOGS) {
      Serial.print("[RX] len invalid: ");
      Serial.println(len);
    }
    return;
  }

  const uint8_t camId = payload[0];
  const uint8_t victimId = payload[1];

  if (camId != CAM_RIGHT && camId != CAM_LEFT) {
    if (ENABLE_DEBUG_LOGS) {
      Serial.print("[RX] cam_id invalid: ");
      Serial.println(camId);
    }
    return;
  }

  if (victimId > VICTIM_OMEGA) {
    if (ENABLE_DEBUG_LOGS) {
      Serial.print("[RX] victim_id invalid: ");
      Serial.println(victimId);
    }
    return;
  }

  if (ENABLE_DEBUG_LOGS) {
    Serial.print("[HOST->ESP] cam=");
    Serial.print(camId);
    Serial.print(" ");
    Serial.print(cameraLabel(camId));
    Serial.print(" | victim=");
    Serial.print(victimId);
    Serial.print(" ");
    Serial.println(victimLabel(victimId));
  }

  lastCamId = camId;
  lastVictimId = victimId;
  packetCounter++;
  renderOled();

  doAction(camId, victimId);
  
  // Send ACK (acknowledgment) back to Raspberry Pi
  // Format: 0xFF 0xAA 0x02 [STATUS] 0x00 [CHECKSUM]
  // STATUS: 0 = OK/received
  uint8_t ack_packet[6] = {0xFF, 0xAA, 0x02, 0x00, 0x00, 0x02};  // Checksum: 0x02+0x00+0x00=0x02
  Serial.write(ack_packet, sizeof(ack_packet));
  Serial.flush();
  
  if (ENABLE_DEBUG_LOGS) {
    Serial.println("[ESP->HOST] ACK sent (DELIVERED)");
  }
}

void sendRequest() {
  // Request format: 0xFF 0xAA 0x01 0x01 0x01
  // HEADER0=0xFF, HEADER1=0xAA, LEN=0x01, CMD=0x01(REQUEST), CHECKSUM=0x02
  uint8_t request[5] = {0xFF, 0xAA, 0x01, 0x01, 0x02};
  Serial.write(request, sizeof(request));
  Serial.flush();
  
  if (ENABLE_DEBUG_LOGS) {
    Serial.println("[ESP->HOST] REQUEST sent");
  }
}

void parseIncomingByte(uint8_t b) {
  switch (rxState) {
    case WAIT_FF:
      if (b == HEADER0) {
        rxState = WAIT_AA;
      }
      break;

    case WAIT_AA:
      if (b == HEADER1) {
        rxState = WAIT_LEN;
      } else {
        rxState = WAIT_FF;
      }
      break;

    case WAIT_LEN:
      rxLen = b;
      rxIndex = 0;
      rxChecksum = rxLen;

      if (rxLen == 0 || rxLen > sizeof(rxPayload)) {
        rxState = WAIT_FF;
      } else {
        rxState = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      rxPayload[rxIndex++] = b;
      rxChecksum = (uint8_t)((rxChecksum + b) & 0xFF);
      if (rxIndex >= rxLen) {
        rxState = WAIT_CHECK;
      }
      break;

    case WAIT_CHECK:
      if (b == rxChecksum) {
        handlePacket(rxLen, rxPayload);
      } else {
        if (ENABLE_DEBUG_LOGS) {
          Serial.print("[RX] checksum invalid. expected=0x");
          Serial.print(rxChecksum, HEX);
          Serial.print(" received=0x");
          Serial.println(b, HEX);
        }
      }
      rxState = WAIT_FF;
      break;
  }
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(300);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledReady) {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("READY");
    oled.display();
    delay(400);
  }

  renderOled();

  if (ENABLE_DEBUG_LOGS) {
    Serial.println("==== Vision Packet Receiver ====");
    Serial.print("Baud: ");
    Serial.println(LINK_BAUD);
    Serial.println("Waiting for packets (0xFF 0xAA [LEN] [CAM] [VICTIM] [CHECKSUM])...");
  }
}

void loop() {
  // Periodically request data from Raspberry Pi
  const uint32_t now = millis();
  if (now - lastRequestMs >= REQUEST_INTERVAL_MS) {
    lastRequestMs = now;
    sendRequest();
  }
  
  // Process incoming bytes from Raspberry Pi
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    parseIncomingByte(b);
  }
}
#endif