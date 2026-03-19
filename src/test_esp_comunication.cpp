#if 0
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const uint32_t LINK_BAUD = 115200;
static const bool ENABLE_DEBUG_LOGS = false;

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

// ----- Protocolo -----
static const uint8_t HEADER0 = 0xFF;
static const uint8_t HEADER1 = 0xAA;

static const uint8_t CMD_REQUEST_RIGHT = 0x02;
static const uint8_t CMD_REQUEST_LEFT  = 0x03;

static const uint8_t CAM_RIGHT = 0;
static const uint8_t CAM_LEFT  = 1;

static const uint8_t VICTIM_NONE  = 0x00;
static const uint8_t VICTIM_PHI   = 0x01; // phi
static const uint8_t VICTIM_PSI   = 0x02; // psi
static const uint8_t VICTIM_OMEGA = 0x03; // omega

static uint8_t lastRequestedCmd = CMD_REQUEST_RIGHT;
static uint8_t lastCamId = CAM_RIGHT;
static uint8_t lastVictimId = VICTIM_NONE;
static uint32_t packetCounter = 0;

// Polling continuo
// ----- Protocol -----
static uint32_t lastPollMs = 0;
static bool askRightNext = true;
// Continuous polling
// status RX parser
enum RxState : uint8_t {
// RX parser state
  WAIT_AA,
  WAIT_LEN,
  // Put your real "do X action" here.
  // For now we keep an example action with LED and logs.
};
  // Example: number of blinks by victim type
static RxState rxState = WAIT_FF;
static uint8_t rxLen = 0;
static uint8_t rxPayload[16];
static uint8_t rxIndex = 0;
      Serial.print("[RX] invalid len: ");

const char* cameraLabel(uint8_t camId) {
  if (camId == CAM_RIGHT) return "RIGHT";
  if (camId == CAM_LEFT) return "LEFT";
  return "UNKNOWN_CAM";
}

      Serial.print("[RX] invalid cam_id: ");
  switch (victimId) {
    case VICTIM_NONE:  return "N";
    case VICTIM_PHI:   return "PHI";
    case VICTIM_PSI:   return "PSI";
    case VICTIM_OMEGA: return "OMEGA";
    default:           return "UNKNOWN_VICTIM";
  }
      Serial.print("[RX] invalid victim_id: ");

void renderOled() {
  if (!oledReady) {
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

          Serial.print("[RX] invalid checksum. expected=0x");
  oled.setCursor(0, 6);
          Serial.print(" received=0x");

  oled.setTextSize(3);
  oled.setCursor(0, 34);
  oled.println(victimLabel(lastVictimId));

  // 1) Continuous polling (ESP always requests)
}

  // 2) Process incoming bytes
  const uint8_t len = 0x01;
  const uint8_t checksum = (uint8_t)((len + cmd) & 0xFF);

  uint8_t pkt[5] = {HEADER0, HEADER1, len, cmd, checksum};
  Serial.write(pkt, sizeof(pkt));
  lastRequestedCmd = cmd;
  renderOled();

  if (ENABLE_DEBUG_LOGS) {
    Serial.print("[ESP->HOST] request cmd=0x");
    Serial.print(cmd, HEX);
    Serial.print(" (");
    Serial.print((cmd == CMD_REQUEST_RIGHT) ? "RIGHT" : "LEFT");
    Serial.println(")");
  }
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
      Serial.print("[RX] len invalido: ");
      Serial.println(len);
    }
    return;
  }

  const uint8_t camId = payload[0];
  const uint8_t victimId = payload[1];

  if (camId != CAM_RIGHT && camId != CAM_LEFT) {
    if (ENABLE_DEBUG_LOGS) {
      Serial.print("[RX] cam_id invalido: ");
      Serial.println(camId);
    }
    return;
  }

  if (victimId > VICTIM_OMEGA) {
    if (ENABLE_DEBUG_LOGS) {
      Serial.print("[RX] victim_id invalido: ");
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
          Serial.print("[RX] checksum invalido. esperado=0x");
          Serial.print(rxChecksum, HEX);
          Serial.print(" recibido=0x");
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
    oled.println("WAIT");
    oled.setTextSize(3);
    oled.setCursor(0, 28);
    oled.println("N");
    oled.display();
    delay(400);
  }

  renderOled();

  if (ENABLE_DEBUG_LOGS) {
    Serial.println("ESP Serial Polling Test");
    Serial.print("Link Serial ");
    Serial.println(LINK_BAUD);
  }
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;
    const uint8_t cmd = askRightNext ? CMD_REQUEST_RIGHT : CMD_REQUEST_LEFT;
    askRightNext = !askRightNext;
    sendRequest(cmd);
  }

    // Polling loop
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    parseIncomingByte(b);
  }
}
#endif