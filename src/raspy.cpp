#include "raspy.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

namespace raspy {

static const uint8_t HEADER0 = 0xFF;
static const uint8_t HEADER1 = 0xAA;

static const uint8_t CMD_REQUEST_RIGHT = 0x02;
static const uint8_t CMD_REQUEST_LEFT = 0x03;

static const uint8_t CAM_RIGHT = 0;
static const uint8_t CAM_LEFT = 1;

static const uint8_t VICTIM_NONE = 0x00;
static const uint8_t VICTIM_PHI = 0x01;
static const uint8_t VICTIM_PSI = 0x02;
static const uint8_t VICTIM_OMEGA = 0x03;

static const uint8_t OLED_SDA_PIN = 21;
static const uint8_t OLED_SCL_PIN = 22;
static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t OLED_WIDTH = 128;
static const uint8_t OLED_HEIGHT = 64;

static const uint32_t POLL_INTERVAL_MS = 250;
static const uint32_t REQUEST_GAP_MS = 10;
static const uint8_t SAMPLES_PER_CAMERA = 3;
static const uint32_t CYCLE_TIMEOUT_MS = 500;

enum RxState : uint8_t {
  WAIT_FF,
  WAIT_AA,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECK
};

static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool oledReady = false;

static uint8_t lastVictimRight = VICTIM_NONE;
static uint8_t lastVictimLeft = VICTIM_NONE;

static uint32_t lastPollMs = 0;
static uint32_t cycleStartMs = 0;
static uint32_t lastRequestMs = 0;

static bool cycleActive = false;
static uint8_t requestStep = 0;

static uint8_t rightSamples = 0;
static uint8_t leftSamples = 0;
static uint8_t rightVotes[4] = {0, 0, 0, 0};
static uint8_t leftVotes[4] = {0, 0, 0, 0};

static RxState rxState = WAIT_FF;
static uint8_t rxLen = 0;
static uint8_t rxPayload[16];
static uint8_t rxIndex = 0;
static uint8_t rxChecksum = 0;

static void renderOled();

const char* victimLabel(uint8_t victimId) {
  switch (victimId) {
    case VICTIM_NONE:
      return "N";
    case VICTIM_PHI:
      return "PHI";
    case VICTIM_PSI:
      return "PSI";
    case VICTIM_OMEGA:
      return "OMEGA";
    default:
      return "UNKNOWN";
  }
}

uint8_t rightVictim() {
  return lastVictimRight;
}

uint8_t leftVictim() {
  return lastVictimLeft;
}

static void sendRequest(uint8_t cmd) {
  const uint8_t payloadLen = 0x01;
  const uint8_t checksum = (uint8_t)((payloadLen + cmd) & 0xFF);
  const uint8_t packet[5] = {HEADER0, HEADER1, payloadLen, cmd, checksum};
  Serial.write(packet, sizeof(packet));
}

static void resetCycle() {
  rightSamples = 0;
  leftSamples = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    rightVotes[i] = 0;
    leftVotes[i] = 0;
  }
  requestStep = 0;
  cycleStartMs = millis();
  lastRequestMs = 0;
  cycleActive = true;
}

static uint8_t chooseMostReliableVictim(const uint8_t* votes, uint8_t fallbackVictim) {
  uint8_t bestVictim = fallbackVictim;
  uint8_t bestCount = 0;

  for (uint8_t victim = VICTIM_NONE; victim <= VICTIM_OMEGA; ++victim) {
    if (votes[victim] > bestCount) {
      bestCount = votes[victim];
      bestVictim = victim;
    } else if (votes[victim] == bestCount && votes[victim] > 0) {
      if (victim != VICTIM_NONE && bestVictim == VICTIM_NONE) {
        bestVictim = victim;
      }
    }
  }

  return bestVictim;
}

static void finalizeCycle() {
  if (rightSamples > 0) {
    lastVictimRight = chooseMostReliableVictim(rightVotes, lastVictimRight);
  }
  if (leftSamples > 0) {
    lastVictimLeft = chooseMostReliableVictim(leftVotes, lastVictimLeft);
  }

  renderOled();
  cycleActive = false;
}

static void renderOled() {
  if (!oledReady) {
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("RIGHT");
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.println(victimLabel(lastVictimRight));

  oled.setTextSize(1);
  oled.setCursor(64, 0);
  oled.println("LEFT");
  oled.setTextSize(2);
  oled.setCursor(64, 12);
  oled.println(victimLabel(lastVictimLeft));

  oled.display();
}

static void onDetectionPacket(uint8_t len, const uint8_t* payload) {
  if (len != 2) {
    return;
  }

  const uint8_t camId = payload[0];
  const uint8_t victimId = payload[1];

  if (camId != CAM_RIGHT && camId != CAM_LEFT) {
    return;
  }

  if (victimId > VICTIM_OMEGA) {
    return;
  }

  if (!cycleActive) {
    if (camId == CAM_RIGHT) {
      lastVictimRight = victimId;
    } else {
      lastVictimLeft = victimId;
    }
    renderOled();
    return;
  }

  if (camId == CAM_RIGHT && rightSamples < SAMPLES_PER_CAMERA) {
    rightVotes[victimId]++;
    rightSamples++;
  } else if (camId == CAM_LEFT && leftSamples < SAMPLES_PER_CAMERA) {
    leftVotes[victimId]++;
    leftSamples++;
  }

  if (rightSamples >= SAMPLES_PER_CAMERA && leftSamples >= SAMPLES_PER_CAMERA) {
    finalizeCycle();
  }
}

static void sendCycleRequests() {
  static const uint8_t requestSequence[SAMPLES_PER_CAMERA * 2] = {
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
  };

  const uint8_t totalSteps = sizeof(requestSequence) / sizeof(requestSequence[0]);
  const uint32_t now = millis();

  if (requestStep >= totalSteps) {
    return;
  }

  if (lastRequestMs == 0 || (now - lastRequestMs) >= REQUEST_GAP_MS) {
    sendRequest(requestSequence[requestStep]);
    lastRequestMs = now;
    requestStep++;
  }
}

static void checkCycleTimeout() {
  if (!cycleActive) {
    return;
  }

  if (millis() - cycleStartMs >= CYCLE_TIMEOUT_MS) {
    finalizeCycle();
  }
}

static void parseIncomingByte(uint8_t byteValue) {
  switch (rxState) {
    case WAIT_FF:
      if (byteValue == HEADER0) {
        rxState = WAIT_AA;
      }
      break;

    case WAIT_AA:
      if (byteValue == HEADER1) {
        rxState = WAIT_LEN;
      } else {
        rxState = WAIT_FF;
      }
      break;

    case WAIT_LEN:
      rxLen = byteValue;
      rxIndex = 0;
      rxChecksum = rxLen;
      if (rxLen == 0 || rxLen > sizeof(rxPayload)) {
        rxState = WAIT_FF;
      } else {
        rxState = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      rxPayload[rxIndex++] = byteValue;
      rxChecksum = (uint8_t)((rxChecksum + byteValue) & 0xFF);
      if (rxIndex >= rxLen) {
        rxState = WAIT_CHECK;
      }
      break;

    case WAIT_CHECK:
      if (byteValue == rxChecksum) {
        onDetectionPacket(rxLen, rxPayload);
      }
      rxState = WAIT_FF;
      break;
  }
}

void setupRaspy() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  renderOled();
}

void getDetection() {
  const uint32_t now = millis();
  if (!cycleActive && (now - lastPollMs >= POLL_INTERVAL_MS)) {
    lastPollMs = now;
    resetCycle();
  }

  if (cycleActive) {
    sendCycleRequests();
  }

  while (Serial.available() > 0) {
    parseIncomingByte((uint8_t)Serial.read());
  }

  checkCycleTimeout();
}

}  // namespace raspy
