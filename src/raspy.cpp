#include "raspy.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

static constexpr uint8_t HEADER0 = 0xFF;
static constexpr uint8_t HEADER1 = 0xAA;

static constexpr uint8_t CMD_REQUEST_RIGHT = 0x02;
static constexpr uint8_t CMD_REQUEST_LEFT = 0x03;

static constexpr uint8_t CAM_RIGHT = 0;
static constexpr uint8_t CAM_LEFT = 1;

static constexpr uint8_t VICTIM_NONE = 0x00;
static constexpr uint8_t VICTIM_PHI = 0x01;
static constexpr uint8_t VICTIM_PSI = 0x02;
static constexpr uint8_t VICTIM_OMEGA = 0x03;

static constexpr uint8_t OLED_SDA_PIN = 21;
static constexpr uint8_t OLED_SCL_PIN = 22;
static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr uint8_t OLED_WIDTH = 128;
static constexpr uint8_t OLED_HEIGHT = 64;

static constexpr uint32_t POLL_INTERVAL_MS = 250;
static constexpr uint32_t REQUEST_GAP_MS = 10;
static constexpr uint8_t SAMPLES_PER_CAMERA = 3;
static constexpr uint32_t CYCLE_TIMEOUT_MS = 500;

static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

RaspyLink raspyLink;

void RaspyLink::setup() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledReady_ = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledReady_) {
    oledReady_ = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  lastPollMs_ = millis() - POLL_INTERVAL_MS;
  renderOled();
  resetCycle();
  sendCycleRequests();
}

void RaspyLink::getDetection() {
  const uint32_t now = millis();
  if (!cycleActive_ && (now - lastPollMs_ >= POLL_INTERVAL_MS)) {
    lastPollMs_ = now;
    resetCycle();
  }

  if (cycleActive_) {
    sendCycleRequests();
  }

  while (Serial.available() > 0) {
    parseIncomingByte((uint8_t)Serial.read());
  }

  checkCycleTimeout();
}

const char* RaspyLink::victimLabel(uint8_t victimId) const {
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

uint8_t RaspyLink::rightVictim() const {
  return lastVictimRight_;
}

uint8_t RaspyLink::leftVictim() const {
  return lastVictimLeft_;
}

void RaspyLink::sendRequest(uint8_t cmd) {
  const uint8_t payloadLen = 0x01;
  const uint8_t checksum = (uint8_t)((payloadLen + cmd) & 0xFF);
  const uint8_t packet[5] = {HEADER0, HEADER1, payloadLen, cmd, checksum};
  Serial.write(packet, sizeof(packet));
  Serial.flush();
}

void RaspyLink::resetCycle() {
  rightSamples_ = 0;
  leftSamples_ = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    rightVotes_[i] = 0;
    leftVotes_[i] = 0;
  }
  requestStep_ = 0;
  cycleStartMs_ = millis();
  lastRequestMs_ = 0;
  cycleActive_ = true;
}

uint8_t RaspyLink::chooseMostReliableVictim(const uint8_t* votes, uint8_t fallbackVictim) const {
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

void RaspyLink::finalizeCycle() {
  if (rightSamples_ > 0) {
    lastVictimRight_ = chooseMostReliableVictim(rightVotes_, lastVictimRight_);
  }
  if (leftSamples_ > 0) {
    lastVictimLeft_ = chooseMostReliableVictim(leftVotes_, lastVictimLeft_);
  }

  renderOled();
  cycleActive_ = false;
}

void RaspyLink::renderOled() {
  if (!oledReady_) {
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("RIGHT");
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.println(victimLabel(lastVictimRight_));

  oled.setTextSize(1);
  oled.setCursor(64, 0);
  oled.println("LEFT");
  oled.setTextSize(2);
  oled.setCursor(64, 12);
  oled.println(victimLabel(lastVictimLeft_));

  oled.display();
}

void RaspyLink::onDetectionPacket(uint8_t len, const uint8_t* payload) {
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

  if (!cycleActive_) {
    if (camId == CAM_RIGHT) {
      lastVictimRight_ = victimId;
    } else {
      lastVictimLeft_ = victimId;
    }
    renderOled();
    return;
  }

  if (camId == CAM_RIGHT && rightSamples_ < SAMPLES_PER_CAMERA) {
    rightVotes_[victimId]++;
    rightSamples_++;
  } else if (camId == CAM_LEFT && leftSamples_ < SAMPLES_PER_CAMERA) {
    leftVotes_[victimId]++;
    leftSamples_++;
  }

  if (rightSamples_ >= SAMPLES_PER_CAMERA && leftSamples_ >= SAMPLES_PER_CAMERA) {
    finalizeCycle();
  }
}

void RaspyLink::sendCycleRequests() {
  static const uint8_t requestSequence[SAMPLES_PER_CAMERA * 2] = {
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
      CMD_REQUEST_RIGHT, CMD_REQUEST_LEFT,
  };

  const uint8_t totalSteps = sizeof(requestSequence) / sizeof(requestSequence[0]);
  const uint32_t now = millis();

  if (requestStep_ >= totalSteps) {
    return;
  }

  if (lastRequestMs_ == 0 || (now - lastRequestMs_) >= REQUEST_GAP_MS) {
    sendRequest(requestSequence[requestStep_]);
    lastRequestMs_ = now;
    requestStep_++;
  }
}

void RaspyLink::checkCycleTimeout() {
  if (!cycleActive_) {
    return;
  }

  if (millis() - cycleStartMs_ >= CYCLE_TIMEOUT_MS) {
    finalizeCycle();
  }
}

void RaspyLink::parseIncomingByte(uint8_t byteValue) {
  switch (rxState_) {
    case WAIT_FF:
      if (byteValue == HEADER0) {
        rxState_ = WAIT_AA;
      }
      break;

    case WAIT_AA:
      if (byteValue == HEADER1) {
        rxState_ = WAIT_LEN;
      } else {
        rxState_ = WAIT_FF;
      }
      break;

    case WAIT_LEN:
      rxLen_ = byteValue;
      rxIndex_ = 0;
      rxChecksum_ = rxLen_;
      if (rxLen_ == 0 || rxLen_ > sizeof(rxPayload_)) {
        rxState_ = WAIT_FF;
      } else {
        rxState_ = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      rxPayload_[rxIndex_++] = byteValue;
      rxChecksum_ = (uint8_t)((rxChecksum_ + byteValue) & 0xFF);
      if (rxIndex_ >= rxLen_) {
        rxState_ = WAIT_CHECK;
      }
      break;

    case WAIT_CHECK:
      if (byteValue == rxChecksum_) {
        onDetectionPacket(rxLen_, rxPayload_);
      }
      rxState_ = WAIT_FF;
      break;
  }
}
