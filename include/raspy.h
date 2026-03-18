#ifndef RASPY_H
#define RASPY_H

#include <Arduino.h>

class RaspyLink {
public:
	void setup();
	void getDetection();

	uint8_t rightVictim() const;
	uint8_t leftVictim() const;
	const char* victimLabel(uint8_t victimId) const;

private:
	enum RxState : uint8_t {
		WAIT_FF,
		WAIT_AA,
		WAIT_LEN,
		WAIT_PAYLOAD,
		WAIT_CHECK
	};

	void sendRequest(uint8_t cmd);
	void resetCycle();
	uint8_t chooseMostReliableVictim(const uint8_t* votes, uint8_t fallbackVictim) const;
	void finalizeCycle();
	void renderOled();
	void onDetectionPacket(uint8_t len, const uint8_t* payload);
	void sendCycleRequests();
	void checkCycleTimeout();
	void parseIncomingByte(uint8_t byteValue);

	bool oledReady_ = false;

	uint8_t lastVictimRight_ = 0x00;
	uint8_t lastVictimLeft_ = 0x00;

	uint32_t lastPollMs_ = 0;
	uint32_t cycleStartMs_ = 0;
	uint32_t lastRequestMs_ = 0;

	bool cycleActive_ = false;
	uint8_t requestStep_ = 0;

	uint8_t rightSamples_ = 0;
	uint8_t leftSamples_ = 0;
	uint8_t rightVotes_[4] = {0, 0, 0, 0};
	uint8_t leftVotes_[4] = {0, 0, 0, 0};

	RxState rxState_ = WAIT_FF;
	uint8_t rxLen_ = 0;
	uint8_t rxPayload_[16] = {0};
	uint8_t rxIndex_ = 0;
	uint8_t rxChecksum_ = 0;
};

extern RaspyLink raspyLink;

#endif
