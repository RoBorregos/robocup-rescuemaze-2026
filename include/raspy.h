#ifndef RASPY_H
#define RASPY_H

#include <Arduino.h>

namespace raspy {

void setupRaspy();
void getDetection();

uint8_t rightVictim();
uint8_t leftVictim();
const char* victimLabel(uint8_t victimId);

}  // namespace raspy

#endif
