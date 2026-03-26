
#ifndef Encoder_h
#define Encoder_h

#include "Pins_ID.h"
#include "motors.h"
#include <Arduino.h>
extern motors robot;
namespace Interrups {
void backRightEncoder();
void backLeftEncoder();
void frontRightEncoder();
void frontLeftEncoder();
void lackOfProgress();
extern int deltaTics[4];
}; // namespace Interrups

#endif