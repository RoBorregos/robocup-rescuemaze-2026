#ifndef PINS_ID_H
#define PINS_ID_H
#include <Arduino.h>

namespace MotorID {
constexpr uint8_t kBackRight = 0; 
constexpr uint8_t kBackLeft = 1;   // front left
constexpr uint8_t kFrontRight = 3; // back left
constexpr uint8_t kFrontLeft = 2;  // front right
} // namespace MotorID

namespace Pins {
constexpr uint8_t encoder[4] = {35, 39, 34, 36};
constexpr uint8_t pwmPin[4] = {32, 14, 5, 4};

constexpr uint8_t digitalOne[4] = {25, 27, 17, 2};
constexpr uint8_t digitalTwo[4] = {33, 26, 16, 15};
constexpr uint8_t vlxPins[7] = { // MUX
    3, 4, 0, 2,
    5, // To define
    6, 7};
constexpr uint8_t tcsPins[1] = {2};
constexpr uint8_t limitSwitchPins[2] = {
    13, // LEFT
    23  // RIGHT
};
constexpr uint8_t servoPinR = 19;
constexpr uint8_t servoPinL = 18;
constexpr uint8_t checkpointPin = 19;
constexpr uint8_t LedsPin = 12;

} // namespace Pins
namespace LimitSwitchID {
constexpr uint8_t kLeft = 0; 
constexpr uint8_t kRight = 1;
} // namespace LimitSwitchID

namespace vlxID {
constexpr uint8_t frontLeft = 4;
constexpr uint8_t frontRight = 1;
// constexpr uint8_t rightDown=1;
constexpr uint8_t right = 3;
constexpr uint8_t left = 6;
// constexpr uint8_t front=4;
// constexpr uint8_t leftDown=5; // backLeft
constexpr uint8_t back = 0;

} // namespace vlxID
namespace kitID {
constexpr uint8_t kRight = 2;
constexpr uint8_t kLeft = 1;
constexpr uint8_t kNone = 0;
} // namespace kitID
namespace rampID {
constexpr uint8_t kUp = 1;
constexpr uint8_t kDown = 2;
constexpr uint8_t kNone = 0;
} // namespace rampID

#endif