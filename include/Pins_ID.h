#ifndef PINS_ID_H
#define PINS_ID_H
#include <Arduino.h>

namespace MotorID{
    constexpr uint8_t kBackRight=0;
    constexpr uint8_t kBackLeft=3;   //front left
    constexpr uint8_t kFrontRight=1; //back left
    constexpr uint8_t kFrontLeft=2;  //front right
}

namespace Pins{
    constexpr uint8_t encoder[4] = {
        36, 
        39, 
        34,  
        35 
    };
    constexpr uint8_t pwmPin[4] = {
        2, 
        5, 
        26, 
        32 
    };

    constexpr uint8_t digitalOne[4] = {
        15,
        17, 
        27, 
        33  
    };  
    constexpr uint8_t digitalTwo[4] = {
        4, 
        16, 
        14,
        25
    };
    constexpr uint8_t vlxPins[7] = {//MUX
        3,  
        4, 
        0, 
        2, 
        5, //To define
        6, 
        7
    };
    constexpr uint8_t tcsPins[1] = {
        6 
    };
    constexpr uint8_t limitSwitchPins[2] = {
        13, // LEFT
        23  // RIGHT
    };
    constexpr uint8_t servoPin = 18;
    constexpr uint8_t checkpointPin = 19;
    constexpr uint8_t LedsPin = 12;


}
namespace LimitSwitchID {
    constexpr uint8_t kLeft = 0;
    constexpr uint8_t kRight = 1;
}

namespace vlxID{
    constexpr uint8_t frontLeft=0;
    constexpr uint8_t frontRight=4;
    //constexpr uint8_t rightDown=1;
    constexpr uint8_t rightUp=1;
    constexpr uint8_t leftUp=3;
    //constexpr uint8_t front=4;
    //constexpr uint8_t leftDown=5; // backLeft
    constexpr uint8_t back=2;

}

constexpr uint8_t TaskVLX1[2]{ 
    //vlxID::frontCenter, 
    //vlxID::front, 
    //vlxID::frontRight, 
    //vlxID::frontLeft 
    vlxID::frontLeft,
    //vlxID::front,  
    vlxID::frontRight,
};

constexpr uint8_t TaskVLX2[3]{
    vlxID::rightUp,
    vlxID::leftUp,
    vlxID::back
    //vlxID::leftDown,
    //vlxID::rightDown
    //vlxID::back

};

namespace kitID{
    constexpr uint8_t kRight=2;
    constexpr uint8_t kLeft=1;
    constexpr uint8_t kNone=0;
}
namespace rampID{
    constexpr uint8_t kUp=1;
    constexpr uint8_t kDown=2;
    constexpr uint8_t kNone=0;
}

#endif
