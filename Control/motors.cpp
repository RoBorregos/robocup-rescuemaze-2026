#include "Pins_ID.h"
#include <WiFi.h>
#include "Jetson.h"
Jetson jetson;

motors::motors(){
}

void motors::PID_selfCenter(uint16_t reference_speed)
{
    float Left = vlx[vlxID::left].getDistance();
    float Right = vlx[vlxID::right].getDistance();
    CenterPID.changeConstants(kP_Center,kI_Center,kD_Center,CenterTime); //To define
    double output=CenterPID.calculate_PID(Right,Left);
    int right_speed=reference_speed-output;
    int left_speed=reference_speed+output;
    right_speed=constrain(right_speed,0,255);
    left_speed=constrain(left_speed,0,255);
    for(int i=0;i<4;i++){
        motor[i].setSpeed((i%2==0) ? left_speed:right_speed);
    }
    delay(20);
}

bool motors::rampInFront(){
    Readings[] readings;
    
    if((vlx[vlxID::frontLeft].getDistance()-vlx[vlxID::front].getDistance())>=2){
    // If we do add a front lower sensors, then: && vlx[vlxID::front].getDistance() < vlx[vlxID::frontCenter].getDistance())
        for (int i=0;i<3;i++){
            readings[i]=vlx[vlxID::front].getDistance();
            delay(20);
        }
        if((abs(readings[0]-readings[1])>=2) && (abs(readings[1]-readings[2])>=2) && (abs(readings[0]-readings[2])>=2)){
            return true;
        }
    }else{
        return false;
    }
}