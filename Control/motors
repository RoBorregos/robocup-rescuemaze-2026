#include "motors.h"
#include "Pins_ID.h"
#include <WiFi.h>
#include "Jetson.h"
Jetson jettson;

motors::motors(){
}

void motors::PID_selfCenter(uint16_t reference_speed)
{
    float Left = vlx[vlxID::left].getDistance();
    float Right = vlx[vlxID::right].getDistance();
    CenterPID.changeConstants(kP_Center,kI_Center,kD_Center,CenterTime);
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

float motors::nearWall(){
    vlx[vlxID::left].getDistance();
    vlx[vlxID::right].getDistance();
    if(vlx[vlxID::left].distance<minDisToLateralWall ){
        changeAngle=maxChangeAngle;
    }else if(vlx[vlxID::right].distance<minDisToLateralWall){
        changeAngle=-maxChangeAngle;
    }else{
        changeAngle=0;
    }
    return changeAngle;
}