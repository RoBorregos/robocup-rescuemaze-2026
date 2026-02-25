#include "RightHand.h"

void rightHandRule() {
    
    uint16_t distFrontLeft  = robot.vlx[vlxID::frontLeft].getDistance();
    uint16_t distFrontRight = robot.vlx[vlxID::frontRight].getDistance();
    uint16_t distFront      = (distFrontLeft + distFrontRight) / 2;
    uint16_t distRight      = robot.vlx[vlxID::rightUp].getDistance();
    uint16_t distLeft       = robot.vlx[vlxID::leftUp].getDistance();

    const uint16_t WALL_THRESHOLD = 20;

    bool frontFree = (distFront < WALL_THRESHOLD) ? false : true;
    bool rightFree = (distRight < WALL_THRESHOLD) ? false : true;
    bool leftFree  = (distLeft  < WALL_THRESHOLD) ? false : true;

    if (!frontFree && !rightFree && !leftFree)
    {
        Serial.println(F("Regla 4: deadEnd -> giro 180°"));
        robot.left();
        robot.left();
        robot.ahead();
    }

    else if (rightFree && leftFree && frontFree)
    {
        Serial.println("Adelante");
        robot.ahead();
    }

    else if (rightFree && leftFree && !frontFree)
    {
        robot.right();
        robot.ahead();
    }

    else if (rightFree && frontFree && !leftFree)
    {
        Serial.println(F("Regla 1: derecha libre -> giro derecha + avanzar"));
        robot.right();
        robot.ahead();
    }

    else if (frontFree && leftFree && !rightFree)
    {
        Serial.println(F("Regla 2: frente libre con pared a la derecha -> avanzar"));
        robot.ahead();
    }

    else if (!frontFree && rightFree && !leftFree)
    {
        robot.right();
        robot.ahead();
    }

    else if (!frontFree && !rightFree && leftFree)
    {
        Serial.println(F("Regla 3: pared enfrente -> giro izquierda + avanzar"));
        robot.left();
        robot.ahead();
    }
}