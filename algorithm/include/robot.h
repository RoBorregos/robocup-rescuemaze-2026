
#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

// Kit types for victim rescue
enum class kitID {
    kNone = 0,
    kHarmed = 1,
    kStable = 2,
    kUnharmed = 3
};

// Pin definitions (modify according to your wiring)
namespace Pins {
    const int checkpointPin = 2;  // Example pin for checkpoint button
}

class Robot {
public:
    Robot() : 
        buttonPressed(false),
        victim(0),
        checkpoint(false),
        blackTile(false),
        blueTile(false),
        rampState(0),
        kitState(kitID::kNone) {}
    
    // (implement these)
    void rotate(int degrees) {
        // TODO: Implement rotation
        
    }
    
    void ahead() {
        // TODO: Implement forward movement
        // Move robot forward one tile
    }
    
    
    bool isWall(int direction) {
        // TODO: Implement wall detection
        // Return true if wall detected in that direction
        return false;
    }
    
    void harmedVictim() {
        // TO DO: Handle harmed victim (red)
        screenPrint("Harmed Victim!");
    }
    
    void stableVictim() {
        // TO DO: Handle stable victim (yellow)
        screenPrint("Stable Victim!");
    }
    
    void unharmedVictim() {
        // TODO: Handle unharmed victim (green)
        screenPrint("Unharmed Victim!");
    }
    
    //  UTILITY FUNCTIONS (implement these) 
    void screenPrint(const char* message) {
        // TODO: Print to screen/serial
        Serial.println(message);
    }
    
    void resetOrientation() {
        // TO DO: Reset robot orientation to initial state
    }
    
    void resetVlx() {
        // TO DO: Reset VL53L0X sensors
    }
    
    void checkpointElection() {
        // TO DO: Handle checkpoint selection
    }
    
    //  PUBLIC VARIABLES (used by maze algorithm) 
    bool buttonPressed;    // Emergency stop button state
    int victim;           // Victim type detected (0=none, 1=harmed, 2=stable, 3=unharmed)
    bool checkpoint;      // Checkpoint detected flag
    bool blackTile;       // Black tile (hole) detected flag
    bool blueTile;        // Blue tile (speed bump) detected flag
    int rampState;        // Ramp state (0=none, 1=up, -1=down)
    kitID kitState;       // Current rescue kit state
};
// Global robot instance
extern Robot robot;

#endif // ROBOT_H