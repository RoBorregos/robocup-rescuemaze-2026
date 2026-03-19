#include <Arduino.h>
#include "Encoder.h"
#include "maze.h"
#include "Test.h"
#include "raspy.h"

void setup() {
  Serial.begin(115200);
  delay(500);
  
  robot.screenBegin();
  robot.screenPrint("INIT");
  Serial.println("=== RoboCup Rescue Maze ===");
  Serial.println("Connecting to Raspberry Pi...");
  
  raspy.connect();
  
  robot.screenPrint("READY");
  Serial.println("Connected. getDetection() ready.");
}

void loop() {
  raspy.getDetection();
}
