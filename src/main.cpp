#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"
#include "motors.h"
#include "Pins_ID.h"
#include "RightHand.h"


SemaphoreHandle_t i2cSemaphore;

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);
void RightHandNavigationTask(void *pv);

int servopos = 0;

void setup() {
  Serial.begin(115200);
  robot.setupMotors();
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), 
                  Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), 
                  Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), 
                  Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), 
                  Interrups::backRightEncoder, RISING);
  
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
  }
  Serial.println("Semaphore created successfully.");
  delay(500);
  robot.resetOrientation();
  
  if (robot.innit == true) {    
    BaseType_t result1 = xTaskCreatePinnedToCore(
      VLXTaskPriority1, 
      "VLXTaskPriority1", 
      8192,
      NULL, 
      2, 
      NULL, 
      0
    );
    if (result1 != pdPASS) {
      Serial.println("ERROR: Couldn't create VLXTaskPriority1");
    }

    BaseType_t result2 = xTaskCreatePinnedToCore(
      VLXTaskPriority2, 
      "VLXTaskPriority2", 
      8192,
      NULL, 
      2, 
      NULL, 
      1
    );
    if (result2 != pdPASS) {
      Serial.println("ERROR: Couldn't create VLXTaskPriority2");
    }
    
    
    BaseType_t result4 = xTaskCreatePinnedToCore(
      RightHandNavigationTask, 
      "RightHandNav", 
      6144,
      NULL, 
      3, 
      NULL, 
      0
    );
    if (result4 != pdPASS) {
      Serial.println("ERROR: Couldn't create RightHandNavigationTask");
    }
    
    Serial.println("All tasks created successfully.");
  } else {
    Serial.println("ERROR: robot.innit = false");
  }
}

void VLXTaskPriority1(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(500));
  
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t id : TaskVLX1) { robot.vlx[id].updateDistance();}
      xSemaphoreGive(i2cSemaphore);
    } else {
      Serial.println("WARN: VLXTask1 timeout in semaphore");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

void VLXTaskPriority2(void *pv) {
  // Delay inicial
  vTaskDelay(pdMS_TO_TICKS(600));  
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t id : TaskVLX2) { robot.vlx[id].updateDistance();}
      xSemaphoreGive(i2cSemaphore);
    } else {
      Serial.println("WARN: VLXTask2 timeout in semaphore");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

void RightHandNavigationTask(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  Serial.println("\n*** Right Hand Rule ***\n");
  
  while (true) {
  
    Serial.println("\n--- Current reading ---");
    Serial.print("Front L: "); Serial.print(robot.vlx[vlxID::frontLeft].getDistance());
    Serial.print(" cm | Front R: "); Serial.print(robot.vlx[vlxID::frontRight].getDistance());
    Serial.println(" cm");
    Serial.print("Right: "); Serial.print(robot.vlx[vlxID::rightUp].getDistance());
    Serial.print(" cm | Left: "); Serial.print(robot.vlx[vlxID::leftUp].getDistance());
    Serial.println(" cm");
    Serial.print("Back: "); Serial.print(robot.vlx[vlxID::back].getDistance());
    Serial.println(" cm");

    rightHandRule(); //comment out if not using
    
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void loop() {

}