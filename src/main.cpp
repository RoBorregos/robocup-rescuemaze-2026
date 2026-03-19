#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"
#include "motors.h"
#include "Pins_ID.h"
#include "raspy.h"


SemaphoreHandle_t i2cSemaphore;
static constexpr uint32_t kVlxTaskDelayMs = 40;

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);

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
  raspyLink.setup();
  
  if (true) {
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
    Serial.println("All tasks created successfully.");
  } else {
    Serial.println("ERROR: setup skipped");
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
    vTaskDelay(pdMS_TO_TICKS(kVlxTaskDelayMs));
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
    vTaskDelay(pdMS_TO_TICKS(kVlxTaskDelayMs));
  }
}


void loop() {
  raspyLink.getDetection();
}