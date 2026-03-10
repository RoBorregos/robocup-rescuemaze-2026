
#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"

constexpr uint8_t Sensors_Amount=8;
static constexpr uint32_t vDelay = 33;
SemaphoreHandle_t i2cSemaphore;

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);


int servopos=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.setupMotors();
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), Interrups::backRightEncoder, RISING);
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
    }
  Serial.println("Semaphore created successfully.");
  delay(3000); 
  if (robot.innit) {
    xTaskCreatePinnedToCore(
      VLXTaskPriority1, "VLXTaskPriority1", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(
      VLXTaskPriority2, "VLXTaskPriority2", 4096, NULL, 1, NULL, 1);

  }
  // robot.reloadKits();
}

void loop() {
  //testEncoders();
  //testPIDWheel();
  // // robot.leds.sequency();

  // robot.kitLeft(1);
  // delay(1000);
  // robot.kitRight(1);
  // delay(1000);

  // robot.ahead();
  // testTCS();
  // robot.setahead();
  // robot.setSpeed(50);
  /*
  int targetAngle = 90;
  robot.rotate(targetAngle);
  delay(500);
  targetAngle += 90;
  robot.rotate(targetAngle);
  delay(500);
  */
  // jeetson.getDetection();
  // delay(300);
  // testButton();
  //robot.ahead();
  //robot.setSpeed(40);
  //robot.moveDistance(30/6, true);
  //delay(10000);
  //robot.moveDistance(50, true);
  //delay(5000);
  //testPIDWheel();
  /*testPIDWheel();
  delay(500);
  pidTest();
  delay(500);
  robot.right();
  delay(1000);
  robot.stop();
  while(1);
  */
  // calibrateColors();
  // robot.checkpointElection(); 
  // robot.buttonPressed=false;
  // testTCS();
  // testLimits();
  // testBnoY();
  
  // testVlxFrontDistance();
  // testVlxFrontLeft();
  // testVlxFrontRigth();
  // testVlxRight();
  // testVlxLeft();
  // testVlxFront();
  // testVlxBack();
}

void VLXTaskPriority1(void *pv) {
  while (true) {
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    for (uint8_t id : TaskVLX1) {
      robot.vlx[id].updateDistance();
    }
    xSemaphoreGive(i2cSemaphore);
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}


void VLXTaskPriority2(void *pv) {
  while (true) {
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    for (uint8_t id : TaskVLX2) {
      robot.vlx[id].updateDistance();
    }
    xSemaphoreGive(i2cSemaphore);
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

