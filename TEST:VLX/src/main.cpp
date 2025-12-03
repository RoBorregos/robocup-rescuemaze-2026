#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_LEFT  25
#define XSHUT_RIGHT 26

VL53L0X sensorLeft;
VL53L0X sensorRight;

void TaskLeft(void *pv) {
  for (;;) {
    int d = sensorLeft.readRangeSingleMillimeters();
    Serial.print("[LEFT] ");
    Serial.print(d);
    Serial.print(" mm  | CORE: ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskRight(void *pv) {
  for (;;) {
    int d = sensorRight.readRangeSingleMillimeters();
    Serial.print("[RIGHT] ");
    Serial.print(d);
    Serial.print(" mm | CORE: ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(pdMS_TO_TICKS(50));
    delay(5000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.init()) {
    Serial.println("ERROR INIT LEFT");
    while (1);
  }
  sensorLeft.setAddress(0x30);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.init()) {
    Serial.println("ERROR INIT RIGHT");
    while (1);
  }
  sensorRight.setAddress(0x31);

  xTaskCreatePinnedToCore(TaskLeft, "LeftTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskRight,"RightTask",4096, NULL, 1, NULL, 1);
}

void loop() {}