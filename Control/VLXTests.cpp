#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>
#include "Pins_ID.h"
#define vDelay 20
#define MUX_ADDR 0x70

int Sensors_Amount = 7;
VLX sensors[Sensors_Amount] = {vlxID};
SemaphoreHandle_t i2cSemaphore;
MUX mux;
volatile unsigned long LastTime[Sensors_Amount] = {0};
volatile unsigned long CurrentTime[Sensors_Amount] = {};
volatile unsigned long DeltaTime[Sensors_Amount] = {};

void loop() {}

void setup(){
    Serial.begin(115200);
    Wire.begin();
    i2cSemaphore = xSemaphoreCreateMutex();
    if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
    }

    for (int i = 0; i < Sensors_Amount; i++) {
        sensors[i] = VLX(Pins::vlxPins[i]);
        sensors[i].begin();
    }

    xTaskCreatePinnedToCore(
    VLXTaskPriority1,
    "VLXTaskPriority1",
    4096,
    NULL,
    2,
    NULL,
    0);

    xTaskCreatePinnedToCore(
    VLXTaskPriority2,
    "VLXTaskPriority2",
    4096,
    NULL,
    1,
    NULL,
    1);

    xTaskCreatePinnedToCore(
    PrintDistances,
    "PrintDistances",
    2096,
    NULL,
    2,
    NULL,
    1);
}

void VLXTaskPriority1(void *pv)
{
    VL53L0X_RangingMeasurementData_t measure;

    while (true){
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
        for (int i: PriorityTaskID::Task1){   
            CurrentTime[i] = esp_timer_get_time();
            mux.selectChannel(Pins::vlxPins[i]);
            sensors[i].rangingTest(&measure, false);

            if (sensors[i].RangeStatus != 4){
                sensors[i].lastDistance = (float)measure.RangeMilliMeter / 10.0f;}
            DeltaTime[i] = CurrentTime[i] - LastTime[i];
            LastTime[i] = CurrentTime[i];
        }
        xSemaphoreGive(i2cSemaphore);
        vTaskDelay(pdMS_TO_TICKS(vDelay));
    }
}

void VLXTaskPriority2(void *pv){
    VL53L0X_RangingMeasurementData_t measure;

    while (true){
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

        for (int i: PriorityTaskID::Task2){
            mux.selectChannel(Pins::vlxPins[i]);
            sensors[i].rangingTest(&measure, false);
            if (sensors[i].RangeStatus != 4) {
                sensors[i].lastDistance = (float)measure.RangeMilliMeter / 10.0f;
            }
        }
        xSemaphoreGive(i2cSemaphore);
        vTaskDelay(pdMS_TO_TICKS(vDelay));
    }
}

void PrintDistances(void *pv){
    while (true){
        Serial.println("VLXPriority1 Task:");
        for (int i: PriorityTaskID::Task1){
            Serial.print("VLX ID ");
            Serial.print(i);
            Serial.print(": Distance: ");
            Serial.print(sensors[i].lastDistance);
            Serial.print(" cm ");
            Serial.print("Time taken: ");
            Serial.print(DeltaTime[i]);
            Serial.println();
        }

        Serial.println("VLXPriority2 Task:");
        for (int i: PriorityTaskID::Task2){
            Serial.print("VLX ID ");
            Serial.print(i);
            Serial.print(": Distance: ");
            Serial.print(sensors[i].lastDistance);
            Serial.print(" cm ");
            Serial.print("Time taken: ");
            Serial.print(DeltaTime[i]);
            Serial.println();
        }
        Serial.println("------------------------");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
