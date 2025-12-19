#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

SemaphoreHandle_t i2cSemaphore;
static MUX mux;

VLX::VLX(){
}

VLX::VLX(const uint8_t posMux){
    mux_.setNewChannel(posMux);
}

void VLX::begin(){
    if (i2cSemaphore == NULL) {i2cSemaphore = xSemaphoreCreateMutex();}
    mux_.selectChannel();
    VLX_.begin();
    if (!VLX_.begin()) {
        Serial.println("¡Error al iniciar el sensor VL53L0X!");
        while (1);
    }
    Serial.println("VL53L0X iniciado correctamente.");  
    VLX_.setMeasurementTimingBudgetMicroSeconds(33000);
    VLX_.startRanging();
}

void VLXTask1(void *pv)
{
    VL53L0X_RangingMeasurementData_t measure;

    while (true)
    {
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

        for (int i: PriorityTaskID::Task1)
        {
            mux.selectChannel(i);
            vlx_hw.rangingTest(&measure, false);

            if (vlx_hw.RangeStatus != 4) 
            {
                vlx[i].lastDistance = (float)measure.RangeMilliMeter / 10.0f;
            }
        }
        xSemaphoreGive(i2cSemaphore);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void VLXTask2(void *pv)
{
    VL53L0X_RangingMeasurementData_t measure;

    while (true)
    {
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

        for (int i: PriorityTaskID::Task2)
        {
            mux.selectChannel(i);
            vlx_hw.rangingTest(&measure, false);

            if (vlx_hw.RangeStatus != 4) 
            {
                vlx[i].lastDistance = (float)measure.RangeMilliMeter / 10.0f;
            }
        }
        xSemaphoreGive(i2cSemaphore);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float VLX::getDistance() const 
{
    return lastDistance;
}

bool VLX::isWall(){
    if(getDistance()<kDistanceToWall){
        return true;
    }else{
        return false;
    }
}


