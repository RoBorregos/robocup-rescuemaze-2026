#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

SemaphoreHandle_t i2cSemaphore;

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

    xTaskCreate(
        RTOSDistance,
        "VLX_Task",
        2048,
        this,
        1,
        &taskHandle);
}

void VLX::setMux(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}
void VLX::updateDistance() {
    mux_.selectChannel();
    VLX_.rangingTest(&measure, false);
}

void VLX::RTOSDistance(void *pv)
{
    VLX *self = static_cast<VLX*>(pv);
    float dist;
    while (true)
    {
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE)
        {
            self->mux_.selectChannel();
            dist = self->VLX_.readRangeContinuousMillimeters() / 10.0f;
            xSemaphoreGive(self->i2cSemaphore);
            xQueueOverwrite(self->distanceQueue, &dist);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float VLX::getDistance()
{
    float distance = lastDistance;
    if (distanceQueue != NULL)
    {
        xQueuePeek(distanceQueue, &distance, 0);
        lastDistance = distance;
    }

    return distance;
}

void VLX::printDistance(){
    updateDistance();
    if (measure.RangeStatus != 4) {
        Serial.print("Distance: ");
        Serial.print(measure.RangeMilliMeter);
        Serial.println(" mm");
    } else {
        Serial.println("Fuera de rango.");
    }
    delay(500); 
    }
bool VLX::isWall(){
    if(getDistance()<kDistanceToWall){
        return true;
    }else{
        return false;
    }
}


