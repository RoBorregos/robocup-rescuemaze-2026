#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

SemaphoreHandle_t syncSemaphore;
QueueHandle_t distanceQueue;
distanceQueue = xQueueCreate(1, sizeof(float));

VLX::VLX(){
}

VLX::VLX(const uint8_t posMux){
    mux_.setNewChannel(posMux);
}

void VLX::begin(){
    mux_.selectChannel();
    VLX_.begin();
    if (!VLX_.begin()) {
        Serial.println("¡Error al iniciar el sensor VL53L0X!");
        while (1);
    }
    Serial.println("VL53L0X iniciado correctamente.");  
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
    float dist;
    while (true)
    {
        xSemaphoreTake(syncSemaphore, portMAX_DELAY);
        float dist = VLX_.readRangeContinuousMillimeters();
        xSemaphoreGive(syncSemaphore);
        dist /= 10;
        xQueueOverwrite(distanceQueue, &dist);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float VLX::getDistance(){
    updateDistance();
    if (measure.RangeStatus != 4) 
    {
    float distance;
    xQueuePeek(distanceQueue, &distance, portMAX_DELAY);
    return distance;
    }
    else{
        updateDistance();
        if (measure.RangeStatus != 4) {
        float distance;
        xQueuePeek(distanceQueue, &distance, portMAX_DELAY);
        return distance;
        }
    }
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


