#include "VLX.h"
#include <Arduino.h>
#include <Wire.h>

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
        Serial.println("Error initializing VL53L0X sensor.");
        while (1);
    }
    Serial.println("VL53L0X initialized correctly.");  
    VLX_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget); //Precision mode
    VLX_.startRanging();
}

/*example:

void VLX::VLXTask(void *pv)
{
    VL53L0X_RangingMeasurementData_t measure;

    while (true)
    {
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

        for (int i: PriorityTaskID::Type tasksID)
        {
            mux.selectChannel(i);
            vlx_hw.rangingTest(&measure, false);

            if (vlx_hw.RangeStatus != 4) 
            {
                vlx[i].lastDistance = (float)measure.RangeMilliMeter / 10.0f;
            }
        }
        xSemaphoreGive(i2cSemaphore);
        vTaskDelay(pdMS_TO_TICKS(vDelay));
    }
}
*/
float VLX::getDistance() const {
    return lastDistance;
}

bool VLX::isWall() const{
    return (getDistance()<kDistanceToWall);
}