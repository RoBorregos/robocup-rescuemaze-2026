#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_LEFT  25
#define XSHUT_RIGHT 26
#define SDA_RIGHT   21
#define SCL_RIGHT   22
#define SDA_LEFT    32
#define SCL_LEFT    33

VL53L0X sensorLeft;
VL53L0X sensorRight;


volatile uint32_t tiempoA = 0;
volatile uint32_t tiempoB = 0;
volatile bool listoA = false;
volatile bool listoB = false;


void initVL53L0X(VL53L0X &sensor, TwoWire &wireBus, uint8_t xshutPin, uint8_t address = 0x29) {
  digitalWrite(xshutPin, LOW);
  delay(100);
  
  digitalWrite(xshutPin, HIGH);
  delay(150);

  sensor.setBus(&wireBus);
  
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (sensor.init()) {
      sensor.setTimeout(500);
      sensor.setMeasurementTimingBudget(50000);
      if (address != 0x29) {
        sensor.setAddress(address);
        delay(100); 
      }
      
   
      wireBus.beginTransmission(address);
      byte error = wireBus.endTransmission();
      if (error == 0) {
       
        return;
      } else {
        Serial.print(" Error: ");
        Serial.println(error);
      }
    } else {
      Serial.print("   Intento ");
      Serial.print(attempt);
      Serial.println(": Falló");
      delay(200);
    }
  }
  
  Serial.println(" INICIALIZACIÓN FALLIDA ");
}
int readVL53L0X(VL53L0X &sensor, TwoWire &wireBus, uint8_t address, const char* name) {
  static int errorCount[2] = {0, 0};
  int sensorIndex = (strcmp(name, "LEFT") == 0) ? 0 : 1;
  

  int dist = sensor.readRangeSingleMillimeters();
  
  if (sensor.timeoutOccurred()) {
    errorCount[sensorIndex]++;
    
    if (errorCount[sensorIndex] % 5 == 0) {
      Serial.print("[");
      Serial.print(name);
      Serial.print("] Error ");
      Serial.print(errorCount[sensorIndex]);
      

      wireBus.beginTransmission(address);
      if (wireBus.endTransmission() == 0) {
        sensor.init(); 
        errorCount[sensorIndex] = 0;
        Serial.print(name);
      }
    }
    return -1;
  }
  
  errorCount[sensorIndex] = 0;
  return dist;
}

void taskLeft(void *pv) {
  Serial.println("[LEFT] Task en Core 1 ");
  

  vTaskDelay(pdMS_TO_TICKS(1000));
  
  while (true) {
    int dist = readVL53L0X(sensorLeft, Wire1, 0x29, "LEFT");
    
    if (dist > 0 && dist < 2000) { 
      tiempoA = millis();
      listoA = true;
      
      Serial.print("[LEFT-C1] ");
      Serial.print(dist);
      Serial.println(" mm");
    } else if (dist != -1) {
      Serial.print("[LEFT] : ");
      Serial.println(dist);
    }
    
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}
void taskRight(void *pv) {
  Serial.println("[RIGHT] Task en Core 0 ");
  
  while (true) {
    int dist = readVL53L0X(sensorRight, Wire, 0x31, "RIGHT");
    
    if (dist > 0 && dist < 2000) { 
      tiempoB = millis();
      listoB = true;
      
      Serial.print("[RIGHT-C0] ");
      Serial.print(dist);
      Serial.println(" mm");
      if (listoA && listoB) {
        uint32_t dt = abs((int32_t)tiempoA - (int32_t)tiempoB);
        Serial.print("[DIF] ");
        Serial.print(dt);
        Serial.println(" ms");
        listoA = listoB = false;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}


void setup() {
  Serial.begin(115200);
  delay(5000);
  

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  Wire.begin(SDA_RIGHT, SCL_RIGHT);
  Wire.setClock(100000); // 100kHz
  Wire.setTimeout(2000);
  Serial.println("   Wire (GPIO21/22): OK");
  
  Wire1.begin(SDA_LEFT, SCL_LEFT);
  Wire1.setClock(100000); // 100kHz  
  Wire1.setTimeout(2000);
 
  
  initVL53L0X(sensorRight, Wire, XSHUT_RIGHT, 0x31);
  
  initVL53L0X(sensorLeft, Wire1, XSHUT_LEFT, 0x29);
  
  Serial.print("Wire > 0x31: ");
  Wire.beginTransmission(0x31);
  Serial.println(Wire.endTransmission() == 0 ? "OK" : "FALLO ");
  
  Serial.print("Wire1 0x29: ");
  Wire1.beginTransmission(0x29);
  Serial.println(Wire1.endTransmission() == 0 ? "OK " : "FALLO ");
  

  
  Serial.print("LEFT: ");
  int leftTest = sensorLeft.readRangeSingleMillimeters();
  if (!sensorLeft.timeoutOccurred()) {
    Serial.print(leftTest);
    Serial.println(" mm ");
  } else {
    Serial.println("TIMEOUT ");
  }
  
  Serial.print("RIGHT: ");
  int rightTest = sensorRight.readRangeSingleMillimeters();
  if (!sensorRight.timeoutOccurred()) {
    Serial.print(rightTest);
    Serial.println(" mm ");
  } else {
  }
  
  // Task RIGHT Core 0
  xTaskCreatePinnedToCore(
    taskRight,
    "TaskRight",
    4096,
    NULL,
    2,
    NULL,
    0
  );
  
  delay(500);
  
  // Task LEFT Core 1  
  xTaskCreatePinnedToCore(
    taskLeft,
    "TaskLeft",
    4096,
    NULL,
    1,
    NULL,
    1
  );
  
  Serial.println("bien");
}

void loop() {

  static unsigned long lastStatus = 0;
  
  if (millis() - lastStatus > 30000) { // Cada 30 segundos

    Serial.print("LEFT (0x29): ");
    Wire1.beginTransmission(0x29);
    bool leftAlive = (Wire1.endTransmission() == 0);
    Serial.println(leftAlive ? "Ok " : "EROR ");
    
    Serial.print("RIGHT (0x31): ");
    Wire.beginTransmission(0x31);
    bool rightAlive = (Wire.endTransmission() == 0);
    Serial.println(rightAlive ? "Ok " : "EROR ");
    
    if (!leftAlive || !rightAlive) {
      digitalWrite(XSHUT_LEFT, LOW);
      digitalWrite(XSHUT_RIGHT, LOW);
      delay(200);
      digitalWrite(XSHUT_LEFT, HIGH);
      digitalWrite(XSHUT_RIGHT, HIGH);
      delay(200);
      sensorLeft.init();
      sensorRight.init();
    }
    
    lastStatus = millis();
  }
  
  delay(1000);
}