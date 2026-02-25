#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"
#include "motors.h"
#include "Pins_ID.h"

constexpr uint8_t Sensors_Amount = 5;
static constexpr uint32_t vDelay = 33;
SemaphoreHandle_t i2cSemaphore;

volatile unsigned long LastTime[Sensors_Amount] = {0};
volatile unsigned long CurrentTime[Sensors_Amount] = {0};
volatile unsigned long IntraDelta[Sensors_Amount] = {0};
volatile unsigned long LastReadTime = 0;
volatile unsigned long InterDelta[Sensors_Amount] = {0};

// Variables para el algoritmo de mano derecha
bool rightHandStarted = true;

// Declaración d//*  *//e funciones
void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);
void PrintDistances(void *pv);
void RightHandNavigationTask(void *pv);

// Función para implementar la regla de mano derecha
void rightHandRule() {
    
    // Obtener distancias de los sensores
    uint16_t distFrontLeft  = robot.vlx[vlxID::frontLeft].getDistance();
    uint16_t distFrontRight = robot.vlx[vlxID::frontRight].getDistance();
    uint16_t distFront      = (distFrontLeft + distFrontRight) / 2;
    uint16_t distRight      = robot.vlx[vlxID::rightUp].getDistance();
    uint16_t distLeft       = robot.vlx[vlxID::leftUp].getDistance();

    const uint16_t WALL_THRESHOLD = 20;

    bool frontFree = (distFront < WALL_THRESHOLD) ? false : true;
    bool rightFree = (distRight < WALL_THRESHOLD) ? false : true;
    bool leftFree  = (distLeft  < WALL_THRESHOLD) ? false : true;

    // Debug
    Serial.print("Frente: "); Serial.print(distFront);
    Serial.print(" cm | Derecha: "); Serial.print(distRight);
    Serial.print(" cm | Izquierda: "); Serial.print(distLeft);
    Serial.println(" cm");
    Serial.print("Front: "); Serial.print(distFront); Serial.print(" cm - ");
    Serial.println(frontFree ? "FREE" : "BLOCKED");
    Serial.print("Right: "); Serial.print(distRight); Serial.print(" cm - ");
    Serial.println(rightFree ? "FREE" : "BLOCKED");
    Serial.print("Left: ");  Serial.print(distLeft);  Serial.print(" cm - ");
    Serial.println(leftFree  ? "FREE" : "BLOCKED");
    Serial.println("---");

    // 4) Tres tapados -> 180°
    if (!frontFree && !rightFree && !leftFree)
    {
        Serial.println(F("Regla 4: deadEnd -> giro 180°"));
        robot.left();
        robot.left();
        robot.ahead();
    }
    // Todos libres -> adelante
    else if (rightFree && leftFree && frontFree)
    {
        Serial.println("Adelante");
        robot.ahead();
    }
    // Derecha e izquierda libres, frente bloqueado
    else if (rightFree && leftFree && !frontFree)
    {
        robot.right();
        robot.ahead();
    }
    // Derecha y frente libres, izquierda bloqueada
    else if (rightFree && frontFree && !leftFree)
    {
        Serial.println(F("Regla 1: derecha libre -> giro derecha + avanzar"));
        robot.right();
        robot.ahead();
    }
    // Frente e izquierda libres, derecha bloqueada
    else if (frontFree && leftFree && !rightFree)
    {
        Serial.println(F("Regla 2: frente libre con pared a la derecha -> avanzar"));
        robot.ahead();
    }
    // Solo derecha libre
    else if (!frontFree && rightFree && !leftFree)
    {
        robot.right();
        robot.ahead();
    }
    // Solo izquierda libre
    else if (!frontFree && !rightFree && leftFree)
    {
        Serial.println(F("Regla 3: pared enfrente -> giro izquierda + avanzar"));
        robot.left();
        robot.ahead();
    }
}
int servopos = 0;

void setup() {
  Serial.begin(115200);
  
  Serial.println("\n\n=================================");
  Serial.println("  ROBOT MANO DERECHA - INICIO");
  Serial.println("=================================\n");
  
  robot.setupMotors();
  
  // Configurar interrupciones para encoders
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), 
                  Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), 
                  Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), 
                  Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), 
                  Interrups::backRightEncoder, RISING);
  
  // Crear semáforo para I2C
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
  }
  Serial.println("Semaphore created successfully.");
  
  //  Aumentar delay para estabilización
  delay(500);
  
  // Calibrar orientación inicial
  robot.resetOrientation();
  
  if (robot.innit == true) {
    Serial.println("Verificando sensores VLX antes de crear tareas...");
    
    //  VERIFICACIÓN CRÍTICA: Probar cada sensor
    for (uint8_t i = 0; i < Sensors_Amount; i++) {
      Serial.print("  Sensor VLX["); 
      Serial.print(i); 
      Serial.print("]: ");
      
      // Intentar lectura
      uint16_t dist = robot.vlx[i].getDistance();
      Serial.print(dist);
      Serial.println(" cm");
      delay(50);
    }
    
    Serial.println("\nCreando tareas FreeRTOS...");
    
    //  Tarea para sensores VLX prioritarios (frente) - Stack aumentado
    BaseType_t result1 = xTaskCreatePinnedToCore(
      VLXTaskPriority1, 
      "VLXTaskPriority1", 
      8192,  //  Aumentado de 4096 a 8192
      NULL, 
      2, 
      NULL, 
      0
    );
    if (result1 != pdPASS) {
      Serial.println("ERROR: No se pudo crear VLXTaskPriority1");
    }

    // Tarea para sensores VLX secundarios - Stack aumentado
    BaseType_t result2 = xTaskCreatePinnedToCore(
      VLXTaskPriority2, 
      "VLXTaskPriority2", 
      8192,  // Aumentado de 4096 a 8192
      NULL, 
      2, 
      NULL, 
      1
    );
    if (result2 != pdPASS) {
      Serial.println("ERROR: No se pudo crear VLXTaskPriority2");
    }
    
    // Tarea para imprimir distancias - Stack aumentado
    BaseType_t result3 = xTaskCreatePinnedToCore(
      PrintDistances, 
      "PrintDistances", 
      3072,  // Aumentado de 2096 a 3072
      NULL, 
      1, 
      NULL, 
      1
    );
    if (result3 != pdPASS) {
      Serial.println("ERROR: No se pudo crear PrintDistances");
    }
    
    // Tarea para navegación - Stack aumentado
    BaseType_t result4 = xTaskCreatePinnedToCore(
      RightHandNavigationTask, 
      "RightHandNav", 
      6144,  // Aumentado de 4096 a 6144
      NULL, 
      3, 
      NULL, 
      0
    );
    if (result4 != pdPASS) {
      Serial.println("ERROR: No se pudo crear RightHandNavigationTask");
    }
    
    Serial.println("Todas las tareas creadas exitosamente.");
  } else {
    Serial.println("ERROR: robot.innit = false");
  }
  
  Serial.println("\n=================================");
  Serial.println("ROBOT LISTO - INICIANDO EN 3 SEG");
  Serial.println("=================================\n");
   // Dar tiempo para posicionar el robot
}


// Tarea para actualizar sensores VLX prioritarios (frontales)
void VLXTaskPriority1(void *pv) {
  // Delay inicial para asegurar que todo esté listo
  vTaskDelay(pdMS_TO_TICKS(500));
  
  Serial.println("VLXTaskPriority1 iniciada");
  
  while (true) {
    // Timeout en semáforo para evitar bloqueos permanentes
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      //tileColor=tcs_.getColor();

      for (uint8_t id : TaskVLX1) {
        // VALIDACIÓN CRÍTICA: Verificar índice
        if (id >= Sensors_Amount) {
          Serial.print("ERROR VLXTask1: ID inválido: ");
          Serial.println(id);
          continue;
        }
        
        unsigned long now = esp_timer_get_time() / 1000; 
        IntraDelta[id] = now - LastTime[id];
        LastTime[id] = now;
        
        // Protección adicional
        robot.vlx[id].updateDistance();
        
        InterDelta[id] = now - LastReadTime;
        LastReadTime = now;
      }
      xSemaphoreGive(i2cSemaphore);
    } else {
      // Timeout - semáforo no disponible
      Serial.println("WARN: VLXTask1 timeout en semáforo");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

// Tarea para actualizar sensores VLX secundarios
void VLXTaskPriority2(void *pv) {
  // Delay inicial
  vTaskDelay(pdMS_TO_TICKS(600));
  
  Serial.println("VLXTaskPriority2 iniciada");
  
  while (true) {
    // Timeout en semáforo
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t id : TaskVLX2) {
        // VALIDACIÓN CRÍTICA
        if (id >= Sensors_Amount) {
          Serial.print("ERROR VLXTask2: ID inválido: ");
          Serial.println(id);
          continue;
        }
        
        unsigned long now = esp_timer_get_time() / 1000;
        IntraDelta[id] = now - LastTime[id];
        LastTime[id] = now;
        
        robot.vlx[id].updateDistance();
        
        InterDelta[id] = now - LastReadTime;
        LastReadTime = now;
      }
      xSemaphoreGive(i2cSemaphore);
    } else {
      Serial.println("WARN: VLXTask2 timeout en semáforo");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

// Tarea para imprimir distancias (debug)

void PrintDistances(void *pv) {
  // Delay inicial
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  while (true) {
    /*
    Serial.println("\n========== SENSORES VLX ==========");
    Serial.println("VLX Priority 1 (Frontales):");
    for (uint8_t id : TaskVLX1) {
      // Validación
      if (id >= Sensors_Amount) continue;
      
      Serial.print("  VLX["); Serial.print(id);
      Serial.print("]: "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraΔ: "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterΔ: "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }

    Serial.println("VLX Priority 2 (Laterales/Trasero):");
    for (uint8_t id : TaskVLX2) {
      // Validación
      if (id >= Sensors_Amount) continue;
      
      Serial.print("  VLX["); Serial.print(id);
      Serial.print("]: "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraΔ: "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterΔ: "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }
  
    Serial.println("===================================\n");
    */
    vTaskDelay(pdMS_TO_TICKS(2000)); // Imprimir cada 2 segundos
  }
}

// Tarea principal para navegación con mano derecha
void RightHandNavigationTask(void *pv) {
  // Delay inicial mayor para que los sensores se estabilicen
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  Serial.println("\n*** NAVEGACIÓN CON MANO DERECHA INICIADA ***\n");
  
  while (true) {
    // Imprimir estado actual
    /*
    Serial.println("\n--- Estado actual ---");
    Serial.print("Frente L: "); Serial.print(robot.vlx[vlxID::frontLeft].getDistance());
    Serial.print(" cm | Frente R: "); Serial.print(robot.vlx[vlxID::frontRight].getDistance());
    Serial.println(" cm");
    Serial.print("Derecha: "); Serial.print(robot.vlx[vlxID::rightUp].getDistance());
    Serial.print(" cm | Izquierda: "); Serial.print(robot.vlx[vlxID::leftUp].getDistance());
    Serial.println(" cm");
    Serial.print("Atrás: "); Serial.print(robot.vlx[vlxID::back].getDistance());
    Serial.println(" cm");
    */
    
    // Ejecutar lógica de mano derecha
     rightHandRule();
    
    // Esperar antes de la siguiente decisión
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void loop() {
  //robot.calibrateColors();
  //robot.calibrateColors();
  //Serial.println(robot.motor[MotorID::kBackRight].tics);
  //robot.calibrateColors();
    //robot.checkTileColor();
  //delay(1000);
  /*
  robot.ahead();
  robot.smoothRotate(90);
  delay(5000);
  robot.ahead();
  robot.ahead();
  robot.right();
  delay(5000);
  */
}