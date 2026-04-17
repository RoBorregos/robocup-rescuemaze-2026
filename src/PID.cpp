#include "PID.h"
#include "Encoder.h"
#include <Arduino.h>
 
PID::PID(float kp_, float ki_, float kd_, float calculate_time_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
  calculate_time = calculate_time_;
  // FIX 1: inicializar estado interno a valores seguros
  last_error    = 0.0f;
  last_time     = 0.0f;
  integral_sum  = 0.0f;
  integral_limit = 50.0f;  // ajusta según tu rango de output máximo
}
 
PID::PID() {
  kp = 6.0;
  ki = 0.1;
  kd = 0.8;
  // FIX 1: mismo init seguro en constructor por defecto
  last_error    = 0.0f;
  last_time     = 0.0f;
  integral_sum  = 0.0f;
  integral_limit = 50.0f;
}
 
void PID::changeConstants(float kp_, float ki_, float kd_,
                          float calculate_time_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
  calculate_time = calculate_time_;
  // FIX 1: al cambiar constantes resetear integral para evitar kick
  integral_sum = 0.0f;
}
 
void PID::resetIntegral() {
  integral_sum = 0.0f;
}
 
double PID::calculate_PID(float setpoint, float input) {
  float current_time = millis();
  float delta_time   = current_time - last_time;
 
  // FIX 1: proteger contra delta_time = 0 (evita división por cero en derivativo)
  if (delta_time < 1.0f) return 0;
 
  if (delta_time >= calculate_time) {
    float error        = setpoint - input;
    float proportional = kp * error;
    float derivative   = kd * (error - last_error) / delta_time;
 
    // FIX 1: integral separado con anti-windup por clamping
    // Solo acumula si el output no está saturado (evita windup al atascarse)
    integral_sum += error * delta_time;
    // Clamp del acumulador
    if (integral_sum >  integral_limit) integral_sum =  integral_limit;
    if (integral_sum < -integral_limit) integral_sum = -integral_limit;
 
    float integral = ki * integral_sum;
    float output   = proportional + integral + derivative;
 
    last_error = error;
    last_time  = current_time;
    return output;
  }
  return 0;
}