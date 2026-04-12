#include "PID.h"
#include "Encoder.h"
#include <Arduino.h>
PID::PID(float kp_, float ki_, float kd_, float calculate_time_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
  calculate_time = calculate_time_;
  last_error = 0.0f;
  last_time = 0.0f;
  accumulated_error = 0.0f;
}
PID::PID() {
  kp = 6.0;
  ki = 0.1;
  kd = 0.8;
  calculate_time = 20;
  last_error = 0.0f;
  last_time = 0.0f;
  accumulated_error = 0.0f;
}
void PID::changeConstants(float kp_, float ki_, float kd_,
                          float calculate_time_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
  calculate_time = calculate_time_;
  accumulated_error = 0.0f;  // Reset integral on reconfiguration
}
double PID::calculate_PID(float setpoint, float input) {
  float current_time = millis();
  if (last_time == 0.0f) {
    last_time = current_time;
    return 0;
  }
  float delta_time = current_time - last_time;
  if (delta_time >= calculate_time) {
    float error = setpoint - input;
    accumulated_error += error;
    accumulated_error = constrain(accumulated_error, -MAX_INTEGRAL, MAX_INTEGRAL);

    float proportional = kp * error;
    float integral = ki * accumulated_error;
    float derivative = kd * (error - last_error) / max(delta_time, 1.0f);
    float output = proportional + integral + derivative;
    last_error = error;
    last_time = current_time;
    return output;
  }
  return 0;
}
