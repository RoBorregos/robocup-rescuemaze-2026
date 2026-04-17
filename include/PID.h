#ifndef PID_H
#define PID_H

class PID {
private:
  float kp;
  float ki;
  float kd;
  float last_error;
  float last_time;
  float integral_sum;          // FIX 1: acumulador separado para anti-windup
  float integral_limit;        // FIX 1: límite del integral

public:
  PID(float, float, float, float);
  PID();
  double calculate_PID(float, float);
  void changeConstants(float, float, float, float);
  void resetIntegral();        // FIX 1: permite resetear el integral externamente
  float calculate_time = 20;
};

#endif