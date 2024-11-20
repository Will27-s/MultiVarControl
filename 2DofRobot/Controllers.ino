#include <cmath>

// Contains all the controllers

float Bang_Bang_Control(int e, int u_amplitude) {
  float u = 0;
  if (e < 0 ) {
    u = -1 * u_amplitude;
  }
  if (e > 0) {
    u = u_amplitude;
  }
  return u;
}

float Proportional_Control(int e, float kp) {
  float u;
  u = kp * e;
  u = set_u_to_max_if_out_of_bounds(u);
  return u;
}

float filter(float u, float u_prev) {
  float filtered_u;
  filtered_u = (alpha * u_prev) + ((1-alpha) * u);
  return filtered_u;
}

float PID_Control(int e, int e_sum, int e_prev, float kp, float ki, float kd) {
  float u;
  float proportional = kp * e;
  // derivative is noisy
  float integral = ki * e_sum * delta_time_seconds;;
  float derivative = kd * (e - e_prev)/delta_time_seconds;

  
  u = proportional + (0.99 * integral) + derivative;
  u = set_u_to_max_if_out_of_bounds(u);

  return u;
}


float set_u_to_max_if_out_of_bounds(float u) {
  if (u > 255) {
    u = 255;
  } 
  if ( u < -255) {
    u = -255;
  }
  return u;
}


