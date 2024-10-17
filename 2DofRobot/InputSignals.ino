float Step_Input(int step_time) { // step_time seconds
  static int increment = 0;
  static float u = u_amplitude;
  int step_cycles = step_time * 1e6/delta_time_micros; // Gets the number of cycles that is equivalent to the chosen step_time
  if (increment >= step_cycles) {
    u *= -1; // Reverse polarity after step time
    increment = 0;
  }
  increment++;
  return u;
}