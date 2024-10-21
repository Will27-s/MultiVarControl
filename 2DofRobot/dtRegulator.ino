void dt_regulator(long delta_time_micros) { // Regulates time so that length of loop is the same
  int current_loop_time = 0;
  do {
    current_loop_time = micros() - loop_start_time;
    ATOMIC() {  // lines between these brackets are executed even if an interrupt occurs
    motor1.set_pos_with_posi();
    motor2.set_pos_with_posi();
  }
  } while (current_loop_time < delta_time_micros);
}





