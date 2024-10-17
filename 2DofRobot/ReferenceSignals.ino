int rotate_every_time_per_rotation_ref_signal() {
// Set a reference angle position for the motor. It will be in counts, so conversion is needed to degrees for output.
// Will change by +360 degrees every time_per_rotation milliseconds  
  if ((millis() - time_start) > time_per_rotation) {
    rotations++;
    time_start = millis();
  }
  return ref_amplitude * rotations;
}