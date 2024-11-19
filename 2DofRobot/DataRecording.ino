void record_pos_data() {
  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
  Serial.print(motor1.get_u());
  Serial.print(",");
  Serial.print(motor1.get_pos());
  Serial.print(",");
  Serial.print(motor1.get_error());
  Serial.print(",");
  Serial.print(ref1);
  Serial.print(",");
  Serial.print(motor2.get_u());
  Serial.print(",");
  Serial.print(motor2.get_pos());
  Serial.print(",");
  Serial.print(motor2.get_error());
  Serial.print(",");
  Serial.println(ref2);
  }
}

void serial_plotting() {
  // print target and position to see the response every print_interval times around the loop
  interval_count = interval_count + 1;
  if (interval_count >= print_interval) {
    interval_count = 0;
    Serial.print(ref1);
    Serial.print(" ");
    Serial.print(motor1.get_pos());
    Serial.print(" ");
    Serial.print(motor2.get_pos());
    Serial.print(" ");
    Serial.print(ref2);
    Serial.print(" ");
    Serial.println();
   
  }
}
