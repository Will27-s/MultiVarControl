// Interrupt service routine 
void readEncoderMot1() {
  int b = digitalRead(motor1.get_ENCB_Pin());
  if (b > 0) {
    motor1.increment_posi();
  } else {
    motor1.decrement_posi();
  }
}
void readEncoderMot2() {
  int b = digitalRead(motor2.get_ENCB_Pin());
  if (b > 0) {
    motor2.increment_posi();
  } else {
    motor2.decrement_posi();
  }
}