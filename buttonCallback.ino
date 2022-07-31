void lightButton_isPressed() {
  SP = 100; /* Suhu */
  SP_Time = 1; /* Menit */
  if (t_minute == SP_Time) {
    timerReset(); SP = 0;
    _reset = 1;
  }
}
void brownButton_isPressed() {
  SP = 200; /* Suhu */
  SP_Time = 1; /* Menit */
  if (t_minute == SP_Time) {
    timerReset(); SP = 0;
    _reset = 1;
  }
}
void darkButton_isPressed() {
  SP = 300; /* Suhu */
  SP_Time = 1; /* Menit */
  if (t_minute == SP_Time) {
    timerReset(); SP = 0;
    _reset = 1;
  }
}
