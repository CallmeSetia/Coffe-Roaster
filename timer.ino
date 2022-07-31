void timerStart() {
  time_state = true;
}
void timerStop() {
  time_state = false;
}
void timerReset() {
  time_state = false;
  t_second = 0;
  t_minute = 0;
}
void timerCallback() {
  if (time_state) {
    t_second += 1;
    if (t_second == 60) {
      t_minute += 1;
      t_second = 0;
    }
  }
}
