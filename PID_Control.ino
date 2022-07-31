void PID_Control(float _SetPoint, float _Input) {

  //Pembacaan error
  PID_error  = _SetPoint - _Input + 3;

  //bagian Proportional
  PID_p = 0.01 * kp  * PID_error ;

  //bagian Integral
  PID_i = 0.01 * PID_i + (ki * PID_error);

  //bagian D
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  //Now we can calculate the D calue
  PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);

  //Total Value PID
  PID_value = PID_p + PID_i + PID_d;

  if (PID_value > 1023) PID_value = 1023;
  else if (PID_value < 0) PID_value = 0;

  previous_error = PID_error;
  out = map(PID_value, 0, 1023, 0, 90); //manipulated variable 0 - 100 (0 - 60 derajat)
}
void PID_readStatus() {
  Serial.println("SetPoint = " + String(SP));
  Serial.println("PID Out = " + String(PID_value));
  Serial.println("Out Map = " + String(out));
  Serial.println("Waktu = " + String(t_minute) + " : " + String(t_second));
}
