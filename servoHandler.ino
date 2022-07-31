void Servo_Init() {
  servo1.attach(33);
  servo1.write(90);
}
void servoBuka() {
  servo1.write(0);
}
void servoTutup() {
  servo1.write(90);
}
void servoHeater(int _input) {
  servo1.write(90 - _input);
}
