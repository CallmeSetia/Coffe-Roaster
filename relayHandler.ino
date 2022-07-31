void relay_Init() {
  pinMode(relay[0], OUTPUT);
  pinMode(relay[1], OUTPUT);
  digitalWrite(relay[0], HIGH);
  digitalWrite(relay[1], HIGH);
}
void LoopTest_Relay() {
  for (int i = 0; i < 2; i++) {
    Serial.println("LOOP TEST RELAY UTAMA : " + String(i));
    digitalWrite(relay[i], LOW);
    delay(1000);
    digitalWrite(relay[i], HIGH);
    delay(1000);
  }
}

void relaySatuOn() {
  digitalWrite(relay[0], LOW);
}
void relaySatuOff() {
  digitalWrite(relay[0], HIGH);
}

void relayDuaOn() {
  digitalWrite(relay[1], LOW);
}
void relayDuaOff() {
  digitalWrite(relay[1], HIGH);
}
