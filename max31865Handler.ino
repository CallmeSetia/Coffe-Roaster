void max31865_Init() {
  //  Serial.println("Adafruit MAX31865 PT100 Sensor Test!")
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
}
void max31865_Begin() {
  uint16_t rtd = thermo.readRTD();
  ratio = rtd;
  ratio /= 32768;
}
float getTemperatureMax31865() {
  return thermo.temperature(RNOMINAL, RREF);
}
float getResistanceMax31865() {
  return RREF * ratio;
}
void max31865_Main() {
  uint16_t rtd = thermo.readRTD();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio, 8);
  Serial.print("Resistance = "); Serial.println(RREF * ratio, 8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    thermo.clearFault();
  }
  Serial.println();
  delay(1000);
}
