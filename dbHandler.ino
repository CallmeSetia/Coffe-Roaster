void kirimDataFireBase() {
  if (Firebase.ready() && signupOK) {
    if (_reset == 1) {
      resetState();
      _reset = 0;
    }
    Firebase.RTDB.setIntAsync(&fbdo, "suhu", int(suhu));
    Firebase.RTDB.setStringAsync(&fbdo, "warna", String(tcs3200getColor()));
    Firebase.RTDB.setStringAsync(&fbdo, "waktu", String(t_second));
  }
}
void BacaButtonFireBase() {
  if (Firebase.RTDB.getString(&fbdo, "st_btn_light")) {
    if (fbdo.dataType() == "string") {
      st_btn_color[0] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_brown")) {
    if (fbdo.dataType() == "string") {
      st_btn_color[1] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_dark")) {
    if (fbdo.dataType() == "string") {
      st_btn_color[2] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_1")) {
    if (fbdo.dataType() == "string") {
      st_btn[0] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_2")) {
    if (fbdo.dataType() == "string") {
      st_btn[1] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_3")) {
    if (fbdo.dataType() == "string") {
      st_btn[3] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
  if (Firebase.RTDB.getString(&fbdo, "st_btn_4")) {
    if (fbdo.dataType() == "string") {
      st_btn[2] = fbdo.stringData();
    }
  }
  else {
    Serial.println(fbdo.errorReason());
  }
}
void resetState() {
  if (Firebase.ready() && signupOK) {
    Firebase.RTDB.setStringAsync(&fbdo, "st_btn_light", "0");
    Firebase.RTDB.setStringAsync(&fbdo, "st_btn_brown", "0");
    Firebase.RTDB.setStringAsync(&fbdo, "st_btn_dark", "0");
  }
}
