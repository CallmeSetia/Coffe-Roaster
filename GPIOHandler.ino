void GPIO_Init() {
  for (int i = 0; i < 4; i++ ) {
    pinMode(buttonPin[i], INPUT);
  }
}
