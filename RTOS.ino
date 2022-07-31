void RTOS_Init() {
  xTaskCreatePinnedToCore(
    FireBase_taskCallback,   /* Task function. */
    "FireBaseTask",     /* name of task. */
    20000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &FireBase_task,      /* Task handle to keep track of created task */
    1);       /* pin task to core 0 */
}
