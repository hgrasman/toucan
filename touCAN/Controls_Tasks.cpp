/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 

void VDKartTask(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(500);

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    Serial.printf("Yo this is the TQ Request. My code isn't done yet\n");
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}