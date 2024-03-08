/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 

void VDKartTask(void *pvParameters){  // This is a task.
  const uint32_t period_ticks = pdMS_TO_TICKS(500);
  
  for(;;){
    
    Serial.printf("Yo this is the TQ Request. My code isn't done yet\n");
    vTaskDelay(period_ticks);
  }
}