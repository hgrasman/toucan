/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "Controls_Tasks.h"
#include "pins.h"

TaskHandle_t xTaskVDKartHandle;

void VDKartTask(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  float trq = 0;

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    //Serial.print(pedalIn1); Serial.print(", 0, 4095,"); Serial.println(pedalIn2);
    trq = (trq +.01);
    if (trq > 1) {trq = 0;}
    CAN0TorqueRequest.setValue(trq);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

uint8_t VDKart_SetupTasks(void){

  xTaskCreatePinnedToCore(
      VDKartTask
      ,  "VDKart" 
      ,  2048        
      ,  NULL
      ,  8  // Priority
      ,  &xTaskVDKartHandle // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

    return (VDKART_SETUP_SUCCESS);
  
  //todo here setup all the sensor tasks too.
}