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

  while (VeCRLR_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  MUTEX_PRINT(pcTaskGetTaskName(NULL)); MUTEX_PRINTLN(" Go");

  float trq = 0;

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    //MUTEX_PRINT(pedalIn1); MUTEX_PRINT(", 0, 4095,"); MUTEX_PRINTln(pedalIn2);
    trq = (trq +.01);
    if (trq > 2) {trq = 0;}
    VeVDKR_CAN0TorqueRequest.setValue(trq/2);
    VeVDKR_CAN1TorqueRequest.setValue(trq/2);
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
  
}