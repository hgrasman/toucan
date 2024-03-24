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
  const TickType_t xPeriod = pdMS_TO_TICKS(100);

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    uint16_t pedalIn1 = analogRead(PEDAL_IN_NML_PIN);
    uint16_t pedalIn2 = analogRead(PEDAL_IN_NML_PIN);
    Serial.print(pedalIn1); Serial.print(" "); Serial.print(pedalIn2);
    CAN0BrokerData.torqueRequest.setValue(0);
    CAN1BrokerData.torqueRequest.setValue(0);
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