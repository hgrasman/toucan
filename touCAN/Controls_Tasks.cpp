/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "Controls_Tasks.h"
#include "x8578_can_db_client.h" //for enums
#include "pins.h"

TaskHandle_t xTaskVDKartHandle;

void VDKartTask(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }

  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  double trq = 0;
  double imuIn = 0;

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    imuIn= (-VeSNSR_a_IMU6AyRaw.getValue())*.1 + imuIn*.9;
    trq = imuIn * 2.5;

    if (trq<.05 && trq>-.05){trq = 0;
    }else if (trq>.05){trq -= .05;
    }else if (trq<-.05){trq += .05;}

    if (trq<0){trq = 0;}

    //WRAP_SERIAL_MUTEX(Serial.print(imuIn); Serial.print(',');Serial.println(trq), pdMS_TO_TICKS(5))
    WRAP_SERIAL_MUTEX(Serial.print(trq); Serial.println(VeCANR_e_CAN0iBSGOpMode.getValue());, pdMS_TO_TICKS(5))

    VeVDKR_tq_CAN0TorqueRequest.setValue(trq);
    VeVDKR_tq_CAN1TorqueRequest.setValue(trq);

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