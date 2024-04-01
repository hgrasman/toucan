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

/*
  This task synthesizes information into state modes and torque requests, namely pedal position -> VDKArt -> torques
  VDKR
*/
TaskHandle_t xTaskVDKartHandle;
void VDKartTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }

  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  double trq = 0;
  double LeCRLR_a_AxFilt;
  double LeCRLR_a_AyFilt;
  double LeCRLR_a_AzFilt;  

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    LeCRLR_a_AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
    LeCRLR_a_AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
    LeCRLR_a_AzFilt = VeSNSR_a_IMU6AzFilt.getValue();

    double LeCRLR_p_TorqueSplitTarget = (LeCRLR_a_AxFilt*2+.5);
    if (LeCRLR_p_TorqueSplitTarget<0){LeCRLR_p_TorqueSplitTarget = 0;}
    if (LeCRLR_p_TorqueSplitTarget>1){LeCRLR_p_TorqueSplitTarget = 1;}

    trq = -LeCRLR_a_AyFilt * 5;

    if (trq<.05 && trq>-.05){trq = 0;
    }else if (trq>.05){trq -= .05;
    }else if (trq<-.05){trq += .05;}

    if (trq<0){trq = 0;}

    WRAP_SERIAL_MUTEX(Serial.print(VeCANR_tq_CAN0_iBSGInstMinTrqLim.getValue());Serial.print(", ");\
                      Serial.print(VeCANR_I_CAN0_BatteryCurrent.getValue());Serial.print(", ");\
                      Serial.println(VeCANR_tq_CAN0_iBSGInstMaxTrqLim.getValue());\
    , portMAX_DELAY )

    VeVDKR_tq_CAN0_TorqueRequest.setValue(trq * LeCRLR_p_TorqueSplitTarget);
    VeVDKR_tq_CAN1_TorqueRequest.setValue(trq * (1-LeCRLR_p_TorqueSplitTarget));

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

//This task handles communications with the high voltage batteries, and controls the relays
//HVPR
void HVManagerTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

    VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE);


  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

  }
}

uint8_t Controls_SetupTasks(void){

  xTaskCreatePinnedToCore(
      VDKartTask
      ,  "VDKart" 
      ,  2048        
      ,  NULL
      ,  8  // Priority
      ,  &xTaskVDKartHandle // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

    return (CONTROLS_SETUP_SUCCESS);
  
}