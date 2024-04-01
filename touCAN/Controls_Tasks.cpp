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

  double LeCRLR_tq_TorqueTarget = 0;
  double LeCRLR_a_AxFilt;
  double LeCRLR_a_AyFilt;
  double LeCRLR_a_AzFilt;  

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    LeCRLR_a_AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
    LeCRLR_a_AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
    LeCRLR_a_AzFilt = VeSNSR_a_IMU6AzFilt.getValue();

    //get torque split from the other IMU
    double LeCRLR_p_TorqueSplitTarget = (LeCRLR_a_AxFilt*2+.5);
    if (LeCRLR_p_TorqueSplitTarget<0){LeCRLR_p_TorqueSplitTarget = 0;}
    if (LeCRLR_p_TorqueSplitTarget>1){LeCRLR_p_TorqueSplitTarget = 1;}

    LeCRLR_tq_TorqueTarget = -LeCRLR_a_AyFilt * 5; //get trq from imu rn

    //deadband
    if (LeCRLR_tq_TorqueTarget<.05 && LeCRLR_tq_TorqueTarget>-.05){LeCRLR_tq_TorqueTarget = 0;
    }else if (LeCRLR_tq_TorqueTarget>.05){LeCRLR_tq_TorqueTarget -= .05;
    }else if (LeCRLR_tq_TorqueTarget<-.05){LeCRLR_tq_TorqueTarget += .05;}

    if (LeCRLR_tq_TorqueTarget<0){LeCRLR_tq_TorqueTarget = 0;} //clamp at 0 for the moment

    //send torque request if prop system is active, otherwise zero
    if (VeHVPR_e_CANx_OpModeRequest.getValue() == X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE){
      VeVDKR_tq_CAN0_TorqueRequest.setValue(LeCRLR_tq_TorqueTarget * LeCRLR_p_TorqueSplitTarget);
      VeVDKR_tq_CAN1_TorqueRequest.setValue(LeCRLR_tq_TorqueTarget * (1-LeCRLR_p_TorqueSplitTarget));
    }else{
      VeVDKR_tq_CAN0_TorqueRequest.setValue(0);
      VeVDKR_tq_CAN1_TorqueRequest.setValue(0);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

//This task controls the relays and propulsion mode, and observes states
//HVPR
#define CeHVPR_e_HVTargetState_OFF 0
#define CeHVPR_e_HVTargetState_PRECHARGE 1
#define CeHVPR_e_HVTargetState_PROPACTIVE 2
void HVPropTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_EPIC_PMZ_A_EM_OPERATING_MODE_EXT_STANDBY_CHOICE);
  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  uint8_t LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF;
  TickType_t offStateTimer = xTaskGetTickCount();

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    //act on states
    switch (LeHVPR_e_HVTargetState){
      case CeHVPR_e_HVTargetState_OFF:
        
        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //disable the motor

        //disable the HV system
        digitalWrite(GATEKEEPER_1_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_1_PRE_PIN, LOW);
        digitalWrite(GATEKEEPER_2_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_2_PRE_PIN, LOW);

        //try to transition to precharge. will be stopped by safety limits
        if ((xTaskGetTickCount() - offStateTimer) > pdMS_TO_TICKS(OFF_STATE_TIMEOUT)){

          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_PRECHARGE; //attempt to precharge
          WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Attempting Precharge");, pdMS_TO_TICKS(8))

        }

        break;
      case CeHVPR_e_HVTargetState_PRECHARGE:

        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //disable the motor

        //Start the precharge relays
        digitalWrite(GATEKEEPER_1_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_1_PRE_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_2_PRE_PIN, HIGH);

        //transition to prop active once motors are up to voltage
        if (abs(VeCANR_V_CAN0_iBSGVoltageDCLink.getValue() - VeBMSR_V_CAN0_BatteryVoltage.getValue()) < PRECHARGE_END_AGREEMENT && \
            abs(VeCANR_V_CAN1_iBSGVoltageDCLink.getValue() - VeBMSR_V_CAN1_BatteryVoltage.getValue()) < PRECHARGE_END_AGREEMENT && \
            abs(VeBMSR_V_CAN0_BatteryVoltage.getValue() - VeBMSR_V_CAN1_BatteryVoltage.getValue()) < BATTERY_AGREEMENT_THRESHOLD \
        ){

          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_PROPACTIVE; //switch to prop active
          WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Attempting Prop Active");, pdMS_TO_TICKS(8))

        }

        break;
      case CeHVPR_e_HVTargetState_PROPACTIVE:

        //Engage the contactors
        digitalWrite(GATEKEEPER_1_REL_PIN, HIGH);
        digitalWrite(GATEKEEPER_1_PRE_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_REL_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_PRE_PIN, HIGH);

        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE); //ENABLE the motor

        break;
      default:
        WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println("BAD");, pdMS_TO_TICKS(100))
        LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF;
        continue; //BAD BAD BAD BAD try again 
    }

    //force into off state if safety conditions are not met
    if (VeBMSR_v_CAN0_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
        VeBMSR_v_CAN1_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
        VeBMSR_v_CAN0_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
        VeBMSR_v_CAN1_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
        VeBMSR_T_CAN0_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
        VeBMSR_T_CAN1_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
        VeBMSR_V_CAN0_BatteryVoltage.getValue() > PACK_VOLTAGE_MIN     || \
        VeBMSR_V_CAN1_BatteryVoltage.getValue() > PACK_VOLTAGE_MIN     || \
        VeBMSR_I_CAN0_BatteryCurrent.getValue() > PACK_CURRENT_MAX     || \
        VeBMSR_I_CAN1_BatteryCurrent.getValue() > PACK_CURRENT_MAX        \
    ){
      if (LeHVPR_e_HVTargetState != CeHVPR_e_HVTargetState_OFF){
        WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Safety Disable");, pdMS_TO_TICKS(5))
      }
      LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF; //force everything off
      offStateTimer = xTaskGetTickCount(); //start chill out timer
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
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

  pinMode(GATEKEEPER_1_REL_PIN, OUTPUT);
  pinMode(GATEKEEPER_1_PRE_PIN, OUTPUT);
  pinMode(GATEKEEPER_2_REL_PIN, OUTPUT);
  pinMode(GATEKEEPER_2_PRE_PIN, OUTPUT);
  xTaskCreatePinnedToCore(
      HVPropTask
      ,  "HVPropCrlr" 
      ,  2048        
      ,  NULL
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  return (CONTROLS_SETUP_SUCCESS);
  
}