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

#define NM_RPM_TO_W 9.5488

/*
  This task synthesizes information into state modes and torque requests, namely pedal position -> VDKArt -> torques
  VDKR
*/
#define CeCHEN_e_HVSelectHighBattery 0
#define CeCHEN_e_HVSelectLowBattery 1
TaskHandle_t xTaskVDKartHandle;
void VDKartTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(5);

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  //Locals
  double LeVDKR_p_PedalPosition;
  double LeVDKR_rpm_CAN0_iBSGRotorSpeed;
  double LeVDKR_rpm_CAN1_iBSGRotorSpeed;
  double LeVDKR_rpm_EncoderSpeed;
  double LeVDKR_phi_ApproxSWA;
  double LeVDKR_phi_SWASensorError;
  double LeVDKR_p_TorqueSplitTarget;
  double LeVDKR_a_AxFilt, LeVDKR_a_AyFilt, LeVDKR_a_AzFilt, LeVDKR_w_WxFilt, LeVDKR_w_WyFilt, LeVDKR_w_WzFilt;
  int64_t LeVDKR_a_AxFiltFreshness, LeVDKR_a_AyFiltFreshness, LeVDKR_a_AzFiltFreshness, LeVDKR_w_WxFiltFreshness, LeVDKR_w_WyFiltFreshness, LeVDKR_w_WzFiltFreshness;

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    //grab data from the broker
    LeVDKR_a_AxFilt = VeSNSR_a_IMU6AxFilt.getValue(&LeVDKR_a_AxFiltFreshness);
    LeVDKR_a_AyFilt = VeSNSR_a_IMU6AyFilt.getValue(&LeVDKR_a_AyFiltFreshness);
    LeVDKR_a_AzFilt = VeSNSR_a_IMU6AzFilt.getValue(&LeVDKR_a_AzFiltFreshness);
    LeVDKR_w_WxFilt = VeSNSR_w_IMU6WxFilt.getValue();
    LeVDKR_w_WyFilt = VeSNSR_w_IMU6WyFilt.getValue();
    LeVDKR_w_WzFilt = VeSNSR_w_IMU6WzFilt.getValue();

    LeVDKR_rpm_CAN0_iBSGRotorSpeed = VeCANR_rpm_CAN0_iBSGRotorSpeed.getValue();
    
    LeVDKR_p_PedalPosition = (double) analogRead(PEDAL_IN_NML_PIN) / 4095.0;

    //Get limits from the motor
    double LeVDKR_tq_CAN0_MinTrqLim = VeCANR_tq_CAN0_iBSGInstMinTrqLim.getValue();
    double LeVDKR_tq_CAN0_MaxTrqLim = VeCANR_tq_CAN0_iBSGInstMaxTrqLim.getValue();

    //stop the thing from spinning backward
    double LeVDKR_tq_MinTrqTaperL = REGEN_TAPER_FUNC(LeVDKR_rpm_CAN0_iBSGRotorSpeed); 
    if (LeVDKR_tq_CAN0_MinTrqLim < LeVDKR_tq_MinTrqTaperL){LeVDKR_tq_CAN0_MinTrqLim = LeVDKR_tq_MinTrqTaperL;}

    //Calculate max power and limit -> recalculate max torque. At very low rpm, use constant min rpm for this calculation
    double LeVDKR_rpm_MaxPowerClampedSpeedL = LeVDKR_rpm_CAN0_iBSGRotorSpeed;
    if(LeVDKR_rpm_MaxPowerClampedSpeedL < POWER_LIMIT_MINRPM){LeVDKR_rpm_MaxPowerClampedSpeedL = POWER_LIMIT_MINRPM;}

    double LeVDKR_P_CombinedMaxPower = (LeVDKR_tq_CAN0_MaxTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedL) / NM_RPM_TO_W; //best the motor can do
    double LeVDKR_P_CombinedMinPower = (LeVDKR_tq_CAN0_MinTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedL) / NM_RPM_TO_W; //quite possibly 28kW

    //limit mechanical power according to competition rules/safety margin
    double LeVDKR_tq_CombinedMaxTrq = LeVDKR_tq_CAN0_MaxTrqLim;
    double LeVDKR_tq_CombinedMinTrq = LeVDKR_tq_CAN0_MinTrqLim;
    if (LeVDKR_P_CombinedMaxPower > (PDGP_POWER_LIMIT-POWER_LIMIT_MARGIN)){
      LeVDKR_P_CombinedMaxPower = (PDGP_POWER_LIMIT-POWER_LIMIT_MARGIN);
      LeVDKR_tq_CombinedMaxTrq = (NM_RPM_TO_W * LeVDKR_P_CombinedMaxPower) / (LeVDKR_rpm_MaxPowerClampedSpeedL);
    }
    if (LeVDKR_P_CombinedMinPower < REGEN_POWER_LIMIT){
      LeVDKR_P_CombinedMinPower = REGEN_POWER_LIMIT;
      LeVDKR_tq_CombinedMinTrq = (NM_RPM_TO_W * LeVDKR_P_CombinedMinPower) / (LeVDKR_rpm_MaxPowerClampedSpeedL);
    }

    //map the pedal
    double LeVDKR_tq_TorqueL = 0;
    double LeVDKR_e_MotorStateRequest = X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE;
    if (LeVDKR_p_PedalPosition < .25){
      LeVDKR_tq_TorqueL = (1-(LeVDKR_p_PedalPosition*4))*LeVDKR_tq_CombinedMinTrq;
      LeVDKR_e_MotorStateRequest = X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE;
    }
    if (LeVDKR_p_PedalPosition > .75){
      LeVDKR_tq_TorqueL = (((LeVDKR_p_PedalPosition-.75)*4))*LeVDKR_tq_CombinedMaxTrq;
      LeVDKR_e_MotorStateRequest = X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE;
    }
    VeVDKR_e_MotorStateRequest.setValue(LeVDKR_e_MotorStateRequest);

    //trigger battery switch
    if (LeVDKR_p_PedalPosition > .625){
      VeCHEN_e_BatterySelectionTarget.setValue(CeCHEN_e_HVSelectHighBattery);
    }
    if (LeVDKR_p_PedalPosition < .375){
      VeCHEN_e_BatterySelectionTarget.setValue(CeCHEN_e_HVSelectLowBattery);
    }
    
    //send torque request if prop system is active, otherwise zero
    if (VeHVPR_e_CANx_OpModeRequest.getValue() == X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE){
      VeVDKR_tq_CAN0_TorqueRequest.setValue(LeVDKR_tq_TorqueL);
    }else{
      VeVDKR_tq_CAN0_TorqueRequest.setValue(0);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

//This task controls the relays and propulsion mode, and observes states
//HVPR
#define CeHVPR_e_HVTargetState_OFF 0
#define CeHVPR_e_HVTargetState_TRANSITION 1
#define CeHVPR_e_HVTargetState_PROPACTIVE 2
#define CeHVPR_e_Battery0Selected 0
#define CeHVPR_e_Battery1Selected 1
#define CeHVPR_e_BatteryNoneSelected 2
void HVPropTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_EPIC_PMZ_A_EM_OPERATING_MODE_EXT_STANDBY_CHOICE);
  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  uint8_t LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF;
  uint8_t LeHVPR_e_HVBatterySelected = CeHVPR_e_BatteryNoneSelected;
  double LeHVPR_e_HighBattery, LeHVPR_e_LowBattery;
  TickType_t offStateTimer = xTaskGetTickCount();

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    //calculate the low battery
    double LeHVPR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
    double LeHVPR_V_CAN1_BatteryVoltage = VeBMSR_V_CAN1_BatteryVoltage.getValue();

    double LeHVPR_e_BatterySelectionTarget = VeCHEN_e_BatterySelectionTarget.getValue();
    double LeHVPR_e_BatterySelectionTargetPhysical;

    //store high vs low batteries
    if (LeHVPR_V_CAN0_BatteryVoltage>(LeHVPR_V_CAN1_BatteryVoltage)){
      LeHVPR_e_HighBattery = CeHVPR_e_Battery0Selected;
      LeHVPR_e_LowBattery = CeHVPR_e_Battery1Selected;
    }else{
      LeHVPR_e_HighBattery = CeHVPR_e_Battery1Selected;
      LeHVPR_e_LowBattery = CeHVPR_e_Battery0Selected;
    }

    //act on states
    switch (LeHVPR_e_HVTargetState){
      case CeHVPR_e_HVTargetState_OFF:
        
        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //disable the motor

        //turn everything off
        digitalWrite(GATEKEEPER_1_REL_PIN, LOW);
        VeHVPR_e_Rel1RelayState.setValue(LOW);
        digitalWrite(GATEKEEPER_1_PRE_PIN, LOW);
        VeHVPR_e_Pre1RelayState.setValue(LOW);
        digitalWrite(GATEKEEPER_2_REL_PIN, LOW);
        VeHVPR_e_Rel2RelayState.setValue(LOW);
        digitalWrite(GATEKEEPER_2_PRE_PIN, LOW);
        VeHVPR_e_Pre2RelayState.setValue(LOW);
        LeHVPR_e_HVBatterySelected = CeHVPR_e_BatteryNoneSelected;

        //try to transition to precharge. will be stopped by safety limits
        if ((xTaskGetTickCount() - offStateTimer) > pdMS_TO_TICKS(OFF_STATE_TIMEOUT)){

          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_TRANSITION;

        }

        break;
      case CeHVPR_e_HVTargetState_TRANSITION:

        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //disable the mot

        if (LeHVPR_e_BatterySelectionTarget == CeCHEN_e_HVSelectHighBattery){
          LeHVPR_e_BatterySelectionTargetPhysical = LeHVPR_e_HighBattery;
        }
        if (LeHVPR_e_BatterySelectionTarget == CeCHEN_e_HVSelectLowBattery){
          LeHVPR_e_BatterySelectionTargetPhysical = LeHVPR_e_LowBattery;
        }

        switch ((uint8_t)LeHVPR_e_BatterySelectionTargetPhysical){
          case CeHVPR_e_Battery0Selected:
            digitalWrite(GATEKEEPER_2_REL_PIN, LOW);
            VeHVPR_e_Rel2RelayState.setValue(LOW);
            digitalWrite(GATEKEEPER_2_PRE_PIN, LOW);
            VeHVPR_e_Pre2RelayState.setValue(LOW);
            digitalWrite(GATEKEEPER_1_REL_PIN, HIGH);
            VeHVPR_e_Rel1RelayState.setValue(HIGH);
            digitalWrite(GATEKEEPER_1_PRE_PIN, HIGH);
            VeHVPR_e_Pre1RelayState.setValue(HIGH);
            LeHVPR_e_HVBatterySelected = CeHVPR_e_Battery0Selected;
            break;
          case CeHVPR_e_Battery1Selected:
            digitalWrite(GATEKEEPER_1_REL_PIN, LOW);
            VeHVPR_e_Rel1RelayState.setValue(LOW);
            digitalWrite(GATEKEEPER_1_PRE_PIN, LOW);
            VeHVPR_e_Pre1RelayState.setValue(LOW);
            digitalWrite(GATEKEEPER_2_REL_PIN, HIGH);
            VeHVPR_e_Rel2RelayState.setValue(HIGH);
            digitalWrite(GATEKEEPER_2_PRE_PIN, HIGH);
            VeHVPR_e_Pre2RelayState.setValue(HIGH);
            LeHVPR_e_HVBatterySelected = CeHVPR_e_Battery1Selected;
            break;
          case CeHVPR_e_BatteryNoneSelected:
            break;
        }

        if (VeVDKR_e_MotorStateRequest.getValue() == X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE){
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_PROPACTIVE;
        }

        break;
      case CeHVPR_e_HVTargetState_PROPACTIVE:

        if (VeVDKR_e_MotorStateRequest.getValue() != X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE){
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF;
        }

        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE); //ENABLE the motor

        //baby the motor
        if (VeCANR_e_CAN0_iBSGOpMode.getValue() == X8578_CAN_DB_CLIENT_EPIC_PMZ_A_EM_OPERATING_MODE_EXT_NOT__CAPABLE_CHOICE){
          //we accidentally hit a motor limit
          VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //turn it off
          //hopefully it turns on again next time around
        }

        break;
      default:
        WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println("BAD");, pdMS_TO_TICKS(100))
        LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF;
        continue; //BAD BAD BAD BAD try again 
    }

    VeHVPR_e_HVTargetState.setValue(LeHVPR_e_HVTargetState);

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