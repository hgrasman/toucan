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
    LeVDKR_rpm_CAN1_iBSGRotorSpeed = VeCANR_rpm_CAN1_iBSGRotorSpeed.getValue();

    //get the pedal and wheel values
    double LeVDKR_p_SWAPositionNML = (double) analogRead(STEER_IN_NML_PIN) / 4095.0;
    double LeVDKR_p_SWAPositionINV = (double) analogRead(STEER_IN_INV_PIN) / 4095.0;
    LeVDKR_phi_SWASensorError = 1 - (LeVDKR_p_SWAPositionNML+LeVDKR_p_SWAPositionINV);
    LeVDKR_phi_ApproxSWA   = (LeVDKR_p_SWAPositionNML - LeVDKR_p_SWAPositionINV)*(SWA_RANGE_DEG/2);
    
    LeVDKR_p_PedalPosition = (double) analogRead(PEDAL_IN_NML_PIN) / 4095.0;

    //Get limits from the motor
    double LeVDKR_tq_CAN0_MinTrqLim = VeCANR_tq_CAN0_iBSGInstMinTrqLim.getValue();
    double LeVDKR_tq_CAN0_MaxTrqLim = VeCANR_tq_CAN0_iBSGInstMaxTrqLim.getValue();
    double LeVDKR_tq_CAN1_MinTrqLim = VeCANR_tq_CAN1_iBSGInstMinTrqLim.getValue();
    double LeVDKR_tq_CAN1_MaxTrqLim = VeCANR_tq_CAN1_iBSGInstMaxTrqLim.getValue();

    //stop the thing from spinning backward
    double LeVDKR_tq_MinTrqTaperL = REGEN_TAPER_FUNC(LeVDKR_rpm_CAN0_iBSGRotorSpeed); if (LeVDKR_tq_MinTrqTaperL>REGEN_LIMIT_MAX_REV_DAMPING){LeVDKR_tq_MinTrqTaperL = REGEN_LIMIT_MAX_REV_DAMPING;}
    double LeVDKR_tq_MinTrqTaperR = REGEN_TAPER_FUNC(LeVDKR_rpm_CAN1_iBSGRotorSpeed); if (LeVDKR_tq_MinTrqTaperR>REGEN_LIMIT_MAX_REV_DAMPING){LeVDKR_tq_MinTrqTaperR = REGEN_LIMIT_MAX_REV_DAMPING;}
    if (LeVDKR_tq_CAN0_MinTrqLim < LeVDKR_tq_MinTrqTaperL){LeVDKR_tq_CAN0_MinTrqLim = LeVDKR_tq_MinTrqTaperL;}
    if (LeVDKR_tq_CAN1_MinTrqLim < LeVDKR_tq_MinTrqTaperR){LeVDKR_tq_CAN1_MinTrqLim = LeVDKR_tq_MinTrqTaperR;}
    

#ifdef DUAL_MOTOR_CART
    //VDKART split. Return to Default if swa errors or input data out of date
    if (abs(LeVDKR_phi_SWASensorError) > SWA_ERROR_THRESHOLD){
      LeVDKR_p_TorqueSplitTarget = LeVDKR_p_TorqueSplitTarget*SWA_ERROR_RETURN_FILT + VECTOR_CENTER_SPLIT * (1-SWA_ERROR_RETURN_FILT); //smoothly return to even split
    }else{
      //TORQUE SPLIT
      LeVDKR_p_TorqueSplitTarget = .5; 
    }
    double LeVDKR_p_TorqueSplitTargetFilt = LeVDKR_p_TorqueSplitTargetFilt*VECTOR_RATE_FILT + LeVDKR_p_TorqueSplitTarget * (1-VECTOR_RATE_FILT);
#else
    double LeVDKR_p_TorqueSplitTargetFilt = 0; //quick and dirty way to put all the power to 1mc
#endif

    //THE FOLLOWING MAPS THE PEDAL INTELLIGENTLY TO THE LIMITS OF THE RACE, MOTORS, BATTERY
    //
    //Find mechanical power limits reported by the motor
    double LeVDKR_tq_CombinedMaxTrq = LeVDKR_tq_CAN0_MaxTrqLim*(1-LeVDKR_p_TorqueSplitTargetFilt) + LeVDKR_tq_CAN1_MaxTrqLim*LeVDKR_p_TorqueSplitTargetFilt;
    double LeVDKR_tq_CombinedMinTrq = LeVDKR_tq_CAN0_MinTrqLim*(1-LeVDKR_p_TorqueSplitTargetFilt) + LeVDKR_tq_CAN1_MinTrqLim*LeVDKR_p_TorqueSplitTargetFilt;

    //Calculate max power and limit -> recalculate max torque. At very low rpm, use constant min rpm for this calculation
    double LeVDKR_rpm_MaxPowerClampedSpeedL = LeVDKR_rpm_CAN0_iBSGRotorSpeed;
    double LeVDKR_rpm_MaxPowerClampedSpeedR = LeVDKR_rpm_CAN1_iBSGRotorSpeed;
    if(LeVDKR_rpm_MaxPowerClampedSpeedL < POWER_LIMIT_MINRPM){LeVDKR_rpm_MaxPowerClampedSpeedL = POWER_LIMIT_MINRPM;}
    if(LeVDKR_rpm_MaxPowerClampedSpeedR < POWER_LIMIT_MINRPM){LeVDKR_rpm_MaxPowerClampedSpeedR = POWER_LIMIT_MINRPM;}
    double LeVDKR_P_CombinedMaxPower = (LeVDKR_tq_CAN0_MaxTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedL*(1-LeVDKR_p_TorqueSplitTargetFilt) \
                                        + LeVDKR_tq_CAN1_MaxTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedR*LeVDKR_p_TorqueSplitTargetFilt) / NM_RPM_TO_W; //best the motor can do
    double LeVDKR_P_CombinedMinPower = (LeVDKR_tq_CAN0_MinTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedL*(1-LeVDKR_p_TorqueSplitTargetFilt) \
                                        + LeVDKR_tq_CAN1_MinTrqLim*LeVDKR_rpm_MaxPowerClampedSpeedR*LeVDKR_p_TorqueSplitTargetFilt) / NM_RPM_TO_W; //quite possibly 28kW

    //limit mechanical power according to competition rules/safety margin
    if (LeVDKR_P_CombinedMaxPower > (PDGP_POWER_LIMIT-POWER_LIMIT_MARGIN)){
      LeVDKR_P_CombinedMaxPower = (PDGP_POWER_LIMIT-POWER_LIMIT_MARGIN);
      LeVDKR_tq_CombinedMaxTrq = (NM_RPM_TO_W * LeVDKR_P_CombinedMaxPower) / ((LeVDKR_rpm_MaxPowerClampedSpeedL*(1-LeVDKR_p_TorqueSplitTargetFilt) + LeVDKR_rpm_MaxPowerClampedSpeedR*LeVDKR_p_TorqueSplitTargetFilt));
    }
    if (LeVDKR_P_CombinedMinPower < REGEN_POWER_LIMIT){
      LeVDKR_P_CombinedMinPower = REGEN_POWER_LIMIT;
      LeVDKR_tq_CombinedMinTrq = (NM_RPM_TO_W * LeVDKR_P_CombinedMinPower) / ((LeVDKR_rpm_MaxPowerClampedSpeedL*(1-LeVDKR_p_TorqueSplitTargetFilt) + LeVDKR_rpm_MaxPowerClampedSpeedR*LeVDKR_p_TorqueSplitTargetFilt));
    }

    //estimate electrical power limit for battery undervoltage protection
    double LeVDKR_V_SSVObservedAvg = (VeBPER_V_CAN0_SSVObserved.getValue() + VeBPER_V_CAN1_SSVObserved.getValue())/2;
    double LeVDKR_R_ESRObservedAvg = (VeBPER_R_CAN0_ESRObserved.getValue() + VeBPER_R_CAN1_ESRObserved.getValue())/2;
    double LeVDKR_P_SSVLimitedPower = ((LeVDKR_V_SSVObservedAvg - PACK_VOLTAGE_MIN) / LeVDKR_R_ESRObservedAvg)*PACK_VOLTAGE_MIN;
    double LeVDKR_P_SSVLimitedRegen = ((LeVDKR_V_SSVObservedAvg - PACK_VOLTAGE_MAX) / LeVDKR_R_ESRObservedAvg)*PACK_VOLTAGE_MAX;
    //TODO USE THESE

    //map the pedal and apply the torque split
    double LeVDKR_tq_TotalTorqueDesired = LeVDKR_tq_CombinedMaxTrq*LeVDKR_p_PedalPosition + LeVDKR_tq_CombinedMinTrq*(1-LeVDKR_p_PedalPosition);
    double LeVDKR_tq_TorqueL = LeVDKR_tq_TotalTorqueDesired*(1-LeVDKR_p_TorqueSplitTargetFilt);
    double LeVDKR_tq_TorqueR = LeVDKR_tq_TotalTorqueDesired*LeVDKR_p_TorqueSplitTargetFilt;

    //limits again (per motor basis)
    if (LeVDKR_tq_TorqueL<LeVDKR_tq_CAN0_MinTrqLim){LeVDKR_tq_TorqueL=LeVDKR_tq_CAN0_MinTrqLim;}
    if (LeVDKR_tq_TorqueL>LeVDKR_tq_CAN0_MaxTrqLim){LeVDKR_tq_TorqueL=LeVDKR_tq_CAN0_MaxTrqLim;}
#ifdef DUAL_MOTOR_CART
    if (LeVDKR_tq_TorqueR<LeVDKR_tq_CAN1_MinTrqLim){LeVDKR_tq_TorqueR=LeVDKR_tq_CAN1_MinTrqLim;}
    if (LeVDKR_tq_TorqueR>LeVDKR_tq_CAN1_MaxTrqLim){LeVDKR_tq_TorqueR=LeVDKR_tq_CAN1_MaxTrqLim;}
#endif

    //calculate actual electrical power and scale
    //TODO ###########################################################

    //send torque request if prop system is active, otherwise zero
    if (VeHVPR_e_CANx_OpModeRequest.getValue() == X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE){
      VeVDKR_tq_CAN0_TorqueRequest.setValue(LeVDKR_tq_TorqueL);
      VeVDKR_tq_CAN1_TorqueRequest.setValue(LeVDKR_tq_TorqueR);
    }else{
      VeVDKR_tq_CAN0_TorqueRequest.setValue(0);
      VeVDKR_tq_CAN1_TorqueRequest.setValue(0);
    }

    /*WRAP_SERIAL_MUTEX(\
                      Serial.print(VeBMSR_V_CAN0_BatteryVoltage.getValue()); Serial.print(", ");\
                      Serial.print(LeVDKR_V_SSVObservedAvg); Serial.print(", ");\
                      Serial.print(VeBPER_V_CAN0_SSVESREstimated.getValue()); Serial.print(", ");\
                      Serial.print(LeVDKR_R_ESRObservedAvg*1000); Serial.print(", ");\
                      Serial.print(""); Serial.println("");\
                      , pdMS_TO_TICKS(100))*/

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
  TickType_t preStateTimer;

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    bool Motor0PreCharged;
    bool Motor1PreCharged;
    bool TimoutReached;
    bool BatteriesBalanced;
    bool bmsCheck;
    bool MotorCheck;
    bool Motor0Powered;
    bool Motor1Powered;
    bool MotorPowerCondition;

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
          //WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Attempting Precharge");, pdMS_TO_TICKS(8))
          preStateTimer = xTaskGetTickCount(); //start timer

        }

        break;
      case CeHVPR_e_HVTargetState_PRECHARGE:

        //force into off state if safety conditions are not met
        if (VeBMSR_v_CAN0_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
            VeBMSR_v_CAN1_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
            VeBMSR_v_CAN0_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
            VeBMSR_v_CAN1_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
            VeBMSR_T_CAN0_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
            VeBMSR_T_CAN1_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
            VeBMSR_V_CAN0_BatteryVoltage.getValue() < PACK_VOLTAGE_MIN     || \
            VeBMSR_V_CAN1_BatteryVoltage.getValue() < PACK_VOLTAGE_MIN     || \
            VeBMSR_I_CAN0_BatteryCurrent.getValue() > PACK_CURRENT_MAX     || \
            VeBMSR_I_CAN1_BatteryCurrent.getValue() > PACK_CURRENT_MAX        \
        ){
          //WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Precharge Not Ready");, pdMS_TO_TICKS(5))
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF; //force everything off
          offStateTimer = xTaskGetTickCount(); //start chill out timer
          break;
        }

        VeHVPR_e_CANx_OpModeRequest.setValue(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_STANDBY_CHOICE); //disable the motor

        //Start the precharge relays
        digitalWrite(GATEKEEPER_1_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_1_PRE_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_REL_PIN, LOW);
        digitalWrite(GATEKEEPER_2_PRE_PIN, HIGH);

        //transition to prop active once motors are up to voltage
        Motor0PreCharged   = abs(VeCANR_V_CAN0_iBSGVoltageDCLink.getValue() - VeBMSR_V_CAN0_BatteryVoltage.getValue()) < PRECHARGE_END_AGREEMENT;
        Motor1PreCharged   = abs(VeCANR_V_CAN1_iBSGVoltageDCLink.getValue() - VeBMSR_V_CAN1_BatteryVoltage.getValue()) < PRECHARGE_END_AGREEMENT;
        TimoutReached      = (xTaskGetTickCount() - preStateTimer) > pdMS_TO_TICKS(PRE_STATE_TIMEOUT);
        BatteriesBalanced  = abs(VeBMSR_V_CAN0_BatteryVoltage.getValue() - VeBMSR_V_CAN1_BatteryVoltage.getValue()) < BATTERY_AGREEMENT_THRESHOLD;
#ifdef ENABLE_REDUNDANT_BMS
        if (VeBMSR_b_CAN0_BMSReporting.getValue() && VeBMSR_b_CAN1_BMSReporting.getValue()){
          bmsCheck = BatteriesBalanced;
        }else{
          bmsCheck = TimoutReached;
        }     
#else
        bmsCheck = BatteriesBalanced;
#endif
#ifdef DUAL_MOTOR_CART
        MotorCheck = Motor0PreCharged && Motor1PreCharged;      
#else
        MotorCheck = Motor0PreCharged;
#endif
        if (bmsCheck && MotorCheck){
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_PROPACTIVE; //switch to prop active
          //WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Attempting Prop Active");, pdMS_TO_TICKS(8))
          break;
        }

        break;
      case CeHVPR_e_HVTargetState_PROPACTIVE:

        //force into off state if safety conditions are not met
        if (VeBMSR_v_CAN0_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
            VeBMSR_v_CAN1_BatteryMAXCell.getValue() > CELL_MAXIMUM_VOLTAGE || \
            VeBMSR_v_CAN0_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
            VeBMSR_v_CAN1_BatteryMINCell.getValue() < CELL_MINIMUM_VOLTAGE || \
            VeBMSR_T_CAN0_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
            VeBMSR_T_CAN1_BatteryMAXTemp.getValue() > PACK_MAXIMUM_TEMP    || \
            VeBMSR_V_CAN0_BatteryVoltage.getValue() < PACK_VOLTAGE_MIN     || \
            VeBMSR_V_CAN1_BatteryVoltage.getValue() < PACK_VOLTAGE_MIN     || \
            VeBMSR_I_CAN0_BatteryCurrent.getValue() > PACK_CURRENT_MAX     || \
            VeBMSR_I_CAN1_BatteryCurrent.getValue() > PACK_CURRENT_MAX        \
        ){
          //WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Safety Disable");, pdMS_TO_TICKS(5))
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_OFF; //force everything off
          offStateTimer = xTaskGetTickCount(); //start chill out timer
          break;
        }

        //if manual switch cuts relays, put it back in precharge
        Motor0Powered = VeCANR_V_CAN0_iBSGVoltageDCLink.getValue() < PACK_VOLTAGE_MIN;
        Motor1Powered = VeCANR_V_CAN1_iBSGVoltageDCLink.getValue() < PACK_VOLTAGE_MIN;
#ifdef DUAL_MOTOR_CART
        MotorPowerCondition = Motor0Powered && Motor1Powered;
#else
        MotorPowerCondition = Motor0Powered;
#endif
        if (MotorPowerCondition){
          LeHVPR_e_HVTargetState = CeHVPR_e_HVTargetState_PRECHARGE; //attempt to precharge
          //WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" -> Contactors Open, Pre Charge");, pdMS_TO_TICKS(8))
          preStateTimer = xTaskGetTickCount();
          break;
        }

        //Engage the contactors
        digitalWrite(GATEKEEPER_1_REL_PIN, HIGH);
        digitalWrite(GATEKEEPER_1_PRE_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_REL_PIN, HIGH);
        digitalWrite(GATEKEEPER_2_PRE_PIN, HIGH);

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