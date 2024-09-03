/*
  Software component responsible for data transfer between threads

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h"
#include "dataBroker.h"

BrokerData::BrokerData(){
      this->value = 0;
      this->last_update_time = -1; //data not valid
      portMUX_INITIALIZE(spinlock);
}

BrokerData::BrokerData(double initial_value){
      this->value = initial_value;
      this->last_update_time = esp_timer_get_time();
      portMUX_INITIALIZE(spinlock);
}

void BrokerData::setValue(double new_value) {
      taskENTER_CRITICAL(spinlock);
      this->value = new_value;
      this->last_update_time = esp_timer_get_time();
      taskEXIT_CRITICAL(spinlock);
}

void BrokerData::setValueISR(double new_value) {
      taskENTER_CRITICAL_ISR(spinlock);
      this->value = new_value;
      this->last_update_time = esp_timer_get_time();
      taskEXIT_CRITICAL_ISR(spinlock);
}

bool BrokerData::setDefault(double default_value) {
      bool status = false;
      taskENTER_CRITICAL(spinlock);
      if (this->last_update_time == -1){
        this->value = default_value;
        status = true;
      }
      taskEXIT_CRITICAL(spinlock);
      return (status);
}

double BrokerData::getValue() { 
      return (this->value);
}

double BrokerData::getValue(int64_t* time_Variable) { 
      taskENTER_CRITICAL(spinlock);
      *time_Variable = esp_timer_get_time() - this->last_update_time;
      double atomic = this->value;
      taskEXIT_CRITICAL(spinlock);
      return (atomic);
}

double BrokerData::getValue(int64_t* time_Variable, int64_t* last_time) { 
      taskENTER_CRITICAL(spinlock);
      *time_Variable = esp_timer_get_time() - this->last_update_time;
      *last_time = this->last_update_time;
      double atomic = this->value;
      taskEXIT_CRITICAL(spinlock);
      return (atomic);
}

bool BrokerData::dataInitialized(void){
      return (this->last_update_time >= 0);
}

SemaphoreHandle_t xSemaphore_SerialMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphore_I2CMutex = xSemaphoreCreateMutex();

//CRLR Controller Ring
BrokerData VeCRLR_b_ControlReadyFlag = BrokerData();

//HVPR high voltage propulsion
BrokerData VeHVPR_e_CANx_OpModeRequest = BrokerData(); //THESE SHOULD BE SYNCHRONIZED
BrokerData VeHVPR_e_HVTargetState      = BrokerData();

//BMSR battery management observer
BrokerData VeBMSR_v_CAN0_BatteryMINCell  = BrokerData();
BrokerData VeBMSR_v_CAN0_BatteryMAXCell  = BrokerData();
BrokerData VeBMSR_V_CAN0_BatteryVoltage  = BrokerData();
BrokerData VeBMSR_T_CAN0_BatteryMAXTemp  = BrokerData();
BrokerData VeBMSR_I_CAN0_BatteryCurrent  = BrokerData();
BrokerData VeBMSR_b_CAN0_BMSReporting    = BrokerData();

BrokerData VeBMSR_v_CAN1_BatteryMINCell  = BrokerData();
BrokerData VeBMSR_v_CAN1_BatteryMAXCell  = BrokerData();
BrokerData VeBMSR_V_CAN1_BatteryVoltage  = BrokerData();
BrokerData VeBMSR_T_CAN1_BatteryMAXTemp  = BrokerData();
BrokerData VeBMSR_I_CAN1_BatteryCurrent  = BrokerData();
BrokerData VeBMSR_b_CAN1_BMSReporting    = BrokerData();

//BPER battery parameter estimation
BrokerData VeBPER_V_CAN0_SSVObserved     = BrokerData();
BrokerData VeBPER_V_CAN0_SSVESREstimated = BrokerData();
BrokerData VeBPER_R_CAN0_ESRObserved     = BrokerData();
BrokerData VeBPER_V_CAN1_SSVObserved     = BrokerData();
BrokerData VeBPER_V_CAN1_SSVESREstimated = BrokerData();
BrokerData VeBPER_R_CAN1_ESRObserved     = BrokerData();

//VDKR VDKart Ring
BrokerData VeVDKR_tq_CAN0_TorqueRequest = BrokerData();
BrokerData VeVDKR_tq_CAN1_TorqueRequest = BrokerData();
BrokerData VeVDKR_phi_SteeringWheelAngle = BrokerData();
BrokerData VeVDKR_p_PedalPosition = BrokerData();
BrokerData VeVDKR_tq_TotalTorqueDesired = BrokerData();
BrokerData VeVDKR_P_CombinedMaxPower = BrokerData();
BrokerData VeVDKR_P_CombinedMinPower = BrokerData();

//CANR CAN driver Ring
BrokerData VeCANR_rpm_CAN0_iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN0_iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN0_iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN0_iBSGTorqueDelivered = BrokerData();
BrokerData VeCANR_pct_CAN0_iBSGInverterTempRate = BrokerData();
BrokerData VeCANR_V_CAN0_iBSGVoltageDCLink = BrokerData();
BrokerData VeCANR_T_CAN0_iBSGStatorTemp = BrokerData();
BrokerData VeCANR_pct_CAN0_iBSGMotorTempRate = BrokerData();
BrokerData VeCANR_tq_CAN0_iBSGInstMinTrqLim = BrokerData();
BrokerData VeCANR_tq_CAN0_iBSGInstMaxTrqLim = BrokerData();

BrokerData VeCANR_rpm_CAN1_iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN1_iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN1_iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN1_iBSGTorqueDelivered = BrokerData();
BrokerData VeCANR_pct_CAN1_iBSGInverterTempRate = BrokerData();
BrokerData VeCANR_V_CAN1_iBSGVoltageDCLink = BrokerData();
BrokerData VeCANR_T_CAN1_iBSGStatorTemp = BrokerData();
BrokerData VeCANR_pct_CAN1_iBSGMotorTempRate = BrokerData();
BrokerData VeCANR_tq_CAN1_iBSGInstMinTrqLim = BrokerData();
BrokerData VeCANR_tq_CAN1_iBSGInstMaxTrqLim = BrokerData();

BrokerData VeCANR_I_CAN0_BatteryCurrentRaw = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell1 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell2 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell3 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell4 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell5 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell6 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell7 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell8 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell9 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell10 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell11 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell12 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell13 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell14 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell15 = BrokerData();
BrokerData VeCANR_v_CAN0_BatteryVoltageCell16 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp1 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp2 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp3 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp4 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp5 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp6 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp7 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp8 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp9 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp10 = BrokerData();
BrokerData VeCANR_T_CAN0_BatteryTemp11 = BrokerData();

BrokerData VeCANR_I_CAN1_BatteryCurrentRaw = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell1 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell2 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell3 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell4 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell5 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell6 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell7 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell8 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell9 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell10 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell11 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell12 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell13 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell14 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell15 = BrokerData();
BrokerData VeCANR_v_CAN1_BatteryVoltageCell16 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp1 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp2 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp3 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp4 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp5 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp6 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp7 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp8 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp9 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp10 = BrokerData();
BrokerData VeCANR_T_CAN1_BatteryTemp11 = BrokerData();


//SNSR Sensor Ring
BrokerData VeSNSR_a_IMU6AxRaw = BrokerData();
BrokerData VeSNSR_a_IMU6AyRaw = BrokerData();
BrokerData VeSNSR_a_IMU6AzRaw = BrokerData();
BrokerData VeSNSR_w_IMU6WxRaw = BrokerData();
BrokerData VeSNSR_w_IMU6WyRaw = BrokerData();
BrokerData VeSNSR_w_IMU6WzRaw = BrokerData();

BrokerData VeSNSR_a_IMU6AxFilt = BrokerData();
BrokerData VeSNSR_a_IMU6AyFilt = BrokerData();
BrokerData VeSNSR_a_IMU6AzFilt = BrokerData();
BrokerData VeSNSR_w_IMU6WxFilt = BrokerData();
BrokerData VeSNSR_w_IMU6WyFilt = BrokerData();
BrokerData VeSNSR_w_IMU6WzFilt = BrokerData();


//WSSR
BrokerData VeWSSR_cnt_WSSPulseCount = BrokerData();


//GPSR GPS RING
BrokerData VeGPSR_deg_GPSLatitude  = BrokerData();
BrokerData VeGPSR_deg_GPSLongitude = BrokerData();
BrokerData VeGPSR_m_GPSAltitude = BrokerData();
BrokerData VeGPSR_deg_GPSHeading = BrokerData();
BrokerData VeGPSR_mps_GPSSpeed = BrokerData();
BrokerData VeGPSR_n_GPSSatellites = BrokerData();
BrokerData VeGPSR_e_GPSFixQuality = BrokerData();








