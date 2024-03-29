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
      last_update_time = -1; //data not valid
}

BrokerData::BrokerData(double initial_value){
      this->value = initial_value;
      last_update_time = -1; //data not valid
}

void BrokerData::setValue(double new_value) {
      this->value = new_value;
      this->last_update_time = esp_timer_get_time();
}

double BrokerData::getValue() { 
      return (this->value);
}

double BrokerData::getValue(int64_t* time_Variable) { 
      *time_Variable = this->last_update_time;
      return (this->value);
}

bool BrokerData::dataInitialized(void){
      return (this->last_update_time >= 0);
}

SemaphoreHandle_t xSemaphore_SerialMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphore_I2CMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphore_CANSPIMutex = xSemaphoreCreateMutex();

//CRLR Controller Ring
BrokerData VeCRLR_b_ControlReadyFlag = BrokerData();

//VDKR VDKart Ring
BrokerData VeVDKR_e_CANx_OpModeRequest = BrokerData(); //THESE SHOULD BE SYNCHRONIZED
BrokerData VeVDKR_tq_CAN0_TorqueRequest = BrokerData();
BrokerData VeVDKR_tq_CAN1_TorqueRequest = BrokerData();

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

BrokerData VeCANR_I_CAN0_BatteryCurrent = BrokerData();
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

BrokerData VeCANR_I_CAN1_BatteryCurrent = BrokerData();
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










