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
BrokerData VeVDKR_tq_CAN0_TorqueRequest = BrokerData();
BrokerData VeVDKR_tq_CAN1_TorqueRequest = BrokerData();

//CANR CAN driver Ring
BrokerData VeCANR_rpm_CAN0_iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN0_iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN0_iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN0_iBSGTorqueDelivered = BrokerData();
BrokerData VeCANR_pct_CAN0_iBSGInverterTemperatureRate = BrokerData();
BrokerData VeCANR_V_CAN0_iBSGVoltageDCLink = BrokerData();
BrokerData VeCANR_T_CAN0_iBSGStatorTemperature = BrokerData();
BrokerData VeCANR_T_CAN0_iBSGRotorTemperature = BrokerData();

BrokerData VeCANR_rpm_CAN1_iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN1_iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN1_iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN1_iBSGTorqueDelivered = BrokerData();
BrokerData VeCANR_pct_CAN1_iBSGInverterTemperatureRate = BrokerData();
BrokerData VeCANR_V_CAN1_iBSGVoltageDCLink = BrokerData();
BrokerData VeCANR_T_CAN1_iBSGStatorTemperature = BrokerData();
BrokerData VeCANR_T_CAN1_iBSGRotorTemperature = BrokerData();

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










