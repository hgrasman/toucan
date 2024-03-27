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

SemaphoreHandle_t xSemaphore_SerialMutex;

//CRLR Controller Ring
BrokerData VeCRLR_b_ControlReadyFlag = BrokerData();

//VDKR VDKart Ring
BrokerData VeVDKR_tq_CAN0TorqueRequest = BrokerData();
BrokerData VeVDKR_tq_CAN1TorqueRequest = BrokerData();

//CANR CAN driver Ring
BrokerData VeCANR_rpm_CAN0iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN0iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN0iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN0iBSGTorqueDelivered = BrokerData();

BrokerData VeCANR_rpm_CAN1iBSGRotorSpeed = BrokerData();
BrokerData VeCANR_e_CAN1iBSGOpMode = BrokerData();
BrokerData VeCANR_I_CAN1iBSGDCCurrent = BrokerData();
BrokerData VeCANR_tq_CAN1iBSGTorqueDelivered = BrokerData();

//SNSR Sensor Ring
BrokerData VeSNSR_a_IMU6AxRaw = BrokerData();
BrokerData VeSNSR_a_IMU6AyRaw = BrokerData();
BrokerData VeSNSR_a_IMU6AzRaw = BrokerData();
BrokerData VeSNSR_a_IMU6WxRaw = BrokerData();
BrokerData VeSNSR_a_IMU6WyRaw = BrokerData();
BrokerData VeSNSR_a_IMU6WzRaw = BrokerData();








