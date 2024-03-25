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

BrokerData::BrokerData(float initial_value){
      this->value = initial_value;
      last_update_time = -1; //data not valid
}

void BrokerData::setValue(float new_value) {
      this->value = new_value;
      this->last_update_time = esp_timer_get_time();
}

float BrokerData::getValue() { 
      return (this->value);
}

float BrokerData::getValue(int64_t* time_Variable) { 
      *time_Variable = this->last_update_time;
      return (this->value);
}

bool BrokerData::dataInitialized(void){
      return (this->last_update_time >= 0);
}

SemaphoreHandle_t xSemaphore_SerialMutex;

//Data for other functions
BrokerData VeCRLR_ControlReadyFlag = BrokerData();

BrokerData VeVDKR_CAN0TorqueRequest = BrokerData();
BrokerData VeVDKR_CAN1TorqueRequest= BrokerData();

BrokerData VeSNSR_IMU6AxRaw = BrokerData();
BrokerData VeSNSR_IMU6AyRaw = BrokerData();
BrokerData VeSNSR_IMU6AzRaw = BrokerData();
BrokerData VeSNSR_IMU6WxRaw = BrokerData();
BrokerData VeSNSR_IMU6WyRaw = BrokerData();
BrokerData VeSNSR_IMU6WzRaw = BrokerData();








