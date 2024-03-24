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
      Serial.print("new is: ");Serial.println(this->value);
      this->last_update_time = esp_timer_get_time();
}

float BrokerData::getValue() { 
      Serial.print("val is: ");Serial.println(this->value);
      return (this->value);
}

//Data for other functions
BrokerData CAN0TorqueRequest = BrokerData();
BrokerData CAN1TorqueRequest= BrokerData();








