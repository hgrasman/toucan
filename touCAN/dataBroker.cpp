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
      value = 0;
      last_update_time = -1; //data not valid
}

void BrokerData::setValue(float new_value) {
      value = new_value;
      last_update_time = esp_timer_get_time();
}

void BrokerData::getValue(float* val, int64_t* time) {
      *val = value;
      *time = last_update_time;
}








