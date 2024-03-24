/*
  Software component responsible for data transfer between threads

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#ifndef DATABROKER
#define DATABROKER

#include "stdint.h"

class BrokerData {
  public:
    BrokerData();
    void setValue(float new_value);
    float getValue(void);            
    float value;
    int64_t last_update_time;
};

extern BrokerData CAN0TorqueRequest;

#endif