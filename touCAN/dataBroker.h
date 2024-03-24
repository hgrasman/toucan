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
    void getValue(float* val, int64_t* time);

  private:            
    float value;
    int64_t last_update_time;
};

typedef struct BrokerCANData{
  BrokerData inverter_temperature_pct;
  BrokerData torqueRequest;
}BrokerCANData;


#endif