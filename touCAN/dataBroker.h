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
    BrokerData(float initial_value);
    void setValue(float new_value);
    float getValue(void);  
    float getValue(int64_t* time_Variable);  
    bool dataInitialized(void); 
  private:    
    volatile float value;
    volatile int64_t last_update_time;
};

#define MUTEX_PRINT(x) if( xSemaphoreTake( xSemaphore_SerialMutex, portMAX_DELAY ) == pdTRUE ){Serial.print(x); xSemaphoreGive( xSemaphore_SerialMutex );}
#define MUTEX_PRINTLN(x) if( xSemaphoreTake( xSemaphore_SerialMutex, portMAX_DELAY ) == pdTRUE ){Serial.println(x); xSemaphoreGive( xSemaphore_SerialMutex );}
extern SemaphoreHandle_t xSemaphore_SerialMutex;

extern BrokerData VeCRLR_ControlReadyFlag;

extern BrokerData VeVDKR_CAN0TorqueRequest;
extern BrokerData VeVDKR_CAN1TorqueRequest;

extern BrokerData VeSNSR_IMU6AxRaw;
extern BrokerData VeSNSR_IMU6AyRaw;
extern BrokerData VeSNSR_IMU6AzRaw;
extern BrokerData VeSNSR_IMU6WxRaw;
extern BrokerData VeSNSR_IMU6WyRaw;
extern BrokerData VeSNSR_IMU6WzRaw;

#endif