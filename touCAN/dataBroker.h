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
    BrokerData(double initial_value);
    void setValue(double new_value);
    double getValue(void);  
    double getValue(int64_t* time_Variable);  
    bool dataInitialized(void); 
  private:    
    volatile double value;
    volatile int64_t last_update_time;
};

#define WRAP_SERIAL_MUTEX(x,y) if(xSemaphoreTake( xSemaphore_SerialMutex, y) == pdTRUE ){ x xSemaphoreGive( xSemaphore_SerialMutex );}
extern SemaphoreHandle_t xSemaphore_SerialMutex;

//CRLR Controller Ring
extern BrokerData VeCRLR_b_ControlReadyFlag;

//VDKR VDKart Ring
extern BrokerData VeVDKR_tq_CAN0TorqueRequest;
extern BrokerData VeVDKR_tq_CAN1TorqueRequest;

//CANR CAN driver Ring
extern BrokerData VeCANR_rpm_CAN0iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN0iBSGOpMode;
extern BrokerData VeCANR_I_CAN0iBSGDCCurrent;
extern BrokerData VeCANR_tq_CAN0iBSGTorqueDelivered;

extern BrokerData VeCANR_rpm_CAN1iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN1iBSGOpMode;
extern BrokerData VeCANR_I_CAN1iBSGDCCurrent;
extern BrokerData VeCANR_tq_CAN1iBSGTorqueDelivered;

//SNSR Sensor Ring
extern BrokerData VeSNSR_a_IMU6AxRaw;
extern BrokerData VeSNSR_a_IMU6AyRaw;
extern BrokerData VeSNSR_a_IMU6AzRaw;
extern BrokerData VeSNSR_w_IMU6WxRaw;
extern BrokerData VeSNSR_w_IMU6WyRaw;
extern BrokerData VeSNSR_w_IMU6WzRaw;

extern BrokerData VeSNSR_a_IMU6AxFilt;
extern BrokerData VeSNSR_a_IMU6AyFilt;
extern BrokerData VeSNSR_a_IMU6AzFilt;

#endif