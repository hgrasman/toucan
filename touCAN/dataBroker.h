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

//Exclusive hardware access
#define WRAP_SERIAL_MUTEX(x,y) if(xSemaphoreTake( xSemaphore_SerialMutex, y) == pdTRUE ){ x xSemaphoreGive( xSemaphore_SerialMutex );}
extern SemaphoreHandle_t xSemaphore_SerialMutex;
//CAN is often multiline so no define
extern SemaphoreHandle_t xSemaphore_CANSPIMutex;
#define WRAP_I2C_MUTEX(x,y) if(xSemaphoreTake( xSemaphore_I2CMutex, y) == pdTRUE ){ x xSemaphoreGive( xSemaphore_I2CMutex );}
extern SemaphoreHandle_t xSemaphore_I2CMutex;

//CRLR Controller Ring
extern BrokerData VeCRLR_b_ControlReadyFlag;

//VDKR VDKart Ring
extern BrokerData VeVDKR_tq_CAN0_TorqueRequest;
extern BrokerData VeVDKR_tq_CAN1_TorqueRequest;

//CANR CAN driver Ring
extern BrokerData VeCANR_rpm_CAN0_iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN0_iBSGOpMode;
extern BrokerData VeCANR_I_CAN0_iBSGDCCurrent;
extern BrokerData VeCANR_tq_CAN0_iBSGTorqueDelivered;
extern BrokerData VeCANR_pct_CAN0_iBSGInverterTempRate;
extern BrokerData VeCANR_V_CAN0_iBSGVoltageDCLink;
extern BrokerData VeCANR_T_CAN0_iBSGStatorTemp;
extern BrokerData VeCANR_pct_CAN0_iBSGMotorTempRate;
extern BrokerData VeCANR_tq_CAN0_iBSGInstMinTrqLim;
extern BrokerData VeCANR_tq_CAN0_iBSGInstMaxTrqLim;

extern BrokerData VeCANR_rpm_CAN1_iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN1_iBSGOpMode;
extern BrokerData VeCANR_I_CAN1_iBSGDCCurrent;
extern BrokerData VeCANR_tq_CAN1_iBSGTorqueDelivered;
extern BrokerData VeCANR_pct_CAN1_iBSGInverterTempRate;
extern BrokerData VeCANR_V_CAN1_iBSGVoltageDCLink;
extern BrokerData VeCANR_T_CAN1_iBSGStatorTemp;
extern BrokerData VeCANR_pct_CAN1_iBSGMotorTempRate;
extern BrokerData VeCANR_tq_CAN1_iBSGInstMinTrqLim;
extern BrokerData VeCANR_tq_CAN1_iBSGInstMaxTrqLim;

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