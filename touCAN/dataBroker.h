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
#define WRAP_SPI_MUTEX(x,y) if(xSemaphoreTake( xSemaphore_CANSPIMutex, y) == pdTRUE ){ x xSemaphoreGive( xSemaphore_CANSPIMutex );}
extern SemaphoreHandle_t xSemaphore_CANSPIMutex;
#define WRAP_I2C_MUTEX(x,y) if(xSemaphoreTake( xSemaphore_I2CMutex, y) == pdTRUE ){ x xSemaphoreGive( xSemaphore_I2CMutex );}
extern SemaphoreHandle_t xSemaphore_I2CMutex;

//CRLR Controller Ring
extern BrokerData VeCRLR_b_ControlReadyFlag;

//HVPR high voltage propulsion
extern BrokerData VeHVPR_e_CANx_OpModeRequest;

//BMSR battery management observer
extern BrokerData VeBMSR_v_CAN0_BatteryMINCell;
extern BrokerData VeBMSR_v_CAN0_BatteryMAXCell;
extern BrokerData VeBMSR_V_CAN0_BatteryVoltage;
extern BrokerData VeBMSR_T_CAN0_BatteryMAXTemp;
extern BrokerData VeBMSR_I_CAN0_BatteryCurrent;

extern BrokerData VeBMSR_v_CAN1_BatteryMINCell;
extern BrokerData VeBMSR_v_CAN1_BatteryMAXCell;
extern BrokerData VeBMSR_V_CAN1_BatteryVoltage;
extern BrokerData VeBMSR_T_CAN1_BatteryMAXTemp;
extern BrokerData VeBMSR_I_CAN1_BatteryCurrent;

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

extern BrokerData VeCANR_I_CAN0_BatteryCurrent;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell1;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell2;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell3;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell4;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell5;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell6;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell7;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell8;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell9;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell10;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell11;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell12;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell13;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell14;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell15;
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell16;
extern BrokerData VeCANR_T_CAN0_BatteryTemp1;
extern BrokerData VeCANR_T_CAN0_BatteryTemp2;
extern BrokerData VeCANR_T_CAN0_BatteryTemp3;
extern BrokerData VeCANR_T_CAN0_BatteryTemp4;
extern BrokerData VeCANR_T_CAN0_BatteryTemp5;
extern BrokerData VeCANR_T_CAN0_BatteryTemp6;
extern BrokerData VeCANR_T_CAN0_BatteryTemp7;
extern BrokerData VeCANR_T_CAN0_BatteryTemp8;
extern BrokerData VeCANR_T_CAN0_BatteryTemp9;
extern BrokerData VeCANR_T_CAN0_BatteryTemp10;
extern BrokerData VeCANR_T_CAN0_BatteryTemp11;

extern BrokerData VeCANR_I_CAN1_BatteryCurrent;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell1;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell2;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell3;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell4;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell5;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell6;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell7;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell8;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell9;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell10;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell11;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell12;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell13;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell14;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell15;
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell16;
extern BrokerData VeCANR_T_CAN1_BatteryTemp1;
extern BrokerData VeCANR_T_CAN1_BatteryTemp2;
extern BrokerData VeCANR_T_CAN1_BatteryTemp3;
extern BrokerData VeCANR_T_CAN1_BatteryTemp4;
extern BrokerData VeCANR_T_CAN1_BatteryTemp5;
extern BrokerData VeCANR_T_CAN1_BatteryTemp6;
extern BrokerData VeCANR_T_CAN1_BatteryTemp7;
extern BrokerData VeCANR_T_CAN1_BatteryTemp8;
extern BrokerData VeCANR_T_CAN1_BatteryTemp9;
extern BrokerData VeCANR_T_CAN1_BatteryTemp10;
extern BrokerData VeCANR_T_CAN1_BatteryTemp11;

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
extern BrokerData VeSNSR_w_IMU6WxFilt;
extern BrokerData VeSNSR_w_IMU6WyFilt;
extern BrokerData VeSNSR_w_IMU6WzFilt;

#endif