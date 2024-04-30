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
    bool setDefault(double default_value);
    double getValue(void);  
    double getValue(int64_t* time_Variable);  
    double getValue(int64_t* time_Variable, int64_t* last_time);
    bool dataInitialized(void); 
  private:    
    volatile double value;
    volatile int64_t last_update_time;
    volatile SemaphoreHandle_t access_mutex;
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
extern BrokerData VeHVPR_e_HVTargetState;

//BMSR battery management observer
extern BrokerData VeBMSR_v_CAN0_BatteryMINCell; //LOGPRECISION 3.
extern BrokerData VeBMSR_v_CAN0_BatteryMAXCell; //LOGPRECISION 3.
extern BrokerData VeBMSR_V_CAN0_BatteryVoltage; //LOGPRECISION 3.
extern BrokerData VeBMSR_T_CAN0_BatteryMAXTemp; //LOGPRECISION 3.
extern BrokerData VeBMSR_I_CAN0_BatteryCurrent; //LOGPRECISION 3.
extern BrokerData VeBMSR_b_CAN0_BMSReporting;

extern BrokerData VeBMSR_v_CAN1_BatteryMINCell; //LOGPRECISION 3.
extern BrokerData VeBMSR_v_CAN1_BatteryMAXCell; //LOGPRECISION 3.
extern BrokerData VeBMSR_V_CAN1_BatteryVoltage; //LOGPRECISION 3.
extern BrokerData VeBMSR_T_CAN1_BatteryMAXTemp; //LOGPRECISION 3.
extern BrokerData VeBMSR_I_CAN1_BatteryCurrent; //LOGPRECISION 3.
extern BrokerData VeBMSR_b_CAN1_BMSReporting;

//BPER
extern BrokerData VeBPER_V_CAN0_SSVObserved;  //LOGPRECISION 3.
extern BrokerData VeBPER_V_CAN0_SSVESREstimated; //LOGPRECISION 3.
extern BrokerData VeBPER_R_CAN0_ESRObserved; //LOGPRECISION 5.
extern BrokerData VeBPER_V_CAN1_SSVObserved; //LOGPRECISION 3.
extern BrokerData VeBPER_V_CAN1_SSVESREstimated; //LOGPRECISION 3.
extern BrokerData VeBPER_R_CAN1_ESRObserved; //LOGPRECISION 5.

//VDKR VDKart Ring
extern BrokerData VeVDKR_tq_CAN0_TorqueRequest; //LOGPRECISION 2.
extern BrokerData VeVDKR_tq_CAN1_TorqueRequest; //LOGPRECISION 2.

//CANR CAN driver Ring
extern BrokerData VeCANR_rpm_CAN0_iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN0_iBSGOpMode;
extern BrokerData VeCANR_I_CAN0_iBSGDCCurrent; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN0_iBSGTorqueDelivered; //LOGPRECISION 3.
extern BrokerData VeCANR_pct_CAN0_iBSGInverterTempRate; //LOGPRECISION 1
extern BrokerData VeCANR_V_CAN0_iBSGVoltageDCLink; //LOGPRECISION 3.
extern BrokerData VeCANR_T_CAN0_iBSGStatorTemp; //LOGPRECISION 3.
extern BrokerData VeCANR_pct_CAN0_iBSGMotorTempRate; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN0_iBSGInstMinTrqLim; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN0_iBSGInstMaxTrqLim; //LOGPRECISION 3.

extern BrokerData VeCANR_rpm_CAN1_iBSGRotorSpeed;
extern BrokerData VeCANR_e_CAN1_iBSGOpMode;
extern BrokerData VeCANR_I_CAN1_iBSGDCCurrent; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN1_iBSGTorqueDelivered; //LOGPRECISION 3.
extern BrokerData VeCANR_pct_CAN1_iBSGInverterTempRate; //LOGPRECISION 1.
extern BrokerData VeCANR_V_CAN1_iBSGVoltageDCLink; //LOGPRECISION 3.
extern BrokerData VeCANR_T_CAN1_iBSGStatorTemp; //LOGPRECISION 3.
extern BrokerData VeCANR_pct_CAN1_iBSGMotorTempRate; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN1_iBSGInstMinTrqLim; //LOGPRECISION 3.
extern BrokerData VeCANR_tq_CAN1_iBSGInstMaxTrqLim; //LOGPRECISION 3.

extern BrokerData VeCANR_I_CAN0_BatteryCurrentRaw; //LOGPRECISION 3.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell1; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell2; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell3; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell4; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell5; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell6; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell7; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell8; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell9; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell10; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell11; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell12; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell13; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell14; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell15; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN0_BatteryVoltageCell16; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp1; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp2; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp3; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp4; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp5; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp6; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp7; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp8; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp9; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp10; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN0_BatteryTemp11; //LOGPRECISION 2.

extern BrokerData VeCANR_I_CAN1_BatteryCurrentRaw; //LOGPRECISION 3.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell1; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell2; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell3; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell4; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell5; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell6; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell7; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell8; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell9; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell10; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell11; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell12; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell13; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell14; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell15; //LOGPRECISION 2.
extern BrokerData VeCANR_v_CAN1_BatteryVoltageCell16; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp1; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp2; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp3; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp4; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp5; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp6; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp7; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp8; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp9; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp10; //LOGPRECISION 2.
extern BrokerData VeCANR_T_CAN1_BatteryTemp11; //LOGPRECISION 2.

//SNSR Sensor Ring
extern BrokerData VeSNSR_a_IMU6AxRaw; //LOGPRECISION 4.
extern BrokerData VeSNSR_a_IMU6AyRaw; //LOGPRECISION 4.
extern BrokerData VeSNSR_a_IMU6AzRaw; //LOGPRECISION 4.
extern BrokerData VeSNSR_w_IMU6WxRaw; //LOGPRECISION 4.
extern BrokerData VeSNSR_w_IMU6WyRaw; //LOGPRECISION 4.
extern BrokerData VeSNSR_w_IMU6WzRaw; //LOGPRECISION 4.

extern BrokerData VeSNSR_a_IMU6AxFilt; //LOGPRECISION 4.
extern BrokerData VeSNSR_a_IMU6AyFilt; //LOGPRECISION 4.
extern BrokerData VeSNSR_a_IMU6AzFilt; //LOGPRECISION 4.
extern BrokerData VeSNSR_w_IMU6WxFilt;
extern BrokerData VeSNSR_w_IMU6WyFilt;
extern BrokerData VeSNSR_w_IMU6WzFilt;

//GPSR GPS RING
extern BrokerData VeGPSR_deg_GPSLatitude; //LOGPRECISION 8.
extern BrokerData VeGPSR_deg_GPSLongitude; //LOGPRECISION 8.
extern BrokerData VeGPSR_m_GPSAltitude; //LOGPRECISION 4.
extern BrokerData VeGPSR_deg_GPSHeading; //LOGPRECISION 2.
extern BrokerData VeGPSR_mps_GPSSpeed; //LOGPRECISION 2.
extern BrokerData VeGPSR_n_GPSSatellites; 
extern BrokerData VeGPSR_t_GPSMillisecondsUnix; //TODO days month year etc
extern BrokerData VeGPSR_e_GPSFixQuality;

#endif