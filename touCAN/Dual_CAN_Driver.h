/*
  Software component responsible for hardware interface of the two CAN networks.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#ifndef DUALCANDRIVER
#define DUALCANDRIVER

#include "Valeo_ext.h"
#include "bms_mc2.h"
#include "DataBroker.h"


//Struct for all the battery data
typedef struct BatteryBroker{
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell1;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell2;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell3;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell4;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell5;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell6;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell7;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell8;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell9;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell10;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell11;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell12;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell13;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell14;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell15;
  BrokerData* VeCANR_v_CANx_BatteryVoltageCell16;
  BrokerData* VeCANR_T_CANx_BatteryTemp1;
  BrokerData* VeCANR_T_CANx_BatteryTemp2;
  BrokerData* VeCANR_T_CANx_BatteryTemp3;
  BrokerData* VeCANR_T_CANx_BatteryTemp4;
  BrokerData* VeCANR_T_CANx_BatteryTemp5;
  BrokerData* VeCANR_T_CANx_BatteryTemp6;
  BrokerData* VeCANR_T_CANx_BatteryTemp7;
  BrokerData* VeCANR_T_CANx_BatteryTemp8;
  BrokerData* VeCANR_T_CANx_BatteryTemp9;
  BrokerData* VeCANR_T_CANx_BatteryTemp10;
  BrokerData* VeCANR_T_CANx_BatteryTemp11;
  BrokerData* VeCANR_I_CANx_BatteryCurrent;
}BatteryBroker;

extern BatteryBroker BatteryDataCAN0;
extern BatteryBroker BatteryDataCAN1;

#define CAN_SETUP_BOTH_SUCCESS 0b00
#define CAN_SETUP_CAN0_FAILURE 0b01
#define CAN_SETUP_CAN1_FAILURE 0b10
#define CAN_SETUP_BOTH_FAILURE 0b11

uint8_t CAN_SetupTasks(void);

#endif