
//Generated 2024-04-23 22:28:54.008616 with logging_helper.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 5
uint8_t flushCounter = 0;

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    WRAP_SPI_MUTEX(logfile.flush();, portMAX_DELAY)
    flushCounter = 0;
  }
}

inline void logging_write_header(File logfile){
WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTimeMs");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCRLR_b_ControlReadyFlag");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_CANx_OpModeRequest");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_HVTargetState");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN0_BatteryMINCell");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN0_BatteryMAXCell");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN0_BatteryVoltage");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_T_CAN0_BatteryMAXTemp");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_I_CAN0_BatteryCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_b_CAN0_BMSReporting");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN1_BatteryMINCell");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN1_BatteryMAXCell");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN1_BatteryVoltage");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_T_CAN1_BatteryMAXTemp");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_I_CAN1_BatteryCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_b_CAN1_BMSReporting");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_V_CAN0_SSVObserved");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_V_CAN0_SSVESREstimated");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_R_CAN0_ESRObserved");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_V_CAN1_SSVObserved");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_V_CAN1_SSVESREstimated");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_R_CAN1_ESRObserved");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeVDKR_tq_CAN0_TorqueRequest");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeVDKR_tq_CAN1_TorqueRequest");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_rpm_CAN0_iBSGRotorSpeed");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_e_CAN0_iBSGOpMode");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_I_CAN0_iBSGDCCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN0_iBSGTorqueDelivered");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_pct_CAN0_iBSGInverterTempRate");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_V_CAN0_iBSGVoltageDCLink");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_T_CAN0_iBSGStatorTemp");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_pct_CAN0_iBSGMotorTempRate");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN0_iBSGInstMinTrqLim");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN0_iBSGInstMaxTrqLim");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_rpm_CAN1_iBSGRotorSpeed");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_e_CAN1_iBSGOpMode");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_I_CAN1_iBSGDCCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN1_iBSGTorqueDelivered");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_pct_CAN1_iBSGInverterTempRate");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_V_CAN1_iBSGVoltageDCLink");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_T_CAN1_iBSGStatorTemp");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_pct_CAN1_iBSGMotorTempRate");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN1_iBSGInstMinTrqLim");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_tq_CAN1_iBSGInstMaxTrqLim");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_I_CAN0_BatteryCurrentRaw");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCANR_I_CAN1_BatteryCurrentRaw");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AxFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AyFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AzFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WxFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WyFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WzFilt");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile){
double LeSDLR_t_currentTimeMs = (double)esp_timer_get_time() / 1000.0;
double LeSDLR_b_ControlReadyFlag = VeCRLR_b_ControlReadyFlag.getValue();
double LeSDLR_e_CANx_OpModeRequest = VeHVPR_e_CANx_OpModeRequest.getValue();
double LeSDLR_e_HVTargetState = VeHVPR_e_HVTargetState.getValue();
double LeSDLR_v_CAN0_BatteryMINCell = VeBMSR_v_CAN0_BatteryMINCell.getValue();
double LeSDLR_v_CAN0_BatteryMAXCell = VeBMSR_v_CAN0_BatteryMAXCell.getValue();
double LeSDLR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
double LeSDLR_T_CAN0_BatteryMAXTemp = VeBMSR_T_CAN0_BatteryMAXTemp.getValue();
double LeSDLR_I_CAN0_BatteryCurrent = VeBMSR_I_CAN0_BatteryCurrent.getValue();
double LeSDLR_b_CAN0_BMSReporting = VeBMSR_b_CAN0_BMSReporting.getValue();
double LeSDLR_v_CAN1_BatteryMINCell = VeBMSR_v_CAN1_BatteryMINCell.getValue();
double LeSDLR_v_CAN1_BatteryMAXCell = VeBMSR_v_CAN1_BatteryMAXCell.getValue();
double LeSDLR_V_CAN1_BatteryVoltage = VeBMSR_V_CAN1_BatteryVoltage.getValue();
double LeSDLR_T_CAN1_BatteryMAXTemp = VeBMSR_T_CAN1_BatteryMAXTemp.getValue();
double LeSDLR_I_CAN1_BatteryCurrent = VeBMSR_I_CAN1_BatteryCurrent.getValue();
double LeSDLR_b_CAN1_BMSReporting = VeBMSR_b_CAN1_BMSReporting.getValue();
double LeSDLR_V_CAN0_SSVObserved = VeBPER_V_CAN0_SSVObserved.getValue();
double LeSDLR_V_CAN0_SSVESREstimated = VeBPER_V_CAN0_SSVESREstimated.getValue();
double LeSDLR_R_CAN0_ESRObserved = VeBPER_R_CAN0_ESRObserved.getValue();
double LeSDLR_V_CAN1_SSVObserved = VeBPER_V_CAN1_SSVObserved.getValue();
double LeSDLR_V_CAN1_SSVESREstimated = VeBPER_V_CAN1_SSVESREstimated.getValue();
double LeSDLR_R_CAN1_ESRObserved = VeBPER_R_CAN1_ESRObserved.getValue();
double LeSDLR_tq_CAN0_TorqueRequest = VeVDKR_tq_CAN0_TorqueRequest.getValue();
double LeSDLR_tq_CAN1_TorqueRequest = VeVDKR_tq_CAN1_TorqueRequest.getValue();
double LeSDLR_rpm_CAN0_iBSGRotorSpeed = VeCANR_rpm_CAN0_iBSGRotorSpeed.getValue();
double LeSDLR_e_CAN0_iBSGOpMode = VeCANR_e_CAN0_iBSGOpMode.getValue();
double LeSDLR_I_CAN0_iBSGDCCurrent = VeCANR_I_CAN0_iBSGDCCurrent.getValue();
double LeSDLR_tq_CAN0_iBSGTorqueDelivered = VeCANR_tq_CAN0_iBSGTorqueDelivered.getValue();
double LeSDLR_pct_CAN0_iBSGInverterTempRate = VeCANR_pct_CAN0_iBSGInverterTempRate.getValue();
double LeSDLR_V_CAN0_iBSGVoltageDCLink = VeCANR_V_CAN0_iBSGVoltageDCLink.getValue();
double LeSDLR_T_CAN0_iBSGStatorTemp = VeCANR_T_CAN0_iBSGStatorTemp.getValue();
double LeSDLR_pct_CAN0_iBSGMotorTempRate = VeCANR_pct_CAN0_iBSGMotorTempRate.getValue();
double LeSDLR_tq_CAN0_iBSGInstMinTrqLim = VeCANR_tq_CAN0_iBSGInstMinTrqLim.getValue();
double LeSDLR_tq_CAN0_iBSGInstMaxTrqLim = VeCANR_tq_CAN0_iBSGInstMaxTrqLim.getValue();
double LeSDLR_rpm_CAN1_iBSGRotorSpeed = VeCANR_rpm_CAN1_iBSGRotorSpeed.getValue();
double LeSDLR_e_CAN1_iBSGOpMode = VeCANR_e_CAN1_iBSGOpMode.getValue();
double LeSDLR_I_CAN1_iBSGDCCurrent = VeCANR_I_CAN1_iBSGDCCurrent.getValue();
double LeSDLR_tq_CAN1_iBSGTorqueDelivered = VeCANR_tq_CAN1_iBSGTorqueDelivered.getValue();
double LeSDLR_pct_CAN1_iBSGInverterTempRate = VeCANR_pct_CAN1_iBSGInverterTempRate.getValue();
double LeSDLR_V_CAN1_iBSGVoltageDCLink = VeCANR_V_CAN1_iBSGVoltageDCLink.getValue();
double LeSDLR_T_CAN1_iBSGStatorTemp = VeCANR_T_CAN1_iBSGStatorTemp.getValue();
double LeSDLR_pct_CAN1_iBSGMotorTempRate = VeCANR_pct_CAN1_iBSGMotorTempRate.getValue();
double LeSDLR_tq_CAN1_iBSGInstMinTrqLim = VeCANR_tq_CAN1_iBSGInstMinTrqLim.getValue();
double LeSDLR_tq_CAN1_iBSGInstMaxTrqLim = VeCANR_tq_CAN1_iBSGInstMaxTrqLim.getValue();
double LeSDLR_I_CAN0_BatteryCurrentRaw = VeCANR_I_CAN0_BatteryCurrentRaw.getValue();
double LeSDLR_I_CAN1_BatteryCurrentRaw = VeCANR_I_CAN1_BatteryCurrentRaw.getValue();
double LeSDLR_a_IMU6AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
double LeSDLR_a_IMU6AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
double LeSDLR_a_IMU6AzFilt = VeSNSR_a_IMU6AzFilt.getValue();
double LeSDLR_w_IMU6WxFilt = VeSNSR_w_IMU6WxFilt.getValue();
double LeSDLR_w_IMU6WyFilt = VeSNSR_w_IMU6WyFilt.getValue();
double LeSDLR_w_IMU6WzFilt = VeSNSR_w_IMU6WzFilt.getValue();

WRAP_SPI_MUTEX(\
logfile.print(LeSDLR_t_currentTimeMs, 4);\
logfile.print(", "); logfile.print(LeSDLR_b_ControlReadyFlag, 4);\
logfile.print(", "); logfile.print(LeSDLR_e_CANx_OpModeRequest, 4);\
logfile.print(", "); logfile.print(LeSDLR_e_HVTargetState, 4);\
logfile.print(", "); logfile.print(LeSDLR_v_CAN0_BatteryMINCell, 4);\
logfile.print(", "); logfile.print(LeSDLR_v_CAN0_BatteryMAXCell, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN0_BatteryVoltage, 4);\
logfile.print(", "); logfile.print(LeSDLR_T_CAN0_BatteryMAXTemp, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN0_BatteryCurrent, 4);\
logfile.print(", "); logfile.print(LeSDLR_b_CAN0_BMSReporting, 4);\
logfile.print(", "); logfile.print(LeSDLR_v_CAN1_BatteryMINCell, 4);\
logfile.print(", "); logfile.print(LeSDLR_v_CAN1_BatteryMAXCell, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN1_BatteryVoltage, 4);\
logfile.print(", "); logfile.print(LeSDLR_T_CAN1_BatteryMAXTemp, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN1_BatteryCurrent, 4);\
logfile.print(", "); logfile.print(LeSDLR_b_CAN1_BMSReporting, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN0_SSVObserved, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN0_SSVESREstimated, 4);\
logfile.print(", "); logfile.print(LeSDLR_R_CAN0_ESRObserved, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN1_SSVObserved, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN1_SSVESREstimated, 4);\
logfile.print(", "); logfile.print(LeSDLR_R_CAN1_ESRObserved, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN0_TorqueRequest, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN1_TorqueRequest, 4);\
logfile.print(", "); logfile.print(LeSDLR_rpm_CAN0_iBSGRotorSpeed, 4);\
logfile.print(", "); logfile.print(LeSDLR_e_CAN0_iBSGOpMode, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN0_iBSGDCCurrent, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN0_iBSGTorqueDelivered, 4);\
logfile.print(", "); logfile.print(LeSDLR_pct_CAN0_iBSGInverterTempRate, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN0_iBSGVoltageDCLink, 4);\
logfile.print(", "); logfile.print(LeSDLR_T_CAN0_iBSGStatorTemp, 4);\
logfile.print(", "); logfile.print(LeSDLR_pct_CAN0_iBSGMotorTempRate, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN0_iBSGInstMinTrqLim, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN0_iBSGInstMaxTrqLim, 4);\
logfile.print(", "); logfile.print(LeSDLR_rpm_CAN1_iBSGRotorSpeed, 4);\
logfile.print(", "); logfile.print(LeSDLR_e_CAN1_iBSGOpMode, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN1_iBSGDCCurrent, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN1_iBSGTorqueDelivered, 4);\
logfile.print(", "); logfile.print(LeSDLR_pct_CAN1_iBSGInverterTempRate, 4);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN1_iBSGVoltageDCLink, 4);\
logfile.print(", "); logfile.print(LeSDLR_T_CAN1_iBSGStatorTemp, 4);\
logfile.print(", "); logfile.print(LeSDLR_pct_CAN1_iBSGMotorTempRate, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN1_iBSGInstMinTrqLim, 4);\
logfile.print(", "); logfile.print(LeSDLR_tq_CAN1_iBSGInstMaxTrqLim, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN0_BatteryCurrentRaw, 4);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN1_BatteryCurrentRaw, 4);\
logfile.print(", "); logfile.print(LeSDLR_a_IMU6AxFilt, 4);\
logfile.print(", "); logfile.print(LeSDLR_a_IMU6AyFilt, 4);\
logfile.print(", "); logfile.print(LeSDLR_a_IMU6AzFilt, 4);\
logfile.print(", "); logfile.print(LeSDLR_w_IMU6WxFilt, 4);\
logfile.print(", "); logfile.print(LeSDLR_w_IMU6WyFilt, 4);\
logfile.print(", "); logfile.print(LeSDLR_w_IMU6WzFilt, 4);\
logfile.print("\n");,portMAX_DELAY)
}

#endif