
//Generated 2024-04-24 14:47:51.697787 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_b_ControlReadyFlag;
  double LeSDLR_e_CANx_OpModeRequest;
  double LeSDLR_e_HVTargetState;
  double LeSDLR_v_CAN0_BatteryMINCell;
  double LeSDLR_v_CAN0_BatteryMAXCell;
  double LeSDLR_V_CAN0_BatteryVoltage;
  double LeSDLR_T_CAN0_BatteryMAXTemp;
  double LeSDLR_I_CAN0_BatteryCurrent;
  double LeSDLR_b_CAN0_BMSReporting;
  double LeSDLR_v_CAN1_BatteryMINCell;
  double LeSDLR_v_CAN1_BatteryMAXCell;
  double LeSDLR_V_CAN1_BatteryVoltage;
  double LeSDLR_T_CAN1_BatteryMAXTemp;
  double LeSDLR_I_CAN1_BatteryCurrent;
  double LeSDLR_b_CAN1_BMSReporting;
  double LeSDLR_V_CAN0_SSVObserved;
  double LeSDLR_V_CAN0_SSVESREstimated;
  double LeSDLR_R_CAN0_ESRObserved;
  double LeSDLR_V_CAN1_SSVObserved;
  double LeSDLR_V_CAN1_SSVESREstimated;
  double LeSDLR_R_CAN1_ESRObserved;
  double LeSDLR_tq_CAN0_TorqueRequest;
  double LeSDLR_tq_CAN1_TorqueRequest;
  double LeSDLR_rpm_CAN0_iBSGRotorSpeed;
  double LeSDLR_e_CAN0_iBSGOpMode;
  double LeSDLR_I_CAN0_iBSGDCCurrent;
  double LeSDLR_tq_CAN0_iBSGTorqueDelivered;
  double LeSDLR_pct_CAN0_iBSGInverterTempRate;
  double LeSDLR_V_CAN0_iBSGVoltageDCLink;
  double LeSDLR_T_CAN0_iBSGStatorTemp;
  double LeSDLR_pct_CAN0_iBSGMotorTempRate;
  double LeSDLR_tq_CAN0_iBSGInstMinTrqLim;
  double LeSDLR_tq_CAN0_iBSGInstMaxTrqLim;
  double LeSDLR_rpm_CAN1_iBSGRotorSpeed;
  double LeSDLR_e_CAN1_iBSGOpMode;
  double LeSDLR_I_CAN1_iBSGDCCurrent;
  double LeSDLR_tq_CAN1_iBSGTorqueDelivered;
  double LeSDLR_pct_CAN1_iBSGInverterTempRate;
  double LeSDLR_V_CAN1_iBSGVoltageDCLink;
  double LeSDLR_T_CAN1_iBSGStatorTemp;
  double LeSDLR_pct_CAN1_iBSGMotorTempRate;
  double LeSDLR_tq_CAN1_iBSGInstMinTrqLim;
  double LeSDLR_tq_CAN1_iBSGInstMaxTrqLim;
  double LeSDLR_I_CAN0_BatteryCurrentRaw;
  double LeSDLR_I_CAN1_BatteryCurrentRaw;
  double LeSDLR_a_IMU6AxFilt;
  double LeSDLR_a_IMU6AyFilt;
  double LeSDLR_a_IMU6AzFilt;
  double LeSDLR_w_IMU6WxFilt;
  double LeSDLR_w_IMU6WyFilt;
  double LeSDLR_w_IMU6WzFilt;
  double LeSDLR_t_endTime;
}loggingMessage, dataToLog;

QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData ) );

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    WRAP_SPI_MUTEX(logfile.flush();, portMAX_DELAY)
    flushCounter = 0;
  }
}

inline bool logging_queue_data(void){
  loggingMessage.LeSDLR_t_currentTime = (double)esp_timer_get_time() / 1000000.0;
  loggingMessage.LeSDLR_b_ControlReadyFlag = VeCRLR_b_ControlReadyFlag.getValue();
  loggingMessage.LeSDLR_e_CANx_OpModeRequest = VeHVPR_e_CANx_OpModeRequest.getValue();
  loggingMessage.LeSDLR_e_HVTargetState = VeHVPR_e_HVTargetState.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryMINCell = VeBMSR_v_CAN0_BatteryMINCell.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryMAXCell = VeBMSR_v_CAN0_BatteryMAXCell.getValue();
  loggingMessage.LeSDLR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryMAXTemp = VeBMSR_T_CAN0_BatteryMAXTemp.getValue();
  loggingMessage.LeSDLR_I_CAN0_BatteryCurrent = VeBMSR_I_CAN0_BatteryCurrent.getValue();
  loggingMessage.LeSDLR_b_CAN0_BMSReporting = VeBMSR_b_CAN0_BMSReporting.getValue();
  loggingMessage.LeSDLR_v_CAN1_BatteryMINCell = VeBMSR_v_CAN1_BatteryMINCell.getValue();
  loggingMessage.LeSDLR_v_CAN1_BatteryMAXCell = VeBMSR_v_CAN1_BatteryMAXCell.getValue();
  loggingMessage.LeSDLR_V_CAN1_BatteryVoltage = VeBMSR_V_CAN1_BatteryVoltage.getValue();
  loggingMessage.LeSDLR_T_CAN1_BatteryMAXTemp = VeBMSR_T_CAN1_BatteryMAXTemp.getValue();
  loggingMessage.LeSDLR_I_CAN1_BatteryCurrent = VeBMSR_I_CAN1_BatteryCurrent.getValue();
  loggingMessage.LeSDLR_b_CAN1_BMSReporting = VeBMSR_b_CAN1_BMSReporting.getValue();
  loggingMessage.LeSDLR_V_CAN0_SSVObserved = VeBPER_V_CAN0_SSVObserved.getValue();
  loggingMessage.LeSDLR_V_CAN0_SSVESREstimated = VeBPER_V_CAN0_SSVESREstimated.getValue();
  loggingMessage.LeSDLR_R_CAN0_ESRObserved = VeBPER_R_CAN0_ESRObserved.getValue();
  loggingMessage.LeSDLR_V_CAN1_SSVObserved = VeBPER_V_CAN1_SSVObserved.getValue();
  loggingMessage.LeSDLR_V_CAN1_SSVESREstimated = VeBPER_V_CAN1_SSVESREstimated.getValue();
  loggingMessage.LeSDLR_R_CAN1_ESRObserved = VeBPER_R_CAN1_ESRObserved.getValue();
  loggingMessage.LeSDLR_tq_CAN0_TorqueRequest = VeVDKR_tq_CAN0_TorqueRequest.getValue();
  loggingMessage.LeSDLR_tq_CAN1_TorqueRequest = VeVDKR_tq_CAN1_TorqueRequest.getValue();
  loggingMessage.LeSDLR_rpm_CAN0_iBSGRotorSpeed = VeCANR_rpm_CAN0_iBSGRotorSpeed.getValue();
  loggingMessage.LeSDLR_e_CAN0_iBSGOpMode = VeCANR_e_CAN0_iBSGOpMode.getValue();
  loggingMessage.LeSDLR_I_CAN0_iBSGDCCurrent = VeCANR_I_CAN0_iBSGDCCurrent.getValue();
  loggingMessage.LeSDLR_tq_CAN0_iBSGTorqueDelivered = VeCANR_tq_CAN0_iBSGTorqueDelivered.getValue();
  loggingMessage.LeSDLR_pct_CAN0_iBSGInverterTempRate = VeCANR_pct_CAN0_iBSGInverterTempRate.getValue();
  loggingMessage.LeSDLR_V_CAN0_iBSGVoltageDCLink = VeCANR_V_CAN0_iBSGVoltageDCLink.getValue();
  loggingMessage.LeSDLR_T_CAN0_iBSGStatorTemp = VeCANR_T_CAN0_iBSGStatorTemp.getValue();
  loggingMessage.LeSDLR_pct_CAN0_iBSGMotorTempRate = VeCANR_pct_CAN0_iBSGMotorTempRate.getValue();
  loggingMessage.LeSDLR_tq_CAN0_iBSGInstMinTrqLim = VeCANR_tq_CAN0_iBSGInstMinTrqLim.getValue();
  loggingMessage.LeSDLR_tq_CAN0_iBSGInstMaxTrqLim = VeCANR_tq_CAN0_iBSGInstMaxTrqLim.getValue();
  loggingMessage.LeSDLR_rpm_CAN1_iBSGRotorSpeed = VeCANR_rpm_CAN1_iBSGRotorSpeed.getValue();
  loggingMessage.LeSDLR_e_CAN1_iBSGOpMode = VeCANR_e_CAN1_iBSGOpMode.getValue();
  loggingMessage.LeSDLR_I_CAN1_iBSGDCCurrent = VeCANR_I_CAN1_iBSGDCCurrent.getValue();
  loggingMessage.LeSDLR_tq_CAN1_iBSGTorqueDelivered = VeCANR_tq_CAN1_iBSGTorqueDelivered.getValue();
  loggingMessage.LeSDLR_pct_CAN1_iBSGInverterTempRate = VeCANR_pct_CAN1_iBSGInverterTempRate.getValue();
  loggingMessage.LeSDLR_V_CAN1_iBSGVoltageDCLink = VeCANR_V_CAN1_iBSGVoltageDCLink.getValue();
  loggingMessage.LeSDLR_T_CAN1_iBSGStatorTemp = VeCANR_T_CAN1_iBSGStatorTemp.getValue();
  loggingMessage.LeSDLR_pct_CAN1_iBSGMotorTempRate = VeCANR_pct_CAN1_iBSGMotorTempRate.getValue();
  loggingMessage.LeSDLR_tq_CAN1_iBSGInstMinTrqLim = VeCANR_tq_CAN1_iBSGInstMinTrqLim.getValue();
  loggingMessage.LeSDLR_tq_CAN1_iBSGInstMaxTrqLim = VeCANR_tq_CAN1_iBSGInstMaxTrqLim.getValue();
  loggingMessage.LeSDLR_I_CAN0_BatteryCurrentRaw = VeCANR_I_CAN0_BatteryCurrentRaw.getValue();
  loggingMessage.LeSDLR_I_CAN1_BatteryCurrentRaw = VeCANR_I_CAN1_BatteryCurrentRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AzFilt = VeSNSR_a_IMU6AzFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WxFilt = VeSNSR_w_IMU6WxFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WyFilt = VeSNSR_w_IMU6WyFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WzFilt = VeSNSR_w_IMU6WzFilt.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
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
  WRAP_SPI_MUTEX(logfile.print(", LeSDLR_t_endTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  WRAP_SPI_MUTEX(logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_ControlReadyFlag, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CANx_OpModeRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_HVTargetState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryMINCell, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryMAXCell, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_BatteryVoltage, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryMAXTemp, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_BatteryCurrent, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_CAN0_BMSReporting, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN1_BatteryMINCell, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN1_BatteryMAXCell, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN1_BatteryVoltage, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN1_BatteryMAXTemp, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN1_BatteryCurrent, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_CAN1_BMSReporting, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_SSVObserved, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_SSVESREstimated, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_R_CAN0_ESRObserved, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN1_SSVObserved, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN1_SSVESREstimated, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_R_CAN1_ESRObserved, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_TorqueRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN1_TorqueRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_rpm_CAN0_iBSGRotorSpeed, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CAN0_iBSGOpMode, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_iBSGDCCurrent, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGTorqueDelivered, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN0_iBSGInverterTempRate, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_iBSGVoltageDCLink, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_iBSGStatorTemp, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN0_iBSGMotorTempRate, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGInstMinTrqLim, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGInstMaxTrqLim, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_rpm_CAN1_iBSGRotorSpeed, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CAN1_iBSGOpMode, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN1_iBSGDCCurrent, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN1_iBSGTorqueDelivered, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN1_iBSGInverterTempRate, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN1_iBSGVoltageDCLink, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN1_iBSGStatorTemp, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN1_iBSGMotorTempRate, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN1_iBSGInstMinTrqLim, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN1_iBSGInstMaxTrqLim, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_BatteryCurrentRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN1_BatteryCurrentRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WxFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WyFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WzFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");,portMAX_DELAY)
}

#endif