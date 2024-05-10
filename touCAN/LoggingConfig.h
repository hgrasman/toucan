
//Generated 2024-05-10 12:06:06.716767 with logging_helper_2.py for EV Kartz Kettering University
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
  double LeSDLR_V_CAN0_SSVObserved;
  double LeSDLR_V_CAN0_SSVESREstimated;
  double LeSDLR_R_CAN0_ESRObserved;
  double LeSDLR_tq_CAN0_TorqueRequest;
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
  double LeSDLR_I_CAN0_BatteryCurrentRaw;
  double LeSDLR_v_CAN0_BatteryVoltageCell1;
  double LeSDLR_v_CAN0_BatteryVoltageCell2;
  double LeSDLR_v_CAN0_BatteryVoltageCell3;
  double LeSDLR_v_CAN0_BatteryVoltageCell4;
  double LeSDLR_v_CAN0_BatteryVoltageCell5;
  double LeSDLR_v_CAN0_BatteryVoltageCell6;
  double LeSDLR_v_CAN0_BatteryVoltageCell7;
  double LeSDLR_v_CAN0_BatteryVoltageCell8;
  double LeSDLR_v_CAN0_BatteryVoltageCell9;
  double LeSDLR_v_CAN0_BatteryVoltageCell10;
  double LeSDLR_v_CAN0_BatteryVoltageCell11;
  double LeSDLR_v_CAN0_BatteryVoltageCell12;
  double LeSDLR_v_CAN0_BatteryVoltageCell13;
  double LeSDLR_v_CAN0_BatteryVoltageCell14;
  double LeSDLR_v_CAN0_BatteryVoltageCell15;
  double LeSDLR_v_CAN0_BatteryVoltageCell16;
  double LeSDLR_T_CAN0_BatteryTemp1;
  double LeSDLR_T_CAN0_BatteryTemp2;
  double LeSDLR_T_CAN0_BatteryTemp3;
  double LeSDLR_T_CAN0_BatteryTemp4;
  double LeSDLR_T_CAN0_BatteryTemp5;
  double LeSDLR_T_CAN0_BatteryTemp6;
  double LeSDLR_T_CAN0_BatteryTemp7;
  double LeSDLR_T_CAN0_BatteryTemp8;
  double LeSDLR_T_CAN0_BatteryTemp9;
  double LeSDLR_T_CAN0_BatteryTemp10;
  double LeSDLR_T_CAN0_BatteryTemp11;
  double LeSDLR_t_endTime;
}loggingMessage, dataToLog;

QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData ) );

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    logfile.flush();
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
  loggingMessage.LeSDLR_V_CAN0_SSVObserved = VeBPER_V_CAN0_SSVObserved.getValue();
  loggingMessage.LeSDLR_V_CAN0_SSVESREstimated = VeBPER_V_CAN0_SSVESREstimated.getValue();
  loggingMessage.LeSDLR_R_CAN0_ESRObserved = VeBPER_R_CAN0_ESRObserved.getValue();
  loggingMessage.LeSDLR_tq_CAN0_TorqueRequest = VeVDKR_tq_CAN0_TorqueRequest.getValue();
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
  loggingMessage.LeSDLR_I_CAN0_BatteryCurrentRaw = VeCANR_I_CAN0_BatteryCurrentRaw.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell1 = VeCANR_v_CAN0_BatteryVoltageCell1.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell2 = VeCANR_v_CAN0_BatteryVoltageCell2.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell3 = VeCANR_v_CAN0_BatteryVoltageCell3.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell4 = VeCANR_v_CAN0_BatteryVoltageCell4.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell5 = VeCANR_v_CAN0_BatteryVoltageCell5.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell6 = VeCANR_v_CAN0_BatteryVoltageCell6.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell7 = VeCANR_v_CAN0_BatteryVoltageCell7.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell8 = VeCANR_v_CAN0_BatteryVoltageCell8.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell9 = VeCANR_v_CAN0_BatteryVoltageCell9.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell10 = VeCANR_v_CAN0_BatteryVoltageCell10.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell11 = VeCANR_v_CAN0_BatteryVoltageCell11.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell12 = VeCANR_v_CAN0_BatteryVoltageCell12.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell13 = VeCANR_v_CAN0_BatteryVoltageCell13.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell14 = VeCANR_v_CAN0_BatteryVoltageCell14.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell15 = VeCANR_v_CAN0_BatteryVoltageCell15.getValue();
  loggingMessage.LeSDLR_v_CAN0_BatteryVoltageCell16 = VeCANR_v_CAN0_BatteryVoltageCell16.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp1 = VeCANR_T_CAN0_BatteryTemp1.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp2 = VeCANR_T_CAN0_BatteryTemp2.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp3 = VeCANR_T_CAN0_BatteryTemp3.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp4 = VeCANR_T_CAN0_BatteryTemp4.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp5 = VeCANR_T_CAN0_BatteryTemp5.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp6 = VeCANR_T_CAN0_BatteryTemp6.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp7 = VeCANR_T_CAN0_BatteryTemp7.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp8 = VeCANR_T_CAN0_BatteryTemp8.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp9 = VeCANR_T_CAN0_BatteryTemp9.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp10 = VeCANR_T_CAN0_BatteryTemp10.getValue();
  loggingMessage.LeSDLR_T_CAN0_BatteryTemp11 = VeCANR_T_CAN0_BatteryTemp11.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  logfile.print("LeSDLR_t_currentTime");
  logfile.print(", VeCRLR_b_ControlReadyFlag");
  logfile.print(", VeHVPR_e_CANx_OpModeRequest");
  logfile.print(", VeHVPR_e_HVTargetState");
  logfile.print(", VeBMSR_v_CAN0_BatteryMINCell");
  logfile.print(", VeBMSR_v_CAN0_BatteryMAXCell");
  logfile.print(", VeBMSR_V_CAN0_BatteryVoltage");
  logfile.print(", VeBMSR_T_CAN0_BatteryMAXTemp");
  logfile.print(", VeBMSR_I_CAN0_BatteryCurrent");
  logfile.print(", VeBMSR_b_CAN0_BMSReporting");
  logfile.print(", VeBPER_V_CAN0_SSVObserved");
  logfile.print(", VeBPER_V_CAN0_SSVESREstimated");
  logfile.print(", VeBPER_R_CAN0_ESRObserved");
  logfile.print(", VeVDKR_tq_CAN0_TorqueRequest");
  logfile.print(", VeCANR_rpm_CAN0_iBSGRotorSpeed");
  logfile.print(", VeCANR_e_CAN0_iBSGOpMode");
  logfile.print(", VeCANR_I_CAN0_iBSGDCCurrent");
  logfile.print(", VeCANR_tq_CAN0_iBSGTorqueDelivered");
  logfile.print(", VeCANR_pct_CAN0_iBSGInverterTempRate");
  logfile.print(", VeCANR_V_CAN0_iBSGVoltageDCLink");
  logfile.print(", VeCANR_T_CAN0_iBSGStatorTemp");
  logfile.print(", VeCANR_pct_CAN0_iBSGMotorTempRate");
  logfile.print(", VeCANR_tq_CAN0_iBSGInstMinTrqLim");
  logfile.print(", VeCANR_tq_CAN0_iBSGInstMaxTrqLim");
  logfile.print(", VeCANR_I_CAN0_BatteryCurrentRaw");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell1");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell2");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell3");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell4");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell5");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell6");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell7");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell8");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell9");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell10");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell11");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell12");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell13");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell14");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell15");
  logfile.print(", VeCANR_v_CAN0_BatteryVoltageCell16");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp1");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp2");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp3");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp4");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp5");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp6");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp7");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp8");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp9");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp10");
  logfile.print(", VeCANR_T_CAN0_BatteryTemp11");
  logfile.print(", LeSDLR_t_endTime");
  logfile.print("\n");
  logfile.flush();
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_ControlReadyFlag, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CANx_OpModeRequest, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_HVTargetState, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryMINCell, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryMAXCell, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_BatteryVoltage, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryMAXTemp, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_BatteryCurrent, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_CAN0_BMSReporting, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_SSVObserved, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_SSVESREstimated, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_R_CAN0_ESRObserved, 5);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_TorqueRequest, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_rpm_CAN0_iBSGRotorSpeed, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CAN0_iBSGOpMode, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_iBSGDCCurrent, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGTorqueDelivered, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN0_iBSGInverterTempRate, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_iBSGVoltageDCLink, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_iBSGStatorTemp, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_pct_CAN0_iBSGMotorTempRate, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGInstMinTrqLim, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_iBSGInstMaxTrqLim, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_I_CAN0_BatteryCurrentRaw, 3);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell1, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell2, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell3, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell4, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell5, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell6, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell7, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell8, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell9, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell10, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell11, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell12, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell13, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell14, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell15, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN0_BatteryVoltageCell16, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp1, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp2, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp3, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp4, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp5, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp6, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp7, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp8, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp9, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp10, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_T_CAN0_BatteryTemp11, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);
  logfile.print("\n");
}

#endif