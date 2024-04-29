
//Generated 2024-04-29 13:35:34.052045 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 100
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_e_CANx_OpModeRequest;
  double LeSDLR_e_HVTargetState;
  double LeSDLR_e_Pre1RelayState;
  double LeSDLR_e_Rel1RelayState;
  double LeSDLR_e_Pre2RelayState;
  double LeSDLR_e_Rel2RelayState;
  double LeSDLR_V_CAN0_BatteryVoltage;
  double LeSDLR_V_CAN1_BatteryVoltage;
  double LeSDLR_tq_CAN0_TorqueRequest;
  double LeSDLR_tq_CAN1_TorqueRequest;
  double LeSDLR_e_MotorStateRequest;
  double LeSDLR_rpm_CAN0_iBSGRotorSpeed;
  double LeSDLR_e_CAN0_iBSGOpMode;
  double LeSDLR_rpm_CAN1_iBSGRotorSpeed;
  double LeSDLR_e_CAN1_iBSGOpMode;
  double LeSDLR_e_BatterySelectionTarget;
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
  loggingMessage.LeSDLR_e_CANx_OpModeRequest = VeHVPR_e_CANx_OpModeRequest.getValue();
  loggingMessage.LeSDLR_e_HVTargetState = VeHVPR_e_HVTargetState.getValue();
  loggingMessage.LeSDLR_e_Pre1RelayState = VeHVPR_e_Pre1RelayState.getValue();
  loggingMessage.LeSDLR_e_Rel1RelayState = VeHVPR_e_Rel1RelayState.getValue();
  loggingMessage.LeSDLR_e_Pre2RelayState = VeHVPR_e_Pre2RelayState.getValue();
  loggingMessage.LeSDLR_e_Rel2RelayState = VeHVPR_e_Rel2RelayState.getValue();
  loggingMessage.LeSDLR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
  loggingMessage.LeSDLR_V_CAN1_BatteryVoltage = VeBMSR_V_CAN1_BatteryVoltage.getValue();
  loggingMessage.LeSDLR_tq_CAN0_TorqueRequest = VeVDKR_tq_CAN0_TorqueRequest.getValue();
  loggingMessage.LeSDLR_tq_CAN1_TorqueRequest = VeVDKR_tq_CAN1_TorqueRequest.getValue();
  loggingMessage.LeSDLR_e_MotorStateRequest = VeVDKR_e_MotorStateRequest.getValue();
  loggingMessage.LeSDLR_rpm_CAN0_iBSGRotorSpeed = VeCANR_rpm_CAN0_iBSGRotorSpeed.getValue();
  loggingMessage.LeSDLR_e_CAN0_iBSGOpMode = VeCANR_e_CAN0_iBSGOpMode.getValue();
  loggingMessage.LeSDLR_rpm_CAN1_iBSGRotorSpeed = VeCANR_rpm_CAN1_iBSGRotorSpeed.getValue();
  loggingMessage.LeSDLR_e_CAN1_iBSGOpMode = VeCANR_e_CAN1_iBSGOpMode.getValue();
  loggingMessage.LeSDLR_e_BatterySelectionTarget = VeCHEN_e_BatterySelectionTarget.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_CANx_OpModeRequest");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_HVTargetState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_Pre1RelayState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_Rel1RelayState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_Pre2RelayState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_Rel2RelayState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN0_BatteryVoltage");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN1_BatteryVoltage");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeVDKR_tq_CAN0_TorqueRequest");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeVDKR_tq_CAN1_TorqueRequest");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeVDKR_e_MotorStateRequest");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCANR_rpm_CAN0_iBSGRotorSpeed");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCANR_e_CAN0_iBSGOpMode");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCANR_rpm_CAN1_iBSGRotorSpeed");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCANR_e_CAN1_iBSGOpMode");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCHEN_e_BatterySelectionTarget");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", LeSDLR_t_endTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  WRAP_SPI_MUTEX(logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CANx_OpModeRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_HVTargetState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_Pre1RelayState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_Rel1RelayState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_Pre2RelayState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_Rel2RelayState, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_BatteryVoltage, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN1_BatteryVoltage, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN0_TorqueRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_tq_CAN1_TorqueRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_MotorStateRequest, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_rpm_CAN0_iBSGRotorSpeed, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CAN0_iBSGOpMode, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_rpm_CAN1_iBSGRotorSpeed, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CAN1_iBSGOpMode, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_BatterySelectionTarget, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");,portMAX_DELAY)
}

#endif