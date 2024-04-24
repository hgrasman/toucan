
//Generated 2024-04-24 14:32:57.884520 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_V_CAN0_BatteryVoltage;
  double LeSDLR_v_CAN1_BatteryMINCell;
  double LeSDLR_b_CAN1_BMSReporting;
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
  loggingMessage.LeSDLR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
  loggingMessage.LeSDLR_v_CAN1_BatteryMINCell = VeBMSR_v_CAN1_BatteryMINCell.getValue();
  loggingMessage.LeSDLR_b_CAN1_BMSReporting = VeBMSR_b_CAN1_BMSReporting.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN0_BatteryVoltage");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN1_BatteryMINCell");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeBMSR_b_CAN1_BMSReporting");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", LeSDLR_t_endTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  WRAP_SPI_MUTEX(logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_V_CAN0_BatteryVoltage, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_v_CAN1_BatteryMINCell, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_CAN1_BMSReporting, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");,portMAX_DELAY)
}

#endif