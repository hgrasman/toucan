
//Generated 2024-04-23 21:46:01.023109 with logging_helper.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define FLUSH_RATE 50
uint8_t flushCounter = 0;

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    logfile.flush()
    flushCounter = 0;
  }
}
inline void logging_write_header(File logfile){
WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_V_CAN0_BatteryVoltage");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_I_CAN1_BatteryCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_b_CAN1_BMSReporting");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile){
double LeSDLR_t_currentTime = (double)esp_timer_get_time() / 100000.0;
double LeSDLR_V_CAN0_BatteryVoltage = VeBMSR_V_CAN0_BatteryVoltage.getValue();
double LeSDLR_I_CAN1_BatteryCurrent = VeBMSR_I_CAN1_BatteryCurrent.getValue();
double LeSDLR_b_CAN1_BMSReporting = VeBMSR_b_CAN1_BMSReporting.getValue();

WRAP_SPI_MUTEX(\
logfile.print(LeSDLR_t_currentTime);\
logfile.print(", "); logfile.print(LeSDLR_V_CAN0_BatteryVoltage);\
logfile.print(", "); logfile.print(LeSDLR_I_CAN1_BatteryCurrent);\
logfile.print(", "); logfile.print(LeSDLR_b_CAN1_BMSReporting);\
logfile.print("\n");,portMAX_DELAY)
}

#endif