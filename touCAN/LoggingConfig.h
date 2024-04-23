
//Generated 2024-04-23 16:37:45.115733 with logging_helper.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

inline void logging_write_header(File logfile){
WRAP_SPI_MUTEX(logfile.print("Time");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeCRLR_b_ControlReadyFlag");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_I_CAN0_BatteryCurrent");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_v_CAN1_BatteryMAXCell");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_T_CAN1_BatteryMAXTemp");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBMSR_b_CAN1_BMSReporting");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", VeBPER_R_CAN1_ESRObserved");, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
}

inline void logging_write_line(File logfile){
WRAP_SPI_MUTEX(logfile.print((double)esp_timer_get_time() / 100000.0);, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeCRLR_b_ControlReadyFlag.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeBMSR_I_CAN0_BatteryCurrent.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeBMSR_v_CAN1_BatteryMAXCell.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeBMSR_T_CAN1_BatteryMAXTemp.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeBMSR_b_CAN1_BMSReporting.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(VeBPER_R_CAN1_ESRObserved.getValue());, portMAX_DELAY)
WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
}

#endif