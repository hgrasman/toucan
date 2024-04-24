
//Generated 2024-04-24 10:55:54.028636 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_a_IMU6AxFilt;
  double LeSDLR_a_IMU6AyFilt;
  double LeSDLR_a_IMU6AzFilt;
}loggingMessage;

QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData * ) );

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    WRAP_SPI_MUTEX(logfile.flush();, portMAX_DELAY)
    flushCounter = 0;
  }
}

inline bool logging_queue_data(void){
  loggingMessage.LeSDLR_t_currentTime = (double)esp_timer_get_time() / 1000000.0;
  loggingMessage.LeSDLR_a_IMU6AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AzFilt = VeSNSR_a_IMU6AzFilt.getValue();

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AxFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AyFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AzFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile){
  WRAP_SPI_MUTEX(logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");,portMAX_DELAY)
}

#endif