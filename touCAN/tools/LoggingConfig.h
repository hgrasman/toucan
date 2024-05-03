
//Generated 2024-05-02 22:39:44.639862 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
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
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  logfile.print("LeSDLR_t_currentTime");
  logfile.print(", LeSDLR_t_endTime");
  logfile.print("\n");
  logfile.flush();
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);
  logfile.print("\n");
}

#endif