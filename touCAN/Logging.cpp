/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "Logging.h"
#include "LoggingConfig.h"
#include "pins.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

File logfile; //used for logging

void LoggingTask(void *pvParameters){

  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  //populate the header
  logging_write_header(logfile); // from LoggingConfig.h

  for(;;){

    //block on queue, run as fast as possible
    if( xQueueReceive( loggingQueue, &( dataToLog ), portMAX_DELAY ) ) 
     {
        logging_write_line(logfile, &dataToLog); //from LoggingConfig.h
     }

    logging_flush_buffer(logfile); //check if it's time to flush
  }
}

void LoggingQueueTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(LOG_RATE);

  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

    xLastWakeTime = xTaskGetTickCount(); // Initialize
    for (;;){

      logging_queue_data(); //run using 100% of SPI bus down time -> log data

      xTaskDelayUntil( &xLastWakeTime, xPeriod );
    }
}

uint8_t Logging_SetupTasks(void){
  
  if(!SD.begin(SD_SPI_CS_PIN, SPI, 50000000, "/sd", 1, false)){
    WRAP_SERIAL_MUTEX(Serial.println("SD Mount Failed");, pdMS_TO_TICKS(100))
    return (LOGGING_SETUP_FAILURE);
  }

  //init the log file 
  WRAP_SPI_MUTEX(logfile = SD.open("/kartlog.csv", FILE_WRITE);, portMAX_DELAY)
  if (!logfile){
    WRAP_SERIAL_MUTEX(Serial.println("SD Log File Open Failed");, pdMS_TO_TICKS(100))
    return (LOGGING_SETUP_FAILURE);
  }

  xTaskCreatePinnedToCore(
      LoggingTask
      ,  "SD Logger Writer" 
      ,  4096        
      ,  NULL 
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
  );

  xTaskCreatePinnedToCore(
      LoggingQueueTask
      ,  "SD Logger Queuer" 
      ,  4096        
      ,  NULL 
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
  );

  return (LOGGING_SETUP_SUCCESS);

}

