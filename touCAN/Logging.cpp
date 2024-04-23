/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "Logging.h"
#include "pins.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

void LoggingTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);
  const TickType_t xFlushPeriod = pdMS_TO_TICKS(100);

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  //init the log file 
  File logfile;
  WRAP_SPI_MUTEX(logfile = SD.open("/kartlog.csv", FILE_WRITE);, portMAX_DELAY)
  if (!logfile){
    WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Log File Open Failed");, pdMS_TO_TICKS(100))
    vTaskDelete( NULL ); //end task
  }

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  uint8_t counter = 0;
  for(;;){

    WRAP_SPI_MUTEX(logfile.print((double)esp_timer_get_time() / 100000.0); logfile.print(", ");, portMAX_DELAY)
    WRAP_SPI_MUTEX(logfile.print(VeSNSR_a_IMU6AxFilt.getValue()); logfile.println("");, portMAX_DELAY)

    if (counter++ >= xFlushPeriod/xPeriod){
      WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
      counter = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

uint8_t Logging_SetupTasks(void){
  
  if(!SD.begin(SD_SPI_CS_PIN)){
        WRAP_SERIAL_MUTEX(Serial.println("SD Mount Failed");, pdMS_TO_TICKS(100))
        return (LOGGING_SETUP_FAILURE);
  }

  xTaskCreatePinnedToCore(
      LoggingTask
      ,  "SD Logger" 
      ,  2048        
      ,  NULL 
      ,  6  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
  );

  return (LOGGING_SETUP_SUCCESS);

}

