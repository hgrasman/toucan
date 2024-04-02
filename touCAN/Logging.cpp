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

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  File logfile = SD.open("kartlog.csv", FILE_WRITE);

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    WRAP_SPI_MUTEX(logfile.print((double)esp_timer_get_time() / 100000.0); logfile.print(", ");, portMAX_DELAY)
    WRAP_SPI_MUTEX(logfile.print(VeSNSR_a_IMU6AxFilt.getValue()); logfile.println("");, portMAX_DELAY)

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

