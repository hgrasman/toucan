/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "Sensor_Tasks.h"
#include "pins.h"

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void MCU6050Task(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  //wait on the global control signal
  while (VeCRLR_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  MUTEX_PRINT(pcTaskGetTaskName(NULL)); MUTEX_PRINTLN(" Go");

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    VeSNSR_IMU6AxRaw.setValue(IMU_DEFAULT_A_RES * (double)ax/IMU_MAX_INT);
    VeSNSR_IMU6AyRaw.setValue(IMU_DEFAULT_A_RES * (double)ay/IMU_MAX_INT);
    VeSNSR_IMU6AzRaw.setValue(IMU_DEFAULT_A_RES * (double)az/IMU_MAX_INT);
    VeSNSR_IMU6WxRaw.setValue(IMU_DEFAULT_W_RES * (double)gx/IMU_MAX_INT);
    VeSNSR_IMU6WyRaw.setValue(IMU_DEFAULT_W_RES * (double)gy/IMU_MAX_INT);
    VeSNSR_IMU6WzRaw.setValue(IMU_DEFAULT_W_RES * (double)gz/IMU_MAX_INT);
    MUTEX_PRINT(VeSNSR_IMU6AxRaw.getValue()); MUTEX_PRINT(", ");
    MUTEX_PRINT(VeSNSR_IMU6AyRaw.getValue()); MUTEX_PRINT(", ");
    MUTEX_PRINT(VeSNSR_IMU6AzRaw.getValue()); MUTEX_PRINT(", ");
    MUTEX_PRINT(VeSNSR_IMU6WxRaw.getValue()); MUTEX_PRINT(", ");
    MUTEX_PRINT(VeSNSR_IMU6WyRaw.getValue()); MUTEX_PRINT(", ");
    MUTEX_PRINTLN(VeSNSR_IMU6WzRaw.getValue());

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

uint8_t MPU6050_SetupTasks(void){

  Wire.begin(); //start I2C on default pins
  MUTEX_PRINTLN("I2C Controller Configured");

  //default 2g and 250 deg/sec?: yes "namely +/- 2g and +/- 250 degrees/sec"
  accelgyro.initialize();
  if (!accelgyro.testConnection()){
    return(MCU6050_INIT_FAILURE);
  }
  MUTEX_PRINTLN("MCU6050 Initialized");

  xTaskCreatePinnedToCore(
      MCU6050Task
      ,  "IMU MCU6050" 
      ,  2048        
      ,  NULL
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  return(MCU6050_INIT_SUCCESS);
  
}