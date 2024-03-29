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

#define IMU_A_FILT_STRENGTH 0.9

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double LeSNSR_a_AxFilt = 0;
double LeSNSR_a_AyFilt = 0;
double LeSNSR_a_AzFilt = 0;

void MCU6050Task(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  //wait on the global control signal
  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, portMAX_DELAY) 

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    WRAP_I2C_MUTEX(accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);, portMAX_DELAY)
    VeSNSR_a_IMU6AxRaw.setValue(IMU_DEFAULT_A_RES * (double)ax/IMU_MAX_INT);
    VeSNSR_a_IMU6AyRaw.setValue(IMU_DEFAULT_A_RES * (double)ay/IMU_MAX_INT);
    VeSNSR_a_IMU6AzRaw.setValue(IMU_DEFAULT_A_RES * (double)az/IMU_MAX_INT);
    VeSNSR_w_IMU6WxRaw.setValue(IMU_DEFAULT_W_RES * (double)gx/IMU_MAX_INT);
    VeSNSR_w_IMU6WyRaw.setValue(IMU_DEFAULT_W_RES * (double)gy/IMU_MAX_INT);
    VeSNSR_w_IMU6WzRaw.setValue(IMU_DEFAULT_W_RES * (double)gz/IMU_MAX_INT);

    //calculate filtered data
    LeSNSR_a_AxFilt = (VeSNSR_a_IMU6AxRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AxFilt*IMU_A_FILT_STRENGTH;
    LeSNSR_a_AyFilt = (VeSNSR_a_IMU6AyRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AyFilt*IMU_A_FILT_STRENGTH;
    LeSNSR_a_AzFilt = (VeSNSR_a_IMU6AzRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AzFilt*IMU_A_FILT_STRENGTH;
    VeSNSR_a_IMU6AxFilt.setValue(LeSNSR_a_AxFilt);
    VeSNSR_a_IMU6AyFilt.setValue(LeSNSR_a_AyFilt);
    VeSNSR_a_IMU6AzFilt.setValue(LeSNSR_a_AzFilt);


    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

uint8_t MPU6050_SetupTasks(void){

  Wire.begin(); //start I2C on default pins
  Serial.println("I2C Controller Configured");

  //default 2g and 250 deg/sec?: yes "namely +/- 2g and +/- 250 degrees/sec"
  accelgyro.initialize();
  if (!accelgyro.testConnection()){
    return(MCU6050_INIT_FAILURE);
  }
  Serial.println("MCU6050 Initialized");

  xTaskCreatePinnedToCore(
      MCU6050Task
      ,  "IMU MCU6050" 
      ,  2048        
      ,  NULL
      ,  7  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  return(MCU6050_INIT_SUCCESS);
  
}