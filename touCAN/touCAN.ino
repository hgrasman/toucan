/*
  Entry point and interface for the MC2 software

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Dual_CAN_Driver.h"
#include "Controls_Tasks.h"
#include "Sensor_Tasks.h"
#include "dataBroker.h"

// The setup function runs once when you press reset or power on the board.
void setup() {

  // Initialize serial communication at 115200 bits per second:
  xSemaphore_SerialMutex = xSemaphoreCreateMutex();
  WRAP_SERIAL_MUTEX(\
    delay(500);\
    Serial.begin(115200);\
    delay(1000);\
    Serial.println("\n Starting...\n");\
    Serial.flush();\
    , portMAX_DELAY)

  if (MPU6050_SetupTasks() == MCU6050_INIT_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("IMU INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("IMU INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  if (VDKart_SetupTasks() == VDKART_SETUP_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("VDKART INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("VDKART INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  if (CAN_SetupTasks() == CAN_SETUP_BOTH_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("CAN HARDWARE INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("CAN HARDWARE INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  WRAP_SERIAL_MUTEX(Serial.println("\nStarting Threads\n");, pdMS_TO_TICKS(5))
  VeCRLR_b_ControlReadyFlag.setValue(0);


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}