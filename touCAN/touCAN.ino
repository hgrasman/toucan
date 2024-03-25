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
  delay(1000);
  Serial.begin(115200);
  delay(2000);
  MUTEX_PRINTLN("\n Starting...\n");
  Serial.flush();

  if (MPU6050_SetupTasks() == MCU6050_INIT_SUCCESS){
    MUTEX_PRINTLN("IMU INIT SUCCESS");
  }else{
    MUTEX_PRINTLN("IMU INIT FAILURE");
  }

  if (VDKart_SetupTasks() == VDKART_SETUP_SUCCESS){
    MUTEX_PRINTLN("VDKART INIT SUCCESS");
  }else{
    MUTEX_PRINTLN("VDKART INIT FAILURE");
  }

  if (CAN_SetupTasks() == CAN_SETUP_BOTH_SUCCESS){
    MUTEX_PRINTLN("CAN HARDWARE INIT SUCCESS");
  }else{
    MUTEX_PRINTLN("CAN HARDWARE INIT FAILURE");
  }

  MUTEX_PRINTLN("Flagging Go");
  VeCRLR_ControlReadyFlag.setValue(0);


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}