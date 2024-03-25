/*
  Software component responsible for interfacing with kart sensors. Primarily IMU and WSS

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/
#ifndef SENSOR_TASKS
#define SENSOR_TASKS

#include "dataBroker.h"
#include "stdint.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"

#define MCU6050_INIT_SUCCESS 0
#define MCU6050_INIT_FAILURE 1

#define IMU_MAX_INT INT16_MAX 
#define IMU_DEFAULT_A_RES 2.0
#define IMU_DEFAULT_W_RES 250.0
uint8_t MPU6050_SetupTasks(void);

#endif