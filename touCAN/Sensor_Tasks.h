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
#include "ControlsConfig.h"
#include "Dual_CAN_Driver.h"
#include "stdint.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"

#define SENSING_INIT_SUCCESS 0
#define SENSING_INIT_FAILURE 1

#define IMU_MAX_INT INT16_MAX 
#define IMU_DEFAULT_A_RES 2.0
#define IMU_DEFAULT_W_RES 250.0

#define CeBMSR_e_NOPACK   0b00
#define CeBMSR_e_LPACK    0b01
#define CeBMSR_e_RPACK    0b10
#define CeBMSR_e_BOTHPACK 0b11

uint8_t Sensing_SetupTasks(void);

#endif