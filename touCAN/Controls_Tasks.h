/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/
#ifndef CONTROLS_TASKS
#define CONTROLS_TASKS

#include "dataBroker.h"
#include "ControlsConfig.h"
#include "stdint.h"

//This is the function handle which will be executed
#define CONTROLS_SETUP_SUCCESS 0b00
#define VDKART_SETUP_FAILURE   0b01
#define HVPROP_SETUP_FAILURE   0b10
#define ALL_SETUP_FAILURE      0b11
extern TaskHandle_t xTaskVDKartHandle;
uint8_t Controls_SetupTasks(void);

#endif