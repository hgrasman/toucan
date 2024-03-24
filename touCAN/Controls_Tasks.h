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

//These are the variables which are to be made accessible to the core code
extern BrokerCANData CAN0BrokerData;
extern BrokerCANData CAN1BrokerData;

//This is the function handle which will be executed
#define VDKART_SETUP_SUCCESS 0b00
#define VDKART_SETUP_FAILURE 0b01
extern TaskHandle_t xTaskVDKartHandle;
uint8_t VDKart_SetupTasks(void);

#endif