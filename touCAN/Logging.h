/*
  Software component responsible for interfacing with the SD card and broker for the purpose of logging data

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/
#ifndef LOGGING
#define LOGGING

#include "dataBroker.h"

#define LOGGING_SETUP_SUCCESS 0
#define LOGGING_SETUP_FAILURE 1

uint8_t Logging_SetupTasks(void);

#endif