/*
  Entry point and interface for the MC2 software

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Dual_CAN_Driver.h"
#include "Controls_Tasks.h"

// Dual Core device
#define CRITICAL_CORE 0
#define DEFAULT_CORE 1

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

    xTaskCreatePinnedToCore(
    VDKartTask
    ,  "VDKart Task" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  1  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    , DEFAULT_CORE //run on the default core
    );

    xTaskCreatePinnedToCore(
    CANRxTask
    ,  "CAN Rx Task" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  1  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    , DEFAULT_CORE //run on the default core
    );

  Serial.printf("Setup Ends\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
  //idle
}