/*
  Entry point and interface for the MC2 software

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Dual_CAN_Driver.h"
#include "Controls_Tasks.h"

// The setup function runs once when you press reset or power on the board.
void setup() {

  //TURN OFF THE GPS so I can use the goddamn serial port
  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);

  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n Starting...\n");
  Serial.flush();

  xTaskCreatePinnedToCore(
    VDKartTask
    ,  "Test Task" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  1  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    , 1 //run on the default core
    );

  if (CAN_SetupTasks() == CAN_SETUP_BOTH_SUCCESS){
    Serial.println("CAN HARDWARE INIT SUCCESS");
  }else{
    Serial.println("CAN HARDWARE INIT FAILURE");
  }

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}