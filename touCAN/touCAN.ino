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
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while(!Serial); 

  if (CAN_SetupRxTasks() == CAN_SETUP_BOTH_SUCCESS){
    Serial.println("CAN HARDWARE INIT SUCCESS");
  };

  Serial.printf("Setup Ends\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
  //idle
}