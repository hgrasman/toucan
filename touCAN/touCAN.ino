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
  delay(2000);
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n Starting...\n");
  Serial.flush();

  if (VDKart_SetupTasks() == VDKART_SETUP_SUCCESS){
    Serial.println("VDKART INIT SUCCESS");
  }else{
    Serial.println("VDKART INIT FAILURE");
  }

  if (CAN_SetupTasks() == CAN_SETUP_BOTH_SUCCESS){
    Serial.println("CAN HARDWARE INIT SUCCESS");
  }else{
    Serial.println("CAN HARDWARE INIT FAILURE");
  }


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}