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
#include "Logging.h"
#include "Pins.h"
#include "dataBroker.h"

// The setup function runs once when you press reset or power on the board.
void setup() {

  //disable gps as LED indicator of setup time
  digitalWrite(GPS_ENABLE_PIN, LOW);
  pinMode(GPS_ENABLE_PIN, OUTPUT);

  // Initialize serial communication at 115200 bits per second:
  WRAP_SERIAL_MUTEX(\
    delay(500);\
    Serial.begin(115200);\
    delay(1000);\
    Serial.println("\n Starting...\n");\
    Serial.flush();\
    , portMAX_DELAY)

  //Sensor Setup
  if (Sensing_SetupTasks() == SENSING_INIT_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("SENSING INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("SENSING INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  //Controls Setup
  if (Controls_SetupTasks() == CONTROLS_SETUP_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("CONTROLS INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("CONTROLS INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  //CAN Setup
  if (CAN_SetupTasks() == CAN_SETUP_BOTH_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("CAN HARDWARE INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("CAN HARDWARE INIT FAILURE");, pdMS_TO_TICKS(5))
  }

  //SD Setup
  /*
  if (Logging_SetupTasks() == LOGGING_SETUP_SUCCESS){
    WRAP_SERIAL_MUTEX(Serial.println("SD INIT SUCCESS");, pdMS_TO_TICKS(5))
  }else{
    WRAP_SERIAL_MUTEX(Serial.println("SD INIT FAILURE");, pdMS_TO_TICKS(5))
  }*/

  WRAP_SERIAL_MUTEX(Serial.println("\nStarting Threads\n");, pdMS_TO_TICKS(5))
  digitalWrite(GPS_ENABLE_PIN, HIGH); //Turn on GPS to show setup is complete
  VeCRLR_b_ControlReadyFlag.setValue(0);


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}