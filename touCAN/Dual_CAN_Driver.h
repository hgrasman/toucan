/*
  Software component responsible for hardware interface of the two CAN networks.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#define CAN_SETUP_BOTH_SUCCESS 0b00
#define CAN_SETUP_CAN0_FAILURE 0b01
#define CAN_SETUP_CAN1_FAILURE 0b10
#define CAN_SETUP_BOTH_FAILURE 0b11

uint8_t CAN_SetupTasks(void);