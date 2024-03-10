/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#define CAN_SETUP_BOTH_SUCCESS 0
#define CAN_SETUP_CAN0_FAILURE 1
#define CAN_SETUP_CAN1_FAILURE 2
#define CAN_SETUP_BOTH_FAILURE 3

uint8_t CAN_SetupTasks(void);
void fHybridTxTask(void *pvParameters);