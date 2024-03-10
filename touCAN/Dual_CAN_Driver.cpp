/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h"
#include "pins.h"
#include <stdint.h> 
#include "Dual_CAN_Driver.h"
#include <SPI.h>
#include <mcp_can.h> //coryjfowler's library
#include <mcp_can_dfs.h>
#include "Valeo_ext/Valeo_ext.h"

typedef struct CANData{ 
  uint32_t arb_id;
  uint8_t data_len;
  uint8_t data[8];
}CANData;

typedef struct CANTaskParams{
  QueueHandle_t xCANRxQueue;
  MCP_CAN CANx;
}CANTaskParams;

//CAN Hardware interface
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t xTaskCAN0RxHandle = NULL;
static TaskHandle_t xTaskCAN1RxHandle = NULL;
CANTaskParams CAN0Params = {0,0};
CANTaskParams CAN1Params = {0,0};
MCP_CAN CAN0(CAN0_SPI_CS_PIN);
MCP_CAN CAN1(CAN1_SPI_CS_PIN);

ICACHE_RAM_ATTR void CAN0_RX_ISR(void){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( xTaskCAN0RxHandle, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
ICACHE_RAM_ATTR void CAN1_RX_ISR(void){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( xTaskCAN1RxHandle, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void CANRxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;
  QueueHandle_t xRxQueue = params->xCANRxQueue;
  
  CANData incomingData; 
  for (;;){
    if( ulTaskNotifyTake(pdTRUE, portMAX_DELAY ))
     {
        while(params->CANx.checkReceive() == CAN_MSGAVAIL){
          params->CANx.readMsgBuf(&incomingData.arb_id, &incomingData.data_len, incomingData.data);
        }
     }
  }
}

void CANTxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;

  CANData testData;
  for (;;){
    uint8_t data[] = {1,1,1,1,1,1,1,1};
    params->CANx.sendMsgBuf(22, 8, data);
    vTaskDelay(1);
  }
}

uint8_t CAN_SetupTasks(void){

  uint8_t status = CAN_SETUP_BOTH_SUCCESS;

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN0.setMode(MCP_LOOPBACK);

    CAN0Params = {NULL, CAN0};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN0 Rx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN0Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  &xTaskCAN0RxHandle // Task handle is not used here - simply pass NULL
      ,  tskNO_AFFINITY //run on the default core
      );

      xTaskCreatePinnedToCore(
      CANTxTask
      ,  "CAN0 Tx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN0Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  NULL // Task handle is not used here - simply pass NULL
      ,  tskNO_AFFINITY //run on the default core
      );

      attachInterrupt(digitalPinToInterrupt(CAN0_INT_RX_PIN), CAN0_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN0_FAILURE;
  }

  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN1.setMode(MCP_LOOPBACK);

    CAN1Params = {NULL, CAN1};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN1 Rx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN1Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  &xTaskCAN1RxHandle // Task handle is not used here - simply pass NULL
      ,  tskNO_AFFINITY //run on the default core
      );

      xTaskCreatePinnedToCore(
      CANTxTask
      ,  "CAN0 Tx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN1Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  NULL // Task handle is not used here - simply pass NULL
      ,  tskNO_AFFINITY //run on the default core
      );

      attachInterrupt(digitalPinToInterrupt(CAN1_INT_RX_PIN), CAN1_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN1_FAILURE;
  }

  return(status);

}