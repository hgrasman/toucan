/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h"
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
QueueHandle_t xCAN0RxQueue;
QueueHandle_t xCAN1RxQueue;
CANTaskParams CAN0Params = {0,0};
CANTaskParams CAN1Params = {0,0};
MCP_CAN CAN0(CAN0_SPI_CS_PIN);
MCP_CAN CAN1(CAN1_SPI_CS_PIN);

ICACHE_RAM_ATTR void CAN0_RX_ISR(void){
  
  //put it in the rx queue internally (one register in HW so we need to clear it quick)
}
ICACHE_RAM_ATTR void CAN1_RX_ISR(void){
  //TODO
  
  //xQueueSendFromISR( xCAN1RxQueue, &cIn, &xHigherPriorityTaskWoken );
}

void CANRxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;
  
  CANData incomingData; 
  for (;;){
    if( xQueueReceive( params->xCANRxQueue, &incomingData, portMAX_DELAY ) )
     {
        Serial.print("Rx'd something: ");
        Serial.println(incomingData.arb_id);
     }
  }
}

void CANTxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;

  CANData testData;
  for (;;){
    testData.arb_id += 1;
    xQueueSend(params->xCANRxQueue, (void*) &testData, 0);
    testData.arb_id += 13;
    vTaskDelay(5000);
  }
}

uint8_t CAN_SetupTasks(void){

  uint8_t status = CAN_SETUP_BOTH_SUCCESS;

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    attachInterrupt(digitalPinToInterrupt(CAN0_INT_RX_PIN), CAN0_RX_ISR, FALLING);
    CAN0.setMode(MCP_NORMAL);

    xCAN0RxQueue = xQueueCreate(8, sizeof(CANData)); //TODO real queue
    CAN0Params = {xCAN0RxQueue, CAN0};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN0 Rx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN0Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  NULL // Task handle is not used here - simply pass NULL
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

  }else{
    status |= CAN_SETUP_CAN0_FAILURE;
  }

  /*
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    attachInterrupt(digitalPinToInterrupt(CAN1_INT_RX_PIN), CAN1_RX_ISR, FALLING);
    CAN1.setMode(MCP_NORMAL);

    xCAN1RxQueue = xQueueCreate(8, sizeof(uint8_t));
    CANTaskParams CAN1Params = {xCAN1RxQueue, NULL};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN1 Rx Task" // A name just for humans
      ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      ,  (void*) &CAN1Params // Task parameter which can modify the task behavior. This must be passed as pointer to void.
      ,  1  // Priority
      ,  NULL // Task handle is not used here - simply pass NULL
      , tskNO_AFFINITY //run on the default core
      );
  


  }else{
    status |= CAN_SETUP_CAN1_FAILURE;
  }*/

  return(status);

}