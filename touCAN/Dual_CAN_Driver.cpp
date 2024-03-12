/*
  Software component responsible for hardware interface of the two CAN networks.

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

//Struct type to hold all CAN data
typedef struct CANData{ 
  uint32_t arb_id;
  uint8_t extended; //glorified boolean
  uint8_t data_len;
  uint8_t data[8];
}CANData;

//Struct for passing parameters to the CANTasks
typedef struct CANTaskParams{
  MCP_CAN CANx;
  ValeoEncodingData EncodingData;
}CANTaskParams;

//structs to hold intermediate data
ValeoEncodingData ValeoEncodingCAN0;
ValeoEncodingData ValeoEncodingCAN1;

//Necessary globals interacted with by threads
static portMUX_TYPE CAN_spinlock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t xTaskCAN0RxHandle;
static TaskHandle_t xTaskCAN1RxHandle;
CANTaskParams CAN0Params = {NULL}; //don't remove the {NULL} 
CANTaskParams CAN1Params = {NULL}; //there is no constructor and the compiler will cry
MCP_CAN CAN0(CAN0_SPI_CS_PIN);
MCP_CAN CAN1(CAN1_SPI_CS_PIN);


//ISRs for each CAN interrupt with a macro bc I'm lazy
#define CREATE_CANx_ISR(CANx_RX_ISR, xTaskCANxRxHandle)\
ICACHE_RAM_ATTR void CANx_RX_ISR(void){\
  taskENTER_CRITICAL_ISR(&CAN_spinlock);\
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;\
  vTaskNotifyGiveFromISR( xTaskCANxRxHandle, &xHigherPriorityTaskWoken );\
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );\
  taskEXIT_CRITICAL_ISR(&CAN_spinlock);\
}
CREATE_CANx_ISR(CAN0_RX_ISR, xTaskCAN0RxHandle)
CREATE_CANx_ISR(CAN1_RX_ISR, xTaskCAN1RxHandle)

/*
  This function defines a receive thread for the CAN hardware defined by pvParameters.
  It blocks on a notification from the corresponding ISR. When data is available,
  values are read, decoded, and sent to the data broker. TODO data broker
*/
void CANRxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;
  
  CANData incomingData; 
  for (;;){
    if( ulTaskNotifyTake(pdTRUE, portMAX_DELAY ))
     {
        while(params->CANx.checkReceive() == CAN_MSGAVAIL){
          params->CANx.readMsgBuf(&incomingData.arb_id, &incomingData.data_len, incomingData.data); //get data
          //do something with the data
          switch(incomingData.arb_id){
            case X8578_CAN_DB_CLIENT_EPIC_PMZ_C_FRAME_ID:
              Serial.print("found a C");
              break;
          }
        }
     }
  }
}

/*
  This function defines a transmit thread for the CAN hardware defined by pvParameters.
  It WILL block until a frame to transmit is queued, encode, pack and send it.
  OR maybe it will transmit periodically and grab data from the broker. probably that
*/
void CANTxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;

  for (;;){
    uint8_t data[] = {1,1,1,1,1,1,1,1};
    params->CANx.sendMsgBuf(22, 8, data);
    vTaskDelay(1);
  }
}

/*
  This function is exposed to the main file.
  It initializes hardware, threads, and interrupts for the driver.
*/
uint8_t CAN_SetupTasks(void){

  uint8_t status = CAN_SETUP_BOTH_SUCCESS;

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN0.setMode(MCP_LOOPBACK);

    CAN0Params = {CAN0,ValeoEncodingCAN0};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN0 Rx Task" 
      ,  2048        
      ,  (void*) &CAN0Params 
      ,  8  // Priority
      ,  &xTaskCAN0RxHandle // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

      xTaskCreatePinnedToCore(
      CANTxTask
      ,  "CAN0 Tx Task" 
      ,  2048        
      ,  (void*) &CAN0Params 
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

      attachInterrupt(digitalPinToInterrupt(CAN0_INT_RX_PIN), CAN0_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN0_FAILURE;
  }

  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN1.setMode(MCP_LOOPBACK);

    CAN1Params = {CAN1,ValeoEncodingCAN1};
    xTaskCreatePinnedToCore(
      CANRxTask
      ,  "CAN1 Rx Task" 
      ,  2048        
      ,  (void*) &CAN1Params 
      ,  8  // Priority
      ,  &xTaskCAN1RxHandle // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

      xTaskCreatePinnedToCore(
      CANTxTask
      ,  "CAN1 Tx Task" 
      ,  2048        
      ,  (void*) &CAN1Params 
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

      attachInterrupt(digitalPinToInterrupt(CAN1_INT_RX_PIN), CAN1_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN1_FAILURE;
  }

  return(status);

}