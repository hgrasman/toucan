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
#include "iBSG_config.h"

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
  BrokerData torqueRequest;
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

    if( !ulTaskNotifyTake(pdTRUE, portMAX_DELAY )){
      Serial.println("CAN stuck waiting.");
    }

    while(params->CANx.checkReceive() == CAN_MSGAVAIL){
      params->CANx.readMsgBuf(&incomingData.arb_id, &incomingData.data_len, incomingData.data); //get data
      //do something with the data
      switch(incomingData.arb_id){
        case X8578_CAN_DB_CLIENT_EPIC_PMZ_A_FRAME_ID:
          Serial.print("found a A ");
          x8578_can_db_client_epic_pmz_a_unpack(&params->EncodingData.pmz_a_msg, incomingData.data, incomingData.data_len);
          break;

        case X8578_CAN_DB_CLIENT_EPIC_PMZ_C_FRAME_ID:
          Serial.print("found a C ");
          x8578_can_db_client_epic_pmz_c_unpack(&params->EncodingData.pmz_c_msg, incomingData.data, incomingData.data_len);
          break;
 
        case X8578_CAN_DB_CLIENT_EPIC_PMZ_E_FRAME_ID:
          Serial.print("found a E ");
          x8578_can_db_client_epic_pmz_e_unpack(&params->EncodingData.pmz_e_msg, incomingData.data, incomingData.data_len);
          break;

        case X8578_CAN_DB_CLIENT_EPIC_PMZ_G_FRAME_ID:
          Serial.print("found a G ");
          x8578_can_db_client_epic_pmz_g_unpack(&params->EncodingData.pmz_g_msg, incomingData.data, incomingData.data_len);
          break;

        case X8578_CAN_DB_CLIENT_EPIC_PMZ_H_FRAME_ID:
          Serial.print("found a H ");
          x8578_can_db_client_epic_pmz_h_unpack(&params->EncodingData.pmz_h_msg, incomingData.data, incomingData.data_len);
          Serial.println(params->EncodingData.pmz_h_msg.em_operating_mode_ext2);
          break;

        case X8578_CAN_DB_CLIENT_EPIC_PMZ_I_FRAME_ID:
          Serial.print("found a I ");
          x8578_can_db_client_epic_pmz_i_unpack(&params->EncodingData.pmz_i_msg, incomingData.data, incomingData.data_len);
          if (params->EncodingData.pmz_i_msg.bisg_diagnostic01) {Serial.print("Diagnostic: ");Serial.println(params->EncodingData.pmz_i_msg.bisg_diagnostic01, HEX);}
          break;

        default:
          break;
          Serial.print("ID: "); Serial.print(incomingData.arb_id);
          /*Serial.print("  [");
          for (int i = 0; i<incomingData.data_len;i++){
            Serial.print(incomingData.data[i]);Serial.print(" ");
          }
          Serial.println("]");*/
          Serial.println();
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
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  uint8_t data[CAN_TX_ARRAYSIZE];
  uint8_t f_hybrid_counter = 0;
  uint8_t w_mhev_counter = 0;
  uint8_t u_mhev_counter = 0;
  uint8_t t_mhev_counter = 0;

  for (;;){

    //WMHEV
    PrepareWMHEV(&params->EncodingData.w_mhev_msg, data, sizeof(data), 
                 0, W_MHEV_TORQUE_GRAD_POS, 0,  W_MHEV_TORQUE_GRAD_NEG, W_MHEV_DC_CURR_LIMIT, w_mhev_counter, W_MHEV_DC_VOLT_MIN);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_LENGTH, data) == CAN_OK ){
      w_mhev_counter = ComputeCounter(w_mhev_counter);
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //UMHEV
    PrepareUMHEV(&params->EncodingData.u_mhev_msg, data, sizeof(data),
                 u_mhev_counter, U_MHEV_TORQ_MIN_LIMIT, U_MHEV_TORQ_MAX_LIMIT, U_MHEV_DMP_MIN_LIMIT, U_MHEV_DMP_MAX_LIMIT);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_LENGTH, data) == CAN_OK ){
      u_mhev_counter = ComputeCounter(u_mhev_counter);
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //TMHEV
    PrepareTMHEV(&params->EncodingData.t_mhev_msg, data, sizeof(data),
                 0, 0, T_MHEV_REGEN_CURR_LIMIT, t_mhev_counter, T_MHEV_REGEN_VOLT_LIMIT);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_LENGTH, data) == CAN_OK ){
      t_mhev_counter = ComputeCounter(t_mhev_counter);
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //BCM
    PrepareBCM(&params->EncodingData.bcm_pmz_msg, data, sizeof(data),
               X8578_CAN_DB_CLIENT_BCM_PMZ_A_CAR_MODE_HS_NORMAL_CHOICE, 0, 0, X8578_CAN_DB_CLIENT_BCM_PMZ_A_POWER_MODE_RUNNING_2_CHOICE);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_BCM_PMZ_A_FRAME_ID, X8578_CAN_DB_CLIENT_BCM_PMZ_A_IS_EXTENDED, X8578_CAN_DB_CLIENT_BCM_PMZ_A_LENGTH, data) == CAN_OK ){
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //GWM
    PrepareGWM(&params->EncodingData.gwm_pmz_msg, data, sizeof(data),
               X8578_CAN_DB_CLIENT_GWM_PMZ_H_CRASH_STATUS_RCM_NO_CRASH_CHOICE, 0);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_GWM_PMZ_H_FRAME_ID, X8578_CAN_DB_CLIENT_GWM_PMZ_H_IS_EXTENDED, X8578_CAN_DB_CLIENT_GWM_PMZ_H_LENGTH, data) == CAN_OK ){
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //F Hybrid
    float LeTorqueRequest = params->torqueRequest.getValue();
    PrepareFHybrid(&params->EncodingData.f_hybrid_msg, data, sizeof(data), f_hybrid_counter,
                   X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_EM_OPERATING_MODE_REQ_EXT_TORQUE__MODE_CHOICE,
                   0,0,LeTorqueRequest);
    if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_LENGTH, data) == CAN_OK ){
      f_hybrid_counter = ComputeCounter(f_hybrid_counter);
      //vTaskDelay(pdMS_TO_TICKS(1));
    }

    xTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

/*
  This function is exposed to the main file.
  It initializes hardware, threads, and interrupts for the driver.
*/
uint8_t CAN_SetupTasks(void){

  uint8_t status = CAN_SETUP_BOTH_SUCCESS;

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN0.setMode(MCP_NORMAL);

    CAN0Params = {CAN0,ValeoEncodingCAN0, CAN0TorqueRequest};
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
    CAN1.setMode(MCP_NORMAL);

    CAN1Params = {CAN1,ValeoEncodingCAN1, CAN0TorqueRequest};
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