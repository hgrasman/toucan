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
#include "ControlsConfig.h"

//Struct for CAN rx/tx intermedate encoding decoding
typedef struct CANEncodingData{
  //CAN TX
  x8578_can_db_client_pcm_pmz_w_mhev_t    w_mhev_msg; //Calibrate limits
  x8578_can_db_client_pcm_pmz_u_mhev_t    u_mhev_msg; 
  x8578_can_db_client_pcm_pmz_t_mhev_t    t_mhev_msg;
  x8578_can_db_client_bcm_pmz_a_t         bcm_pmz_msg; //set car to running
  x8578_can_db_client_gwm_pmz_h_t         gwm_pmz_msg; //crash status
  x8578_can_db_client_pcm_pmz_f_hybrid_t  f_hybrid_msg; //torque command

  //CAN RX
  x8578_can_db_client_epic_pmz_a_t        pmz_a_msg;
  x8578_can_db_client_epic_pmz_i_t        pmz_i_msg; //DTC Diagnostic
  x8578_can_db_client_epic_pmz_c_t        pmz_c_msg; //recv
  x8578_can_db_client_epic_pmz_g_t        pmz_g_msg; //recv
  x8578_can_db_client_epic_pmz_h_t        pmz_h_msg; //recv
  x8578_can_db_client_epic_pmz_e_t        pmz_e_msg; //recv

  bms_mc2_bms_data_a_t                    bms_a_msg;
  bms_mc2_bms_data_b_t                    bms_b_msg;
  bms_mc2_bms_data_c_t                    bms_c_msg;
  bms_mc2_bms_data_d_t                    bms_d_msg;
  bms_mc2_bms_data_e_t                    bms_e_msg;
  bms_mc2_bms_data_f_t                    bms_f_msg;
  bms_mc2_bms_data_g_t                    bms_g_msg;
} CANEncodingData;

//Struct for passing parameters to the CANTasks
typedef struct CANTaskParams{
  const uint8_t can_rx_pin;
  MCP_CAN CANx;
  CANEncodingData EncodingData;
  BatteryBroker BatteryData;
  BrokerData* VeVDKR_tq_CANx_TorqueRequest;
  BrokerData* VeHVPR_e_CANx_OpModeRequest;
  BrokerData* VeCANR_rpm_CANx_iBSGRotorSpeed;
  BrokerData* VeCANR_e_CANx_iBSGOpMode;
  BrokerData* VeCANR_I_CANx_iBSGDCCurrent;
  BrokerData* VeCANR_tq_CANx_iBSGTorqueDelivered;
  BrokerData* VeCANR_pct_CANx_iBSGInverterTempRate;
  BrokerData* VeCANR_V_CANx_iBSGVoltageDCLink;
  BrokerData* VeCANR_T_CANx_iBSGStatorTemp;
  BrokerData* VeCANR_pct_CANx_iBSGMotorTempRate;
  BrokerData* VeCANR_tq_CANx_iBSGInstMinTrqLim;
  BrokerData* VeCANR_tq_CANx_iBSGInstMaxTrqLim;
}CANTaskParams;

//Struct type to hold all CAN data
typedef struct CANData{ 
  uint32_t arb_id;
  uint8_t extended; //glorified boolean
  uint8_t data_len;
  uint8_t data[8];
}CANData;


//Unique data for CAN0
CANEncodingData EncodingCAN0;
MCP_CAN CAN0(CAN0_SPI_CS_PIN);
BatteryBroker BatteryDataCAN0 = { &VeCANR_v_CAN0_BatteryVoltageCell1, &VeCANR_v_CAN0_BatteryVoltageCell2, &VeCANR_v_CAN0_BatteryVoltageCell3, &VeCANR_v_CAN0_BatteryVoltageCell4, 
                                  &VeCANR_v_CAN0_BatteryVoltageCell5, &VeCANR_v_CAN0_BatteryVoltageCell6, &VeCANR_v_CAN0_BatteryVoltageCell7, &VeCANR_v_CAN0_BatteryVoltageCell8, 
                                  &VeCANR_v_CAN0_BatteryVoltageCell9, &VeCANR_v_CAN0_BatteryVoltageCell10, &VeCANR_v_CAN0_BatteryVoltageCell11, &VeCANR_v_CAN0_BatteryVoltageCell12,
                                  &VeCANR_v_CAN0_BatteryVoltageCell13, &VeCANR_v_CAN0_BatteryVoltageCell14, &VeCANR_v_CAN0_BatteryVoltageCell15, &VeCANR_v_CAN0_BatteryVoltageCell16,  
                                  &VeCANR_T_CAN0_BatteryTemp1, &VeCANR_T_CAN0_BatteryTemp2, &VeCANR_T_CAN0_BatteryTemp3, &VeCANR_T_CAN0_BatteryTemp4,
                                  &VeCANR_T_CAN0_BatteryTemp5, &VeCANR_T_CAN0_BatteryTemp6, &VeCANR_T_CAN0_BatteryTemp7, &VeCANR_T_CAN0_BatteryTemp8, 
                                  &VeCANR_T_CAN0_BatteryTemp9, &VeCANR_T_CAN0_BatteryTemp10, &VeCANR_T_CAN0_BatteryTemp11, &VeCANR_I_CAN0_BatteryCurrentRaw};
CANTaskParams CAN0Params = {CAN0_INT_RX_PIN, CAN0, EncodingCAN0, BatteryDataCAN0, &VeVDKR_tq_CAN0_TorqueRequest, &VeHVPR_e_CANx_OpModeRequest,
                            &VeCANR_rpm_CAN0_iBSGRotorSpeed, 
                            &VeCANR_e_CAN0_iBSGOpMode, &VeCANR_I_CAN0_iBSGDCCurrent, &VeCANR_tq_CAN0_iBSGTorqueDelivered,
                            &VeCANR_pct_CAN0_iBSGInverterTempRate, &VeCANR_V_CAN0_iBSGVoltageDCLink, &VeCANR_T_CAN0_iBSGStatorTemp,
                            &VeCANR_pct_CAN0_iBSGMotorTempRate, &VeCANR_tq_CAN0_iBSGInstMinTrqLim, &VeCANR_tq_CAN0_iBSGInstMaxTrqLim};

//CAN1 Data
CANEncodingData EncodingCAN1;
MCP_CAN CAN1(CAN1_SPI_CS_PIN);
BatteryBroker BatteryDataCAN1 = { &VeCANR_v_CAN1_BatteryVoltageCell1, &VeCANR_v_CAN1_BatteryVoltageCell2, &VeCANR_v_CAN1_BatteryVoltageCell3, &VeCANR_v_CAN1_BatteryVoltageCell4, 
                                  &VeCANR_v_CAN1_BatteryVoltageCell5, &VeCANR_v_CAN1_BatteryVoltageCell6, &VeCANR_v_CAN1_BatteryVoltageCell7, &VeCANR_v_CAN1_BatteryVoltageCell8, 
                                  &VeCANR_v_CAN1_BatteryVoltageCell9, &VeCANR_v_CAN1_BatteryVoltageCell10, &VeCANR_v_CAN1_BatteryVoltageCell11, &VeCANR_v_CAN1_BatteryVoltageCell12,
                                  &VeCANR_v_CAN1_BatteryVoltageCell13, &VeCANR_v_CAN1_BatteryVoltageCell14, &VeCANR_v_CAN1_BatteryVoltageCell15, &VeCANR_v_CAN1_BatteryVoltageCell16,  
                                  &VeCANR_T_CAN1_BatteryTemp1, &VeCANR_T_CAN1_BatteryTemp2, &VeCANR_T_CAN1_BatteryTemp3, &VeCANR_T_CAN1_BatteryTemp4,
                                  &VeCANR_T_CAN1_BatteryTemp5, &VeCANR_T_CAN1_BatteryTemp6, &VeCANR_T_CAN1_BatteryTemp7, &VeCANR_T_CAN1_BatteryTemp8, 
                                  &VeCANR_T_CAN1_BatteryTemp9, &VeCANR_T_CAN1_BatteryTemp10, &VeCANR_T_CAN1_BatteryTemp11, &VeCANR_I_CAN1_BatteryCurrentRaw};
CANTaskParams CAN1Params = {CAN1_INT_RX_PIN, CAN1, EncodingCAN1, BatteryDataCAN1, &VeVDKR_tq_CAN1_TorqueRequest, &VeHVPR_e_CANx_OpModeRequest,
                            &VeCANR_rpm_CAN1_iBSGRotorSpeed, 
                            &VeCANR_e_CAN1_iBSGOpMode, &VeCANR_I_CAN1_iBSGDCCurrent, &VeCANR_tq_CAN1_iBSGTorqueDelivered,
                            &VeCANR_pct_CAN1_iBSGInverterTempRate, &VeCANR_V_CAN1_iBSGVoltageDCLink, &VeCANR_T_CAN1_iBSGStatorTemp,
                            &VeCANR_pct_CAN1_iBSGMotorTempRate, &VeCANR_tq_CAN1_iBSGInstMinTrqLim, &VeCANR_tq_CAN1_iBSGInstMaxTrqLim};


static TaskHandle_t xTaskCAN0RxHandle;
static TaskHandle_t xTaskCAN1RxHandle;
static portMUX_TYPE CAN_INT_spinlock = portMUX_INITIALIZER_UNLOCKED;
//ISRs for each CAN interrupt with a macro bc I'm lazy
/*#define CREATE_CANx_ISR(CANx_RX_ISR, xTaskCANxRxHandle)\
ICACHE_RAM_ATTR void CANx_RX_ISR(void){\
  taskENTER_CRITICAL_ISR(&CAN_INT_spinlock);\
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;\
  vTaskNotifyGiveFromISR( xTaskCANxRxHandle, &xHigherPriorityTaskWoken );\
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );\
  taskEXIT_CRITICAL_ISR(&CAN_INT_spinlock);\
}
CREATE_CANx_ISR(CAN0_RX_ISR, xTaskCAN0RxHandle)
CREATE_CANx_ISR(CAN1_RX_ISR, xTaskCAN1RxHandle)*/

/*
  This function defines a receive thread for the CAN hardware defined by pvParameters.
  It blocks on a notification from the corresponding ISR. When data is available,
  values are read, decoded, and sent to the data broker. TODO data broker
*/
void CANRxTask(void *pvParameters){

  CANTaskParams *params = (CANTaskParams*) pvParameters;

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, portMAX_DELAY) 
  
  CANData incomingData; 
  for (;;){

    //check for data
    if (digitalRead(params->can_rx_pin)){vTaskDelay(1); continue;}

    //see if there actually is data
    bool msgAvailable;
    WRAP_SPI_MUTEX(msgAvailable = params->CANx.readMsgBuf(&incomingData.arb_id, &incomingData.data_len, incomingData.data) == CAN_OK;, portMAX_DELAY) //get data
    if (!msgAvailable){vTaskDelay(1); continue;}

    //do something with the data
    switch(incomingData.arb_id){
      case X8578_CAN_DB_CLIENT_EPIC_PMZ_A_FRAME_ID:
        x8578_can_db_client_epic_pmz_a_unpack(&params->EncodingData.pmz_a_msg, incomingData.data, incomingData.data_len);
        params->VeCANR_pct_CANx_iBSGInverterTempRate->setValue(x8578_can_db_client_epic_pmz_a_inverter_temperature_decode(params->EncodingData.pmz_a_msg.inverter_temperature));
        break;

      case X8578_CAN_DB_CLIENT_EPIC_PMZ_C_FRAME_ID:
        x8578_can_db_client_epic_pmz_c_unpack(&params->EncodingData.pmz_c_msg, incomingData.data, incomingData.data_len);
        params->VeCANR_tq_CANx_iBSGTorqueDelivered->setValue(x8578_can_db_client_epic_pmz_c_em_torque_ext_decode(params->EncodingData.pmz_c_msg.em_torque_ext));
        break;

      case X8578_CAN_DB_CLIENT_EPIC_PMZ_E_FRAME_ID:
        x8578_can_db_client_epic_pmz_e_unpack(&params->EncodingData.pmz_e_msg, incomingData.data, incomingData.data_len);
        params->VeCANR_I_CANx_iBSGDCCurrent->setValue(x8578_can_db_client_epic_pmz_e_em_current_dc_link_decode(params->EncodingData.pmz_e_msg.em_current_dc_link));
        params->VeCANR_T_CANx_iBSGStatorTemp->setValue(x8578_can_db_client_epic_pmz_e_em_temperature_decode(params->EncodingData.pmz_e_msg.em_temperature));
        params->VeCANR_pct_CANx_iBSGMotorTempRate->setValue(x8578_can_db_client_epic_pmz_e_temperature_rate_decode(params->EncodingData.pmz_e_msg.temperature_rate));
        break;

      case X8578_CAN_DB_CLIENT_EPIC_PMZ_G_FRAME_ID:
        x8578_can_db_client_epic_pmz_g_unpack(&params->EncodingData.pmz_g_msg, incomingData.data, incomingData.data_len);
        params->VeCANR_tq_CANx_iBSGInstMinTrqLim->setValue(x8578_can_db_client_epic_pmz_g_em_min_torque_limit_decode(params->EncodingData.pmz_g_msg.em_min_torque_limit));
        params->VeCANR_tq_CANx_iBSGInstMaxTrqLim->setValue(x8578_can_db_client_epic_pmz_g_em_max_torque_limit_decode(params->EncodingData.pmz_g_msg.em_max_torque_limit));
        break;

      case X8578_CAN_DB_CLIENT_EPIC_PMZ_H_FRAME_ID:
        x8578_can_db_client_epic_pmz_h_unpack(&params->EncodingData.pmz_h_msg, incomingData.data, incomingData.data_len);
        params->VeCANR_rpm_CANx_iBSGRotorSpeed->setValue(x8578_can_db_client_epic_pmz_h_bisg_speed_decode(params->EncodingData.pmz_h_msg.bisg_speed));
        params->VeCANR_e_CANx_iBSGOpMode->setValue(x8578_can_db_client_epic_pmz_h_em_operating_mode_ext2_decode(params->EncodingData.pmz_h_msg.em_operating_mode_ext2));
        params->VeCANR_V_CANx_iBSGVoltageDCLink->setValue(x8578_can_db_client_epic_pmz_h_em_voltage_dc_link_mv_decode(params->EncodingData.pmz_h_msg.em_voltage_dc_link_mv));
        break;

      case X8578_CAN_DB_CLIENT_EPIC_PMZ_I_FRAME_ID:
        x8578_can_db_client_epic_pmz_i_unpack(&params->EncodingData.pmz_i_msg, incomingData.data, incomingData.data_len);
        if (params->EncodingData.pmz_i_msg.bisg_diagnostic01) {Serial.print("Diagnostic: ");Serial.println(params->EncodingData.pmz_i_msg.bisg_diagnostic01);}
        break;

      case BMS_MC2_BMS_DATA_A_FRAME_ID:
        bms_mc2_bms_data_a_unpack(&params->EncodingData.bms_a_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell1->setValue(bms_mc2_bms_data_a_cell_1_voltage_decode(params->EncodingData.bms_a_msg.cell_1_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell2->setValue(bms_mc2_bms_data_a_cell_2_voltage_decode(params->EncodingData.bms_a_msg.cell_2_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell3->setValue(bms_mc2_bms_data_a_cell_3_voltage_decode(params->EncodingData.bms_a_msg.cell_3_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell4->setValue(bms_mc2_bms_data_a_cell_4_voltage_decode(params->EncodingData.bms_a_msg.cell_4_voltage));
        break;

      case BMS_MC2_BMS_DATA_B_FRAME_ID:
        bms_mc2_bms_data_b_unpack(&params->EncodingData.bms_b_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell5->setValue(bms_mc2_bms_data_b_cell_5_voltage_decode(params->EncodingData.bms_b_msg.cell_5_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell6->setValue(bms_mc2_bms_data_b_cell_6_voltage_decode(params->EncodingData.bms_b_msg.cell_6_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell7->setValue(bms_mc2_bms_data_b_cell_7_voltage_decode(params->EncodingData.bms_b_msg.cell_7_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell8->setValue(bms_mc2_bms_data_b_cell_8_voltage_decode(params->EncodingData.bms_b_msg.cell_8_voltage));
        break;

      case BMS_MC2_BMS_DATA_C_FRAME_ID:
        bms_mc2_bms_data_c_unpack(&params->EncodingData.bms_c_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell9->setValue(bms_mc2_bms_data_c_cell_9_voltage_decode(params->EncodingData.bms_c_msg.cell_9_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell10->setValue(bms_mc2_bms_data_c_cell_10_voltage_decode(params->EncodingData.bms_c_msg.cell_10_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell11->setValue(bms_mc2_bms_data_c_cell_11_voltage_decode(params->EncodingData.bms_c_msg.cell_11_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell12->setValue(bms_mc2_bms_data_c_cell_12_voltage_decode(params->EncodingData.bms_c_msg.cell_12_voltage));
        break;

      case BMS_MC2_BMS_DATA_D_FRAME_ID:
        bms_mc2_bms_data_d_unpack(&params->EncodingData.bms_d_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell13->setValue(bms_mc2_bms_data_d_cell_13_voltage_decode(params->EncodingData.bms_d_msg.cell_13_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell14->setValue(bms_mc2_bms_data_d_cell_14_voltage_decode(params->EncodingData.bms_d_msg.cell_14_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell15->setValue(bms_mc2_bms_data_d_cell_15_voltage_decode(params->EncodingData.bms_d_msg.cell_15_voltage));
        params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell16->setValue(bms_mc2_bms_data_d_cell_16_voltage_decode(params->EncodingData.bms_d_msg.cell_16_voltage));
        break;

      case BMS_MC2_BMS_DATA_E_FRAME_ID:
        bms_mc2_bms_data_e_unpack(&params->EncodingData.bms_e_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_T_CANx_BatteryTemp1->setValue(bms_mc2_bms_data_e_temp1_decode(params->EncodingData.bms_e_msg.temp1));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp2->setValue(bms_mc2_bms_data_e_temp2_decode(params->EncodingData.bms_e_msg.temp2));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp3->setValue(bms_mc2_bms_data_e_temp3_decode(params->EncodingData.bms_e_msg.temp3));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp4->setValue(bms_mc2_bms_data_e_temp4_decode(params->EncodingData.bms_e_msg.temp4));
        break;

      case BMS_MC2_BMS_DATA_F_FRAME_ID:
        bms_mc2_bms_data_f_unpack(&params->EncodingData.bms_f_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_T_CANx_BatteryTemp5->setValue(bms_mc2_bms_data_f_temp5_decode(params->EncodingData.bms_f_msg.temp5));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp6->setValue(bms_mc2_bms_data_f_temp6_decode(params->EncodingData.bms_f_msg.temp6));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp7->setValue(bms_mc2_bms_data_f_temp7_decode(params->EncodingData.bms_f_msg.temp7));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp8->setValue(bms_mc2_bms_data_f_temp8_decode(params->EncodingData.bms_f_msg.temp8));
        break;

      case BMS_MC2_BMS_DATA_G_FRAME_ID:
        bms_mc2_bms_data_g_unpack(&params->EncodingData.bms_g_msg, incomingData.data, incomingData.data_len);
        params->BatteryData.VeCANR_T_CANx_BatteryTemp9->setValue(bms_mc2_bms_data_g_temp9_decode(params->EncodingData.bms_g_msg.temp9));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp10->setValue(bms_mc2_bms_data_g_temp10_decode(params->EncodingData.bms_g_msg.temp10));
        params->BatteryData.VeCANR_T_CANx_BatteryTemp11->setValue(bms_mc2_bms_data_g_temp11_decode(params->EncodingData.bms_g_msg.temp11));
        params->BatteryData.VeCANR_I_CANx_BatteryCurrent->setValue(bms_mc2_bms_data_g_current_decode(params->EncodingData.bms_g_msg.current));
        break;

      default:
        //WRAP_SERIAL_MUTEX(Serial.print("ID: "); Serial.println(incomingData.arb_id);, pdMS_TO_TICKS(5)) 
        break;
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

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, portMAX_DELAY) 

  uint8_t configSelector = 0;
  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for (;;){

    //F Hybrid
    double LeCANR_tq_TorqueRequest = params->VeVDKR_tq_CANx_TorqueRequest->getValue();
    double LeCANR_e_OpModeRequest = params->VeHVPR_e_CANx_OpModeRequest->getValue();
    PrepareFHybrid(&params->EncodingData.f_hybrid_msg, data, sizeof(data), f_hybrid_counter,
                   LeCANR_e_OpModeRequest,0,0,LeCANR_tq_TorqueRequest);
    if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
      if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_F_HYBRID_LENGTH, data) == CAN_OK ){
        f_hybrid_counter = ComputeCounter(f_hybrid_counter);
      }
      xSemaphoreGive( xSemaphore_CANSPIMutex );
    }

    //Send configs one every 10ms tx period
    switch(configSelector++){
      case 0:
        //WMHEV
        PrepareWMHEV(&params->EncodingData.w_mhev_msg, data, sizeof(data), 
                    0, W_MHEV_TORQUE_GRAD_POS, 0,  W_MHEV_TORQUE_GRAD_NEG, W_MHEV_DC_CURR_LIMIT, w_mhev_counter, W_MHEV_DC_VOLT_MIN);
        if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
          if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_W_MHEV_LENGTH, data) == CAN_OK ){
            w_mhev_counter = ComputeCounter(w_mhev_counter);
          }
          xSemaphoreGive( xSemaphore_CANSPIMutex );
        }
        break;
      case 1:
        //UMHEV
        PrepareUMHEV(&params->EncodingData.u_mhev_msg, data, sizeof(data),
                    u_mhev_counter, U_MHEV_TORQ_MIN_LIMIT, U_MHEV_TORQ_MAX_LIMIT, U_MHEV_DMP_MIN_LIMIT, U_MHEV_DMP_MAX_LIMIT);           
        if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
          if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_U_MHEV_LENGTH, data) == CAN_OK ){
            u_mhev_counter = ComputeCounter(u_mhev_counter);
          }
          xSemaphoreGive( xSemaphore_CANSPIMutex );
        }
        break;
      case 2:
        //TMHEV
        PrepareTMHEV(&params->EncodingData.t_mhev_msg, data, sizeof(data),
                    0, 0, T_MHEV_REGEN_CURR_LIMIT, t_mhev_counter, T_MHEV_REGEN_VOLT_LIMIT);
        if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
          if (params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_FRAME_ID, X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_IS_EXTENDED, X8578_CAN_DB_CLIENT_PCM_PMZ_T_MHEV_LENGTH, data) == CAN_OK ){
            t_mhev_counter = ComputeCounter(t_mhev_counter);
          }
          xSemaphoreGive( xSemaphore_CANSPIMutex );
        }
        break;
      case 3:
        //BCM
        PrepareBCM(&params->EncodingData.bcm_pmz_msg, data, sizeof(data),
                  X8578_CAN_DB_CLIENT_BCM_PMZ_A_CAR_MODE_HS_NORMAL_CHOICE, 0, 0, X8578_CAN_DB_CLIENT_BCM_PMZ_A_POWER_MODE_RUNNING_2_CHOICE);
        if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
          params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_BCM_PMZ_A_FRAME_ID, X8578_CAN_DB_CLIENT_BCM_PMZ_A_IS_EXTENDED, X8578_CAN_DB_CLIENT_BCM_PMZ_A_LENGTH, data);
          xSemaphoreGive( xSemaphore_CANSPIMutex );
        }
        break;
      case 4:
        //GWM
        PrepareGWM(&params->EncodingData.gwm_pmz_msg, data, sizeof(data),
                  X8578_CAN_DB_CLIENT_GWM_PMZ_H_CRASH_STATUS_RCM_NO_CRASH_CHOICE, 0);
        if(xSemaphoreTake( xSemaphore_CANSPIMutex, portMAX_DELAY) == pdTRUE ){
          params->CANx.sendMsgBuf(X8578_CAN_DB_CLIENT_GWM_PMZ_H_FRAME_ID, X8578_CAN_DB_CLIENT_GWM_PMZ_H_IS_EXTENDED, X8578_CAN_DB_CLIENT_GWM_PMZ_H_LENGTH, data);
          xSemaphoreGive( xSemaphore_CANSPIMutex );
        }
        break;
      default:
        configSelector = 0; //reset
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

  WRAP_SERIAL_MUTEX(Serial.println("CAN0 Setup Beginning");, pdMS_TO_TICKS(5)) 
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN0.setMode(MCP_NORMAL);

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

      //attachInterrupt(digitalPinToInterrupt(CAN0_INT_RX_PIN), CAN0_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN0_FAILURE;
  }

  WRAP_SERIAL_MUTEX(Serial.println("CAN1 Setup Beginning");, pdMS_TO_TICKS(5)) 
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ){
    CAN1.setMode(MCP_NORMAL);

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

      //attachInterrupt(digitalPinToInterrupt(CAN1_INT_RX_PIN), CAN1_RX_ISR, FALLING);

  }else{
    status |= CAN_SETUP_CAN1_FAILURE;
  }

  return(status);

}