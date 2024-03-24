/*
  Software component responsible for hardware interface of the two CAN networks.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#ifndef DUALCANDRIVER
#define DUALCANDRIVER

#include "Valeo_ext.h"
#include "DataBroker.h"

//Struct for CAN rx/tx intermedate encoding decoding
typedef struct ValeoEncodingData{
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
} ValeoEncodingData;

extern BrokerData CAN0TorqueRequest;

#define CAN_SETUP_BOTH_SUCCESS 0b00
#define CAN_SETUP_CAN0_FAILURE 0b01
#define CAN_SETUP_CAN1_FAILURE 0b10
#define CAN_SETUP_BOTH_FAILURE 0b11

uint8_t CAN_SetupTasks(void);

#endif