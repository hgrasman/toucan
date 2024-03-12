/*
Fuctions implementing packet counter update functions and CRC as defined by Valeo Documentation

For EV Cartz, Kettering University

CONFIDENTIAL? It's all according to open standards but best not to share it.

-Henry Grasman

*/

#ifndef Valeo_ext
#define Valeo_ext

#include "x8578_can_db_client.h"
#include "stdint.h"

typedef struct ValeoEncodingData{
  //CAN TX
  x8578_can_db_client_pcm_pmz_w_mhev_t    w_mhev_msg; //Calibrate limits
  x8578_can_db_client_pcm_pmz_u_mhev_t    u_mhev_msg; 
  x8578_can_db_client_pcm_pmz_t_mhev_t    t_mhev_msg;
  x8578_can_db_client_bcm_pmz_a_t         bcm_pmz_msg; //set car to running
  x8578_can_db_client_gwm_pmz_h_t         gwm_pmz_msg; //crash status
  x8578_can_db_client_pcm_pmz_f_hybrid_t  f_hybrid_msg; //torque command

  //CAN RX
  x8578_can_db_client_epic_pmz_i_t        pmz_i_msg; //DTC Diagnostic
  x8578_can_db_client_epic_pmz_c_t        pmz_c_msg; //recv
  x8578_can_db_client_epic_pmz_g_t        pmz_g_msg; //recv
  x8578_can_db_client_epic_pmz_h_t        pmz_h_msg; //recv
  x8578_can_db_client_epic_pmz_e_t        pmz_e_msg; //recv
} ValeoEncodingData;

uint8_t ComputeCounter(uint8_t Counter);

uint8_t CheckCounter(uint8_t u8CounterPrev, uint8_t u8CounterToCks);

uint32_t ComputeCrc(uint8_t* MsgData, uint8_t CrcLength);

uint16_t ArrangeFHybrid(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_f_hybrid_t* src_p,
    uint16_t size);

uint16_t ArrangeWMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_w_mhev_t* src_p,
    uint16_t size);

uint16_t ArrangeTMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_t_mhev_t* src_p,
    uint16_t size);

uint16_t ArrangeUMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_u_mhev_t* src_p,
    uint16_t size);


#endif

