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

