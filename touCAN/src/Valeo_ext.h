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

#define CAN_TX_ARRAYSIZE 8

uint8_t ComputeCounter(uint8_t Counter);

uint8_t CheckCounter(uint8_t u8CounterPrev, uint8_t u8CounterToCks);

uint32_t ComputeCrc(uint8_t* MsgData, uint8_t CrcLength);

uint16_t ArrangeFHybrid(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_f_hybrid_t* src_p,
    uint16_t size);

uint8_t PrepareFHybrid(
  struct x8578_can_db_client_pcm_pmz_f_hybrid_t* src_p,
  uint8_t* data,
  uint8_t size,
  uint8_t vscem_req_gp_counter,
  double em_operating_mode_req_ext,
  double em_voltage_dc_link_req_ext,
  double em_speed_request_ext,
  double em_torque_request_ext);

uint16_t ArrangeWMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_w_mhev_t* src_p,
    uint16_t size);

uint8_t PrepareWMHEV(
	struct x8578_can_db_client_pcm_pmz_w_mhev_t* src_p,
  uint8_t* data,
  uint8_t size,
  double bisg_calibration_id_req,
  double em_torque_gradient_pos,
  double engine_torque_ripple,
  double em_torque_gradient_neg,
  double em_cur_dc_link_motor_limit,
  uint8_t em_motor_limit_grp_counter,
  double em_vol_dc_link_motor_limit);

uint16_t ArrangeTMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_t_mhev_t* src_p,
    uint16_t size);

uint8_t PrepareTMHEV(
	struct x8578_can_db_client_pcm_pmz_t_mhev_t* src_p,
  uint8_t* data,
  uint8_t size,
  double toc_pump_dr_req,
  double em_coolant_pump_dr_req,
  double em_cur_dc_link_gen_limit,
  uint8_t em_gen_limit_grp_counter,
  double em_vol_dc_link_gen_limit);

uint16_t ArrangeUMHEV(
    uint8_t* dst_p,
    struct x8578_can_db_client_pcm_pmz_u_mhev_t* src_p,
    uint16_t size);

uint8_t PrepareUMHEV(
	struct x8578_can_db_client_pcm_pmz_u_mhev_t* src_p,
  uint8_t* data,
  uint8_t size,
  uint8_t vscem_torq_lim_gp_counter,
  double em_torque_min_limit,
  double em_torque_max_limit,
  double drvline_damp_torq_min_lim,
  double drvline_damp_torq_max_lim);

uint8_t PrepareBCM(
	struct x8578_can_db_client_bcm_pmz_a_t* src_p,
  uint8_t* data,
  uint8_t size,
  double car_mode_hs,
  double car_mode_hs_ub,
  double power_mode_ub,
  double power_mode);

uint8_t PrepareGWM(
	struct x8578_can_db_client_gwm_pmz_h_t* src_p,
  uint8_t* data,
  uint8_t size,
  double crash_status_rcm,
  double crash_status_rcm_ub);

#endif

