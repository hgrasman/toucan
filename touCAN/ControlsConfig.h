#ifndef CONTROLCONFIG
#define CONTROLCONFIG

#include "iBSG_config.h"

#define DUAL_MOTOR_CART
#define ENABLE_REDUNDANT_BMS
#define ENABLE_LOGGING

#define PDGP_POWER_LIMIT 14000
#define MECH_POWER_LIMIT PDGP_POWER_LIMIT-13600
#define POWER_LIMIT_MINRPM 250
#define REGEN_POWER_LIMIT -50 //-5000
#define REGEN_IDLE_FORWARD_TQ 0
#define REGEN_LIMIT_MAX_REV_DAMPING 2.5
inline double REGEN_TAPER_FUNC(double rpm){
  if (rpm>0){
    return (-(log(abs(rpm)/1000+1)*50.0) + REGEN_IDLE_FORWARD_TQ);
  }else{
    return ( (log(abs(rpm)/1000+1)*10.0) + REGEN_IDLE_FORWARD_TQ);
  }
} 

#define CELL_MINIMUM_VOLTAGE 2.7
#define CELL_MAXIMUM_VOLTAGE 4.25
#define CELL_MAXIMUM_CURRENT 35.0
#define CELL_MAXIMUM_TEMP 60.0

#define PACK_CELLS_P 9
#define PACK_CELLS_S 13
#define PACK_ESR_EST 0.020 
#define PACK_MAXIMUM_TEMP CELL_MAXIMUM_TEMP
#define PACK_CURRENT_MAX PACK_CELLS_P*CELL_MAXIMUM_CURRENT
#define PACK_VOLTAGE_MIN PACK_CELLS_S*CELL_MINIMUM_VOLTAGE
#define PACK_VOLTAGE_MAX PACK_CELLS_S*CELL_MAXIMUM_VOLTAGE

//strengths for imu filtering
#define IMU_A_FILT_STRENGTH 0.9
#define IMU_W_FILT_STRENGTH 0.9

//SWA
#define SWA_ERROR_THRESHOLD .15
#define SWA_ERROR_RETURN_FILT .98
#define SWA_RANGE_DEG 180

//rate limit and return to center if SWA fails
#define VECTOR_RATE_FILT .90
#define VECTOR_CENTER_SPLIT .5

//state machine standby time
#define OFF_STATE_TIMEOUT 500
#define PRE_STATE_TIMEOUT 5000

//State machine comparisons
#define STALE_DATA_THRESHOLD 150000
#define PRECHARGE_END_AGREEMENT 1.0
#define BATTERY_AGREEMENT_THRESHOLD .1
#define USEDCELLS 13

//BMSR
#define SSV_LEARN_MAX_I 2.5 //small error even with bad esr estimate 
#define SSV_LEARN_STRENGTH .99 //slow
#define SSV_EST_MAX_I 350
#define SSV_EST_MAX_STRENGTH .05

#define ESR_LEARN_MIN_I 50
#define ESR_LEARN_MAX_I 350
#define ESR_EST_MAX_STRENGTH .025 //slow


#endif