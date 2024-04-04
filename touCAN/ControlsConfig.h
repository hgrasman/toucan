#ifndef CONTROLCONFIG
#define CONTROLCONFIG

#define PDGP_POWER_LIMIT 14000
#define POWER_LIMIT_MARGIN 500
#define POWER_LIMIT_MINRPM 100
#define REGEN_POWER_LIMIT -5000
#define REGEN_TAPER_FUNC(rpm) -(log(rpm/1000 + 1.0)*25.0)

#define CELL_MINIMUM_VOLTAGE 2.8
#define CELL_MAXIMUM_VOLTAGE 4.25
#define CELL_MAXIMUM_CURRENT 35.0
#define CELL_MAXIMUM_TEMP 60.0

#define PACK_CELLS_P 9
#define PACK_CELLS_S 13
#define PACK_ESR_EST 0.010 
#define PACK_MAXIMUM_TEMP CELL_MAXIMUM_TEMP
#define PACK_CURRENT_MAX PACK_CELLS_P*CELL_MAXIMUM_CURRENT
#define PACK_VOLTAGE_MIN PACK_CELLS_P*CELL_MINIMUM_VOLTAGE
#define PACK_VOLTAGE_MIN PACK_CELLS_P*CELL_MAXIMUM_VOLTAGE

#define IMU_A_FILT_STRENGTH 0.9
#define IMU_W_FILT_STRENGTH 0.9

#define COUNTS_PER_VOLT (UINT16_MAX/5)

//SWA
#define SWA_ERROR_THRESHOLD .15
#define SWA_ERROR_RETURN_FILT .98
#define SWA_RANGE_DEG 180

#define VECTOR_RATE_FILT .90
#define VECTOR_CENTER_SPLIT .5

#define OFF_STATE_TIMEOUT 500

//State machine comparisons
#define STALE_DATA_THRESHOLD 50000
#define PRECHARGE_END_AGREEMENT 1.0
#define BATTERY_AGREEMENT_THRESHOLD .1
#define USEDCELLS 13


#endif