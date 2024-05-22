/*
  Software component responsible for monitoring cart systems and issuing commands.

  Contributors:
    -Henry Grasman

  Kettering University
  EV Kartz 2023-24

*/

#include "Arduino.h" 
#include "driver/adc.h"
#include "Adafruit_GPS.h"
#include "Sensor_Tasks.h"
#include "pins.h"

Adafruit_GPS GPS(&Serial2); //on the gps pins

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double LeSNSR_a_AxFilt = 0;
double LeSNSR_a_AyFilt = 0;
double LeSNSR_a_AzFilt = 0;
double LeSNSR_w_WxFilt = 0;
double LeSNSR_w_WyFilt = 0;
double LeSNSR_w_WzFilt = 0;

//interrupt on WSS pin saving count and timestamp
ICACHE_RAM_ATTR void WSS_COUNT_ISR(void){
  VeWSSR_cnt_WSSPulseCount.setValueISR(VeWSSR_cnt_WSSPulseCount.getValue() + 1); //increment pulse count
}

void MCU6050Task(void *pvParameters){  // This is a task.
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  //wait on the global control signal
  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, portMAX_DELAY) 

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){
    
    WRAP_I2C_MUTEX(accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);, portMAX_DELAY)
    VeSNSR_a_IMU6AxRaw.setValue(-IMU_DEFAULT_A_RES * (double)ay/IMU_MAX_INT);
    VeSNSR_a_IMU6AyRaw.setValue(IMU_DEFAULT_A_RES * (double)ax/IMU_MAX_INT);
    VeSNSR_a_IMU6AzRaw.setValue(-IMU_DEFAULT_A_RES * (double)az/IMU_MAX_INT);
    VeSNSR_w_IMU6WxRaw.setValue(-IMU_DEFAULT_W_RES * (double)gy/IMU_MAX_INT);
    VeSNSR_w_IMU6WyRaw.setValue(-IMU_DEFAULT_W_RES * (double)gx/IMU_MAX_INT);
    VeSNSR_w_IMU6WzRaw.setValue(-IMU_DEFAULT_W_RES * (double)gz/IMU_MAX_INT);

    //calculate filtered data
    LeSNSR_a_AxFilt = (VeSNSR_a_IMU6AxRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AxFilt*IMU_A_FILT_STRENGTH;
    LeSNSR_a_AyFilt = (VeSNSR_a_IMU6AyRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AyFilt*IMU_A_FILT_STRENGTH;
    LeSNSR_a_AzFilt = (VeSNSR_a_IMU6AzRaw.getValue())*(1-IMU_A_FILT_STRENGTH) + LeSNSR_a_AzFilt*IMU_A_FILT_STRENGTH;
    LeSNSR_w_WxFilt = (VeSNSR_w_IMU6WxRaw.getValue())*(1-IMU_W_FILT_STRENGTH) + LeSNSR_w_WxFilt*IMU_W_FILT_STRENGTH;
    LeSNSR_w_WyFilt = (VeSNSR_w_IMU6WyRaw.getValue())*(1-IMU_W_FILT_STRENGTH) + LeSNSR_w_WyFilt*IMU_W_FILT_STRENGTH;
    LeSNSR_w_WzFilt = (VeSNSR_w_IMU6WzRaw.getValue())*(1-IMU_W_FILT_STRENGTH) + LeSNSR_w_WzFilt*IMU_W_FILT_STRENGTH;
    VeSNSR_a_IMU6AxFilt.setValue(LeSNSR_a_AxFilt);
    VeSNSR_a_IMU6AyFilt.setValue(LeSNSR_a_AyFilt);
    VeSNSR_a_IMU6AzFilt.setValue(LeSNSR_a_AzFilt);


    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


typedef struct BMSRTaskParams{
  BatteryBroker BatteryData;
  BrokerData* VeBMSR_v_CANx_BatteryMINCell;
  BrokerData* VeBMSR_v_CANx_BatteryMAXCell;
  BrokerData* VeBMSR_V_CANx_BatteryVoltage;
  BrokerData* VeBMSR_T_CANx_BatteryMAXTemp;
  BrokerData* VeBMSR_I_CANx_BatteryCurrent;
  BrokerData* VeBMSR_b_CANx_BMSReporting;;
  BrokerData* VeBPER_V_CANx_SSVObserved;
  BrokerData* VeBPER_V_CANx_SSVESREstimated;
  BrokerData* VeBPER_R_CANx_ESRObserved;
}BMSRTaskTaskParams;
BMSRTaskTaskParams BMSRCAN0TaskParams = { BatteryDataCAN0, &VeBMSR_v_CAN0_BatteryMINCell, &VeBMSR_v_CAN0_BatteryMAXCell, 
                                          &VeBMSR_V_CAN0_BatteryVoltage, &VeBMSR_T_CAN0_BatteryMAXTemp, &VeBMSR_I_CAN0_BatteryCurrent, &VeBMSR_b_CAN0_BMSReporting,
                                          &VeBPER_V_CAN0_SSVObserved, &VeBPER_V_CAN0_SSVESREstimated, &VeBPER_R_CAN0_ESRObserved};
BMSRTaskTaskParams BMSRCAN1TaskParams = { BatteryDataCAN1, &VeBMSR_v_CAN1_BatteryMINCell, &VeBMSR_v_CAN1_BatteryMAXCell, 
                                          &VeBMSR_V_CAN1_BatteryVoltage, &VeBMSR_T_CAN1_BatteryMAXTemp, &VeBMSR_I_CAN1_BatteryCurrent, &VeBMSR_b_CAN1_BMSReporting,
                                          &VeBPER_V_CAN1_SSVObserved, &VeBPER_V_CAN1_SSVESREstimated, &VeBPER_R_CAN1_ESRObserved};
//This task calculates its own metrics from raw battery data for use elsewhere
//BMSR
void BMSObserverTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  BMSRTaskTaskParams *params = (BMSRTaskTaskParams*) pvParameters;

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }
  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  double  LeBMSR_v_CellVoltages[16];
  int64_t LeBMSR_t_CellVoltagesFreshness[16];
  double  LeBMSR_T_ProbeTemps[11];
  int64_t LeBMSR_t_ProbeTempsFreshness[11];
  double  LeBMSR_I_PackCurrent;
  int64_t LeBMSR_I_PackCurrentFreshness;
  double  LeBPER_p_SSVESRLearnStrength;
  double  LeBPER_p_ESRLearnStrength;

  params->VeBPER_V_CANx_SSVObserved->setDefault(PACK_VOLTAGE_MIN);
  params->VeBPER_V_CANx_SSVESREstimated->setDefault(PACK_VOLTAGE_MIN);
  params->VeBPER_R_CANx_ESRObserved->setDefault(PACK_ESR_EST);
  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for(;;){

    //READ ALL THE DATA
    LeBMSR_v_CellVoltages[0] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell1->getValue(&LeBMSR_t_CellVoltagesFreshness[0]);
    LeBMSR_v_CellVoltages[1] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell2->getValue(&LeBMSR_t_CellVoltagesFreshness[1]);
    LeBMSR_v_CellVoltages[2] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell3->getValue(&LeBMSR_t_CellVoltagesFreshness[2]);
    LeBMSR_v_CellVoltages[3] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell4->getValue(&LeBMSR_t_CellVoltagesFreshness[3]);
    LeBMSR_v_CellVoltages[4] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell5->getValue(&LeBMSR_t_CellVoltagesFreshness[4]);
    LeBMSR_v_CellVoltages[5] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell6->getValue(&LeBMSR_t_CellVoltagesFreshness[5]);
    LeBMSR_v_CellVoltages[6] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell7->getValue(&LeBMSR_t_CellVoltagesFreshness[6]);
    LeBMSR_v_CellVoltages[7] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell8->getValue(&LeBMSR_t_CellVoltagesFreshness[7]);
    LeBMSR_v_CellVoltages[8] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell9->getValue(&LeBMSR_t_CellVoltagesFreshness[8]);
    LeBMSR_v_CellVoltages[9] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell10->getValue(&LeBMSR_t_CellVoltagesFreshness[9]);
    LeBMSR_v_CellVoltages[10] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell11->getValue(&LeBMSR_t_CellVoltagesFreshness[10]);
    LeBMSR_v_CellVoltages[11] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell12->getValue(&LeBMSR_t_CellVoltagesFreshness[11]);
    LeBMSR_v_CellVoltages[12] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell13->getValue(&LeBMSR_t_CellVoltagesFreshness[12]);
    LeBMSR_v_CellVoltages[13] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell14->getValue(&LeBMSR_t_CellVoltagesFreshness[13]);
    LeBMSR_v_CellVoltages[14] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell15->getValue(&LeBMSR_t_CellVoltagesFreshness[14]);
    LeBMSR_v_CellVoltages[15] = params->BatteryData.VeCANR_v_CANx_BatteryVoltageCell16->getValue(&LeBMSR_t_CellVoltagesFreshness[15]);

    LeBMSR_T_ProbeTemps[0] = params->BatteryData.VeCANR_T_CANx_BatteryTemp1->getValue(&LeBMSR_t_ProbeTempsFreshness[0]);
    LeBMSR_T_ProbeTemps[1] = params->BatteryData.VeCANR_T_CANx_BatteryTemp2->getValue(&LeBMSR_t_ProbeTempsFreshness[1]);
    LeBMSR_T_ProbeTemps[2] = params->BatteryData.VeCANR_T_CANx_BatteryTemp3->getValue(&LeBMSR_t_ProbeTempsFreshness[2]);
    LeBMSR_T_ProbeTemps[3] = params->BatteryData.VeCANR_T_CANx_BatteryTemp4->getValue(&LeBMSR_t_ProbeTempsFreshness[3]);
    LeBMSR_T_ProbeTemps[4] = params->BatteryData.VeCANR_T_CANx_BatteryTemp5->getValue(&LeBMSR_t_ProbeTempsFreshness[4]);
    LeBMSR_T_ProbeTemps[5] = params->BatteryData.VeCANR_T_CANx_BatteryTemp6->getValue(&LeBMSR_t_ProbeTempsFreshness[5]);
    LeBMSR_T_ProbeTemps[6] = params->BatteryData.VeCANR_T_CANx_BatteryTemp7->getValue(&LeBMSR_t_ProbeTempsFreshness[6]);
    LeBMSR_T_ProbeTemps[7] = params->BatteryData.VeCANR_T_CANx_BatteryTemp8->getValue(&LeBMSR_t_ProbeTempsFreshness[7]);
    LeBMSR_T_ProbeTemps[8] = params->BatteryData.VeCANR_T_CANx_BatteryTemp9->getValue(&LeBMSR_t_ProbeTempsFreshness[8]);
    LeBMSR_T_ProbeTemps[9] = params->BatteryData.VeCANR_T_CANx_BatteryTemp10->getValue(&LeBMSR_t_ProbeTempsFreshness[9]);
    LeBMSR_T_ProbeTemps[10] = params->BatteryData.VeCANR_T_CANx_BatteryTemp11->getValue(&LeBMSR_t_ProbeTempsFreshness[10]);

    LeBMSR_I_PackCurrent = params->BatteryData.VeCANR_I_CANx_BatteryCurrent->getValue(&LeBMSR_I_PackCurrentFreshness); //TODO conversions

    //find min/max cell voltage
    double LeBMSR_v_minVoltage = CELL_MAXIMUM_VOLTAGE;
    double LeBMSR_v_maxVoltage = CELL_MINIMUM_VOLTAGE;
    double LeBMSR_V_packVoltage = 0;
    bool LeBMSR_b_VoltagesFresh = true;
    bool LeBMSR_b_TempsFresh = true;
    bool LeBMSR_b_CurrentFresh = true;
    for (int i = 0; i < USEDCELLS; i++){
      if (LeBMSR_t_CellVoltagesFreshness[i] > STALE_DATA_THRESHOLD){
        LeBMSR_b_VoltagesFresh = false;
        break;
      }
      if (LeBMSR_v_CellVoltages[i] < LeBMSR_v_minVoltage){
        LeBMSR_v_minVoltage = LeBMSR_v_CellVoltages[i];
      }
      if (LeBMSR_v_CellVoltages[i] > LeBMSR_v_maxVoltage){
        LeBMSR_v_maxVoltage = LeBMSR_v_CellVoltages[i];
      }
      LeBMSR_V_packVoltage += LeBMSR_v_CellVoltages[i]; //accumulate series voltages
    }
    if (LeBMSR_b_VoltagesFresh){ //let the staleness propogate after 50ms
      params->VeBMSR_v_CANx_BatteryMINCell->setValue(LeBMSR_v_minVoltage);
      params->VeBMSR_v_CANx_BatteryMAXCell->setValue(LeBMSR_v_maxVoltage);
      params->VeBMSR_V_CANx_BatteryVoltage->setValue(LeBMSR_V_packVoltage);
    }
#ifdef ENABLE_REDUNDANT_BMS
    else{
      params->VeBMSR_v_CANx_BatteryMINCell->setValue(VeBMSR_v_CAN0_BatteryMINCell.getValue());
      params->VeBMSR_v_CANx_BatteryMAXCell->setValue(VeBMSR_v_CAN0_BatteryMAXCell.getValue());
      params->VeBMSR_V_CANx_BatteryVoltage->setValue(VeBMSR_V_CAN0_BatteryVoltage.getValue());
    }
#endif

    //do the same for temperature
    double LeBMSR_T_maxTemp = 0;
    for (int i = 0; i < 11; i++){
      if (LeBMSR_t_ProbeTempsFreshness[i] > STALE_DATA_THRESHOLD){
        LeBMSR_b_TempsFresh = false;
        break;
      }
      if (LeBMSR_T_ProbeTemps[i] > LeBMSR_T_maxTemp){
        LeBMSR_T_maxTemp = LeBMSR_T_ProbeTemps[i];
      }
    }
    if (LeBMSR_b_TempsFresh){ //let the staleness propogate after 50ms
      params->VeBMSR_T_CANx_BatteryMAXTemp->setValue(LeBMSR_T_maxTemp);
    }

    //pretty much just pass current right now
    LeBMSR_b_CurrentFresh  = LeBMSR_I_PackCurrentFreshness < STALE_DATA_THRESHOLD;
    if (LeBMSR_b_CurrentFresh){
      params->VeBMSR_I_CANx_BatteryCurrent->setValue(LeBMSR_I_PackCurrent);
    }

    //if the battery is properly reporting
    params->VeBMSR_b_CANx_BMSReporting->setValue(LeBMSR_b_VoltagesFresh && LeBMSR_b_TempsFresh && LeBMSR_b_CurrentFresh);


    //BATTERY PARAMETER ESTIMATION
    //Observes SSV at low current, estimates during current using set ESR, Estimates ESR at load 
    //update observation at low current
    if (abs(LeBMSR_I_PackCurrent) < SSV_LEARN_MAX_I){
      params->VeBPER_V_CANx_SSVObserved->setValue(params->VeBPER_V_CANx_SSVObserved->getValue()*SSV_LEARN_STRENGTH + LeBMSR_V_packVoltage*(1-SSV_LEARN_STRENGTH));
    } 

    //estimate SSV under load. learn most near low current
    if (abs(LeBMSR_I_PackCurrent) <= SSV_EST_MAX_I){
      LeBPER_p_SSVESRLearnStrength = (SSV_EST_MAX_I - abs(LeBMSR_I_PackCurrent))/SSV_EST_MAX_I * SSV_EST_MAX_STRENGTH;
    }else{
      LeBPER_p_SSVESRLearnStrength = 0;
    }
    double LeBPER_V_ESRProjectedSSV = LeBMSR_V_packVoltage + params->VeBPER_R_CANx_ESRObserved->getValue()*LeBMSR_I_PackCurrent; 
    params->VeBPER_V_CANx_SSVESREstimated->setValue(params->VeBPER_V_CANx_SSVESREstimated->getValue()*(1-LeBPER_p_SSVESRLearnStrength) + LeBPER_V_ESRProjectedSSV*LeBPER_p_SSVESRLearnStrength);

    //estimate ESR under heavy load 
    double LeBPER_R_ProjectedESR = (params->VeBPER_V_CANx_SSVObserved->getValue() - LeBMSR_V_packVoltage)/LeBMSR_I_PackCurrent;
    if (LeBPER_R_ProjectedESR<0){LeBPER_R_ProjectedESR=0;} //if it's unstable it will swing negative, this will drag the estimate back up
    if (abs(LeBMSR_I_PackCurrent) < ESR_LEARN_MIN_I){
      LeBPER_p_ESRLearnStrength = 0;
    }else if ((abs(LeBMSR_I_PackCurrent) > ESR_LEARN_MAX_I)){
      LeBPER_p_ESRLearnStrength = 1;
      params->VeBPER_R_CANx_ESRObserved->setValue(params->VeBPER_R_CANx_ESRObserved->getValue()*(1-LeBPER_p_ESRLearnStrength) + LeBPER_R_ProjectedESR*LeBPER_p_ESRLearnStrength);
    }else{
      LeBPER_p_ESRLearnStrength = (((abs(LeBMSR_I_PackCurrent)-ESR_LEARN_MAX_I) / (ESR_LEARN_MAX_I-ESR_LEARN_MIN_I))+1) * ESR_EST_MAX_STRENGTH;
      params->VeBPER_R_CANx_ESRObserved->setValue(params->VeBPER_R_CANx_ESRObserved->getValue()*(1-LeBPER_p_ESRLearnStrength) + LeBPER_R_ProjectedESR*LeBPER_p_ESRLearnStrength);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


//this task serves to monitor the gps and update the relevant values
//GPSR
#define KNOTS_PER_MPS 1.9438452
void GPSTask(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(5);

  while (VeCRLR_b_ControlReadyFlag.dataInitialized() != true){
    vTaskDelay(1);
  }

  WRAP_SERIAL_MUTEX(Serial.print(pcTaskGetTaskName(NULL)); Serial.println(" Go");, pdMS_TO_TICKS(100))

  xLastWakeTime = xTaskGetTickCount(); // Initialize
  for (;;){

    //get data
    while(GPS.available()){
      GPS.read();
    }

    //if a string is complete, parse the data
    if(GPS.newNMEAreceived()) {
      
      if (!GPS.parse(GPS.lastNMEA())){ continue; } //failed, try next time

      //number of tracked satellites
      VeGPSR_n_GPSSatellites.setValue(GPS.satellites);

      //fix quality
      VeGPSR_e_GPSFixQuality.setValue(GPS.fixquality);

      //check for fix to update location data
      if (GPS.fix){

        Serial.println(GPS.longitude, 4);
        
        //latitude
        double LeGPSR_LatitudeRaw = GPS.latitude;
        double LeGPSR_LatitudeDegrees = floor(LeGPSR_LatitudeRaw/100.0);
        double LeGPSR_LatitudeMinutes =  LeGPSR_LatitudeRaw - (LeGPSR_LatitudeDegrees*100.0);
        double LeGPSR_LatitudeProcessed = LeGPSR_LatitudeDegrees + LeGPSR_LatitudeMinutes/60.0;
        if (GPS.lat == 'S'){LeGPSR_LatitudeProcessed *= -1;} //negate for south
        VeGPSR_deg_GPSLatitude.setValue(LeGPSR_LatitudeProcessed);

        //longitude
        double LeGPSR_LongitudeRaw = GPS.longitude;
        double LeGPSR_LongitudeDegrees = floor(LeGPSR_LongitudeRaw/100.0);
        double LeGPSR_LongitudeMinutes =  LeGPSR_LongitudeRaw - (LeGPSR_LongitudeDegrees*100.0);
        double LeGPSR_LongitudeProcessed = LeGPSR_LongitudeDegrees + LeGPSR_LongitudeMinutes/60.0;
        if (GPS.lon == 'W'){LeGPSR_LongitudeProcessed *= -1;} //negate for west
        VeGPSR_deg_GPSLongitude.setValue(LeGPSR_LongitudeProcessed);


        //altitude
        VeGPSR_m_GPSAltitude.setValue(GPS.altitude); //meters

        //Heading
        VeGPSR_deg_GPSHeading.setValue(GPS.angle); //degrees from north clockwise

        //ground speed
        VeGPSR_mps_GPSSpeed.setValue(GPS.speed/KNOTS_PER_MPS);
      }

    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);

  }
}


//Spawn threads
uint8_t Sensing_SetupTasks(void){

  adc_power_acquire(); //keep the adc on
  attachInterrupt(digitalPinToInterrupt(WSS_HALL_FRONTR), WSS_COUNT_ISR, FALLING);

  xTaskCreatePinnedToCore(
      BMSObserverTask
      ,  "Batt0 Obsrvr" 
      ,  2048        
      ,  &BMSRCAN0TaskParams
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  xTaskCreatePinnedToCore(
      BMSObserverTask
      ,  "Batt1 Obsrvr" 
      ,  2048        
      ,  &BMSRCAN1TaskParams
      ,  8  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  Wire.begin(); //start I2C on default pins
  Serial.println("I2C Controller Configured");

  //default 2g and 250 deg/sec?: yes "namely +/- 2g and +/- 250 degrees/sec"
  accelgyro.initialize();
  if (!accelgyro.testConnection()){
    return(SENSING_INIT_FAILURE);
  }
  accelgyro.CalibrateAccel(3);
  accelgyro.CalibrateGyro(2);
  Serial.println("MCU6050 Initialized and Calibrated");

  xTaskCreatePinnedToCore(
      MCU6050Task
      ,  "IMU MCU6050" 
      ,  2048        
      ,  NULL
      ,  7  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

   //GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //setup minimum data plus fix data
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);   // calculate fix a 5hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate over uart
  GPS.sendCommand(PMTK_ENABLE_WAAS);   // 1enable DGPS in america
  Serial.println("GPS Configured");

  xTaskCreatePinnedToCore(
      GPSTask
      ,  "GPS Task" 
      ,  2048     
      ,  NULL
      ,  7  // Priority
      ,  NULL // Task handle
      ,  tskNO_AFFINITY // run on whatever core
      );

  return(SENSING_INIT_SUCCESS);
  
}