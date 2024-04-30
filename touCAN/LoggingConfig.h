
//Generated 2024-04-30 16:29:27.932641 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 100
#define FLUSH_RATE 1
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_b_ControlReadyFlag;
  double LeSDLR_e_CANx_OpModeRequest;
  double LeSDLR_e_HVTargetState;
  double LeSDLR_a_IMU6AxRaw;
  double LeSDLR_a_IMU6AyRaw;
  double LeSDLR_a_IMU6AzRaw;
  double LeSDLR_w_IMU6WxRaw;
  double LeSDLR_w_IMU6WyRaw;
  double LeSDLR_w_IMU6WzRaw;
  double LeSDLR_a_IMU6AxFilt;
  double LeSDLR_a_IMU6AyFilt;
  double LeSDLR_a_IMU6AzFilt;
  double LeSDLR_w_IMU6WxFilt;
  double LeSDLR_w_IMU6WyFilt;
  double LeSDLR_w_IMU6WzFilt;
  double LeSDLR_deg_GPSLatitude;
  double LeSDLR_deg_GPSLongitude;
  double LeSDLR_m_GPSAltitude;
  double LeSDLR_deg_GPSHeading;
  double LeSDLR_mps_GPSSpeed;
  double LeSDLR_n_GPSSatellites;
  double LeSDLR_t_GPSMillisecondsUnix;
  double LeSDLR_e_GPSFixQuality;
  double LeSDLR_t_endTime;
}loggingMessage, dataToLog;

QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData ) );

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    WRAP_SPI_MUTEX(logfile.flush();, portMAX_DELAY)
    flushCounter = 0;
  }
}

inline bool logging_queue_data(void){
  loggingMessage.LeSDLR_t_currentTime = (double)esp_timer_get_time() / 1000000.0;
  loggingMessage.LeSDLR_b_ControlReadyFlag = VeCRLR_b_ControlReadyFlag.getValue();
  loggingMessage.LeSDLR_e_CANx_OpModeRequest = VeHVPR_e_CANx_OpModeRequest.getValue();
  loggingMessage.LeSDLR_e_HVTargetState = VeHVPR_e_HVTargetState.getValue();
  loggingMessage.LeSDLR_a_IMU6AxRaw = VeSNSR_a_IMU6AxRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AyRaw = VeSNSR_a_IMU6AyRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AzRaw = VeSNSR_a_IMU6AzRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WxRaw = VeSNSR_w_IMU6WxRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WyRaw = VeSNSR_w_IMU6WyRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WzRaw = VeSNSR_w_IMU6WzRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AzFilt = VeSNSR_a_IMU6AzFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WxFilt = VeSNSR_w_IMU6WxFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WyFilt = VeSNSR_w_IMU6WyFilt.getValue();
  loggingMessage.LeSDLR_w_IMU6WzFilt = VeSNSR_w_IMU6WzFilt.getValue();
  loggingMessage.LeSDLR_deg_GPSLatitude = VeGPSR_deg_GPSLatitude.getValue();
  loggingMessage.LeSDLR_deg_GPSLongitude = VeGPSR_deg_GPSLongitude.getValue();
  loggingMessage.LeSDLR_m_GPSAltitude = VeGPSR_m_GPSAltitude.getValue();
  loggingMessage.LeSDLR_deg_GPSHeading = VeGPSR_deg_GPSHeading.getValue();
  loggingMessage.LeSDLR_mps_GPSSpeed = VeGPSR_mps_GPSSpeed.getValue();
  loggingMessage.LeSDLR_n_GPSSatellites = VeGPSR_n_GPSSatellites.getValue();
  loggingMessage.LeSDLR_t_GPSMillisecondsUnix = VeGPSR_t_GPSMillisecondsUnix.getValue();
  loggingMessage.LeSDLR_e_GPSFixQuality = VeGPSR_e_GPSFixQuality.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  WRAP_SPI_MUTEX(logfile.print("LeSDLR_t_currentTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeCRLR_b_ControlReadyFlag");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_CANx_OpModeRequest");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeHVPR_e_HVTargetState");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AxRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AyRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AzRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WxRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WyRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WzRaw");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AxFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AyFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_a_IMU6AzFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WxFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WyFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeSNSR_w_IMU6WzFilt");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_deg_GPSLatitude");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_deg_GPSLongitude");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_m_GPSAltitude");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_deg_GPSHeading");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_mps_GPSSpeed");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_n_GPSSatellites");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_t_GPSMillisecondsUnix");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", VeGPSR_e_GPSFixQuality");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", LeSDLR_t_endTime");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.flush();,portMAX_DELAY)
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  WRAP_SPI_MUTEX(logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_b_ControlReadyFlag, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_CANx_OpModeRequest, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_HVTargetState, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WxRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WyRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WzRaw, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzFilt, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WxFilt, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WyFilt, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WzFilt, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSLatitude, 8);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSLongitude, 8);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_m_GPSAltitude, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSHeading, 2);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_mps_GPSSpeed, 2);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_n_GPSSatellites, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_GPSMillisecondsUnix, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_GPSFixQuality, 0);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);, portMAX_DELAY)
  WRAP_SPI_MUTEX(logfile.print("\n");,portMAX_DELAY)
}

#endif