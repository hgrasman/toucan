
//Generated 2024-05-29 10:28:13.579969 with logging_helper_2.py for EV Kartz Kettering University
//Henry Grasman

#ifndef LOGGING_CONFIG
#define LOGGING_CONFIG

#include "FS.h"

#define LOG_RATE 10
#define FLUSH_RATE 10
uint8_t flushCounter = 0;

struct loggingData{
  double LeSDLR_t_currentTime;
  double LeSDLR_a_IMU6AxRaw;
  double LeSDLR_a_IMU6AyRaw;
  double LeSDLR_a_IMU6AzRaw;
  double LeSDLR_w_IMU6WxRaw;
  double LeSDLR_w_IMU6WyRaw;
  double LeSDLR_w_IMU6WzRaw;
  double LeSDLR_a_IMU6AxFilt;
  double LeSDLR_a_IMU6AyFilt;
  double LeSDLR_a_IMU6AzFilt;
  double LeSDLR_cnt_WSSPulseCount;
  double LeSDLR_deg_GPSLatitude;
  double LeSDLR_deg_GPSLongitude;
  double LeSDLR_m_GPSAltitude;
  double LeSDLR_deg_GPSHeading;
  double LeSDLR_mps_GPSSpeed;
  double LeSDLR_n_GPSSatellites;
  double LeSDLR_e_GPSFixQuality;
  double LeSDLR_t_endTime;
}loggingMessage, dataToLog;

QueueHandle_t loggingQueue = xQueueCreate( 16, sizeof( struct loggingData ) );

inline void logging_flush_buffer(File logfile){
  if (flushCounter++ > FLUSH_RATE){
    logfile.flush();
    flushCounter = 0;
  }
}

inline bool logging_queue_data(void){
  loggingMessage.LeSDLR_t_currentTime = (double)esp_timer_get_time() / 1000000.0;
  loggingMessage.LeSDLR_a_IMU6AxRaw = VeSNSR_a_IMU6AxRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AyRaw = VeSNSR_a_IMU6AyRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AzRaw = VeSNSR_a_IMU6AzRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WxRaw = VeSNSR_w_IMU6WxRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WyRaw = VeSNSR_w_IMU6WyRaw.getValue();
  loggingMessage.LeSDLR_w_IMU6WzRaw = VeSNSR_w_IMU6WzRaw.getValue();
  loggingMessage.LeSDLR_a_IMU6AxFilt = VeSNSR_a_IMU6AxFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AyFilt = VeSNSR_a_IMU6AyFilt.getValue();
  loggingMessage.LeSDLR_a_IMU6AzFilt = VeSNSR_a_IMU6AzFilt.getValue();
  loggingMessage.LeSDLR_cnt_WSSPulseCount = VeWSSR_cnt_WSSPulseCount.getValue();
  loggingMessage.LeSDLR_deg_GPSLatitude = VeGPSR_deg_GPSLatitude.getValue();
  loggingMessage.LeSDLR_deg_GPSLongitude = VeGPSR_deg_GPSLongitude.getValue();
  loggingMessage.LeSDLR_m_GPSAltitude = VeGPSR_m_GPSAltitude.getValue();
  loggingMessage.LeSDLR_deg_GPSHeading = VeGPSR_deg_GPSHeading.getValue();
  loggingMessage.LeSDLR_mps_GPSSpeed = VeGPSR_mps_GPSSpeed.getValue();
  loggingMessage.LeSDLR_n_GPSSatellites = VeGPSR_n_GPSSatellites.getValue();
  loggingMessage.LeSDLR_e_GPSFixQuality = VeGPSR_e_GPSFixQuality.getValue();
  loggingMessage.LeSDLR_t_endTime = (double)esp_timer_get_time() / 1000000.0;

  return (xQueueSend( loggingQueue, ( void * ) &loggingMessage, portMAX_DELAY ) == pdTRUE);
}

inline void logging_write_header(File logfile){
  logfile.print("LeSDLR_t_currentTime");
  logfile.print(", VeSNSR_a_IMU6AxRaw");
  logfile.print(", VeSNSR_a_IMU6AyRaw");
  logfile.print(", VeSNSR_a_IMU6AzRaw");
  logfile.print(", VeSNSR_w_IMU6WxRaw");
  logfile.print(", VeSNSR_w_IMU6WyRaw");
  logfile.print(", VeSNSR_w_IMU6WzRaw");
  logfile.print(", VeSNSR_a_IMU6AxFilt");
  logfile.print(", VeSNSR_a_IMU6AyFilt");
  logfile.print(", VeSNSR_a_IMU6AzFilt");
  logfile.print(", VeWSSR_cnt_WSSPulseCount");
  logfile.print(", VeGPSR_deg_GPSLatitude");
  logfile.print(", VeGPSR_deg_GPSLongitude");
  logfile.print(", VeGPSR_m_GPSAltitude");
  logfile.print(", VeGPSR_deg_GPSHeading");
  logfile.print(", VeGPSR_mps_GPSSpeed");
  logfile.print(", VeGPSR_n_GPSSatellites");
  logfile.print(", VeGPSR_e_GPSFixQuality");
  logfile.print(", LeSDLR_t_endTime");
  logfile.print("\n");
  logfile.flush();
}

inline void logging_write_line(File logfile, struct loggingData *pdataToLog){
  logfile.print(pdataToLog->LeSDLR_t_currentTime, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WxRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WyRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_w_IMU6WzRaw, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AxFilt, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AyFilt, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_a_IMU6AzFilt, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_cnt_WSSPulseCount, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSLatitude, 10);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSLongitude, 10);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_m_GPSAltitude, 4);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_deg_GPSHeading, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_mps_GPSSpeed, 2);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_n_GPSSatellites, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_e_GPSFixQuality, 0);
  logfile.print(", "); logfile.print(pdataToLog->LeSDLR_t_endTime, 4);
  logfile.print("\n");
}

#endif