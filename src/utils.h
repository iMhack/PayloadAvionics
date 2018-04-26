#ifndef __UTILS_H__
#define __UTILS_H__


#include <Arduino.h>
#include <TeensyThreads.h>
#include <TinyGPS++.h>
#include <vector.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <telemetry_protocol.h>
#include <simpleCRC.h>
#include <string>
#include <sstream>
#include <stdint.h>

void Blink_(int PIN, int DELAY_MS, int loops);
void smartDelay(unsigned long ms);
// static void printFloat(float val, bool valid, int len, int prec);
void displayInfo(TinyGPSPlus &gps);
void displayInfo(Adafruit_BNO055 &bno);
void displayInfo(Adafruit_BME280 &bme);
typedef struct
{
  float temperature;
  float pressure;
  float altitude;
} BARO_data;

void* CreateTelemetryDatagram_GPS(float lat,float lng,float altitude,uint32_t measurement_time);
void* createTelemetryDatagram (imu::Vector<3> accel, imu::Vector<3> euler, BARO_data baro, uint32_t measurement_time, uint8_t *datas);

inline void write8 (uint8_t v, uint8_t* datas, int currentPos);

inline void write16 (uint16_t v, uint8_t* datas, int currentPos);

inline void write32u (uint32_t v, uint8_t* datas, int currentPos);

inline void write32f (float v, uint8_t* datas, int currentPos);
// static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
// static void printStr(const char *str, int len);
#endif
