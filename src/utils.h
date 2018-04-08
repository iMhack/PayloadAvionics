#ifndef __UTILS_H__
#define __UTILS_H__

#include <Arduino.h>
#include <TeensyThreads.h>
#include <TinyGPS++.h>

void Blink_(int PIN, int DELAY_MS, int loops);
void smartDelay(unsigned long ms);
// static void printFloat(float val, bool valid, int len, int prec);
void printInt(unsigned long val, bool valid, int len);
void displayInfo(TinyGPSPlus &gps);
// static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
// static void printStr(const char *str, int len);

#endif