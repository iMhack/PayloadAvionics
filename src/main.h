//
// Created by clement on 18/10/2017.
//

#ifndef __MAIN_H__
#define __MAIN_H__


#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

// pins
#define   BUZZER_PIN                         2
#define   BRAKES_PIN                         A1
#define   BUTTON_PIN                         3
//#define   ERROR_LED_PIN                       7
//#define   SD_LED_PIN                          8
#define   CHIP_SELECT_PIN                    10


struct measurements {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
};

enum State {
    READY,
    MOTOR,
    OPEN,
    CLOSE,
    END
};


void telem_write_uint32(uint32_t val);

void telem_write_uint16(uint16_t val);

void telem_write_uint8(uint8_t val);

uint8_t escapedCharacter(uint8_t byte);

void sendTelemetryByte(uint8_t byte);

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data);

void I2Cread_NoReg(uint8_t Address, uint8_t Nbytes ,uint8_t *Data);

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

void writePgmChartoSerial(const char addr[]);

void beginTelemetryDatagram(uint8_t payloadType);

void finalizeTelemetryDatagram();

void bip(int duration);

void serialLog(const String &str);

float velocityFromPitot(float delta_p);

// sensor adresses
#define   MPU9250_ADDRESS                     0x68
#define   MAG_ADDRESS                         0x0C
#define   DIFF_PRESS_HIGH_RANGE_SENSOR_ADDR   0x58

// sensor registers
#define   GYRO_FULL_SCALE_250_DPS             0x00
#define   GYRO_FULL_SCALE_500_DPS             0x08
#define   GYRO_FULL_SCALE_1000_DPS            0x10
#define   GYRO_FULL_SCALE_2000_DPS            0x18

#define   ACC_FULL_SCALE_2_G                  0x00
#define   ACC_FULL_SCALE_4_G                  0x08
#define   ACC_FULL_SCALE_8_G                  0x10
#define   ACC_FULL_SCALE_16_G                 0x18

// boolean
#define   VERBOSE                             true
#define   PITOT_TUBE                          true
#define   GY_91                               true

#define   TS                                  0x40
#define   AX                                  0x20
#define   AY                                  0x21
#define   AZ                                  0x22

#define   ACC_THRESHOLD                       8350 //=20*417.5 /2000

#define   PRESSURE_SENSOR_STATUS_NORMAL       0b00
#define   PRESSURE_SENSOR_STATUS_COMMAND      0b01
#define   PRESSURE_SENSOR_STATUS_STALE_DATA   0b10
#define   PRESSURE_SENSOR_STATUS_DIAGNOSTIC   0b11

/* SSCMRNN015PG5A3 0 - 15 psi*/
#define   PRESSURE_SENSOR2_MAX                103421.f    /* [Pa] */
#define   PRESSURE_SENSOR2_MIN                0.f         /* [Pa] */


#endif //DATA_LOGGER_PIO_DATA_LOGGER_H
