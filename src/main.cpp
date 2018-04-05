#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#include "main.h"
#include "GPS.h"

/* To communicate with the GPS we need to increase the receive buffer size! Therefore add:
#define SERIAL_RX_BUFFER_SIZE 128
to the "HardwareSerial.h" file.
*/
TinyGPSPlus gps;


const float rho = 1.225;  // air density at 15°C [kg/m^3]
constexpr unsigned long period = 13 * 1000; // period in microseconds
bool gps_sentence_decoded = false;
uint32_t sequenceNumber;

static uint8_t packetGroupIndex = 0;

Adafruit_BME280 bmp; // I2C
SoftwareSerial *telemSerial;
String DELIMITER;

// Initializations
void setup() {
    DELIMITER = F(",\t");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    //Xbee serial: RX = digital pin 5, TX = digital pin 6
    writePgmChartoSerial(GPS_fix_rate);
    writePgmChartoSerial(GPS_pos_fix_rate);


#if GY_91
    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

    if (!bmp.begin()) {
        serialLog(F("Could not find a valid BME280 sensor, check wiring! (or adress!!!)"));
    }
#endif
}

void loop() {
    // Read GPS data on HardwareSerial
    while(Serial.available()){
        char c = Serial.read();
        if (gps.encode(c)) gps_sentence_decoded = true;
    }

    static unsigned long measurement_time = 0;
    while (micros() - measurement_time < period) {
        delayMicroseconds(100);
    }
    measurement_time = static_cast<uint32_t>(micros());

    static uint8_t counter = 0;

    // array containing the data (acc, gyro, mag)
    measurements data{};

#if GY_91
    // ____________________________________
    // :::  accelerometer and gyroscope :::

    uint8_t bufIMU[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, bufIMU);


    data.ax = bufIMU[0] << 8 | bufIMU[1];   // ax
    data.ay = bufIMU[2] << 8 | bufIMU[3];  // ay
    data.az = bufIMU[4] << 8 | bufIMU[5];  // az
    data.gx = bufIMU[8] << 8 | bufIMU[9];  // gx
    data.gy = bufIMU[10] << 8 | bufIMU[11];// gy
    data.gz = bufIMU[12] << 8 | bufIMU[13];// gz

    // _____________________
    // :::  Magnetometer :::
    // Read register Status 1 and wait for the DRDY: Data Ready

    uint8_t ST1;
    do {
        I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
    } while (!(ST1 & 0x01));

    // Read magnetometer data
    uint8_t bufMag[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, bufMag);
    // request for the following mag measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

    data.mx = bufMag[3] << 8 | bufMag[2]; // mx
    data.my = bufMag[1] << 8 | bufMag[0]; // my
    data.mz = bufMag[5] << 8 | bufMag[4]; // mz

    // pressure from BME280
    uint32_t pressure = (uint32_t) bmp.readPressure();
    uint32_t temperature = (uint32_t) bmp.readTemperature();

#endif



/*
    String dataStringSD = measurement_time + DELIMITER
                          + data.ax + DELIMITER
                          + data.ay + DELIMITER
                          + data.az + DELIMITER
                          + data.gx + DELIMITER
                          + data.gy + DELIMITER
                          + data.gz + DELIMITER
                          + data.mx + DELIMITER
                          + data.my + DELIMITER
                          + data.mz + DELIMITER
                          + pressure.fl + DELIMITER
#if PITOT_TUBE
    + delta_p;
#else
    ;
#endif
*/

    // start the Finite State Machine
    static char state = READY;
    static long lastState = 0;

    switch (state) {
        case READY:
            if (abs(data.ay) > ACC_THRESHOLD) {
                state = MOTOR;
                lastState = millis();
            }
            break;

        case MOTOR:

            // timer ou fin de l'accélération
            if (abs(data.ay) < ACC_THRESHOLD) {
                if (millis() - lastState < 500) {
                    state = READY;
                    break;
                }
            }
            if (millis() - lastState > 2500) { // increase this delay
                lastState = millis();
                state = OPEN;
            }
            break;

        case OPEN:
            if (millis() - lastState > 800 && counter < 3) {
                counter++;
                state = CLOSE;
                lastState = millis();
            }
            else if(millis()-lastState > 1000){
                state = END;
            }
            break;
        case CLOSE:
            if (millis() - lastState > 800) {
                state = OPEN;
                lastState = millis();
            }
            break;
        case END:
            break;
    }
}

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}

void I2Cread_NoReg(uint8_t Address, uint8_t Nbytes, uint8_t *Data) {
    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}


uint8_t escapedCharacter(uint8_t byte) {
    switch (byte) {
        case 0x7e:
            return 0x53;
        case 0x7d:
            return 0x5d;
        case 0x11:
            return 0x31;
        case 0x13:
            return 0x33;
        default:
            return 0x00;
    }
}

// send a bip when the logging starts
void bip(int duration) {
    int period = 100;
    int count = duration * 5;
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(period);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(period);
    }
}

void serialLog(const String &str) {
#if VERBOSE
    Serial.println(str);
#endif
}

void writePgmChartoSerial(const char addr[]) {

    for (size_t k = 0; k < strlen_P(addr); k++)
  {
    char c =  pgm_read_byte_near(addr + k);
    Serial.write(c);
  }
    Serial.println();
}
