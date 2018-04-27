#include <TeensyThreads.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <utils.h>

#define LED 2

/* GPS DEFINES */
#define GPSSerial Serial1
#define GPSECHO true
//Adafruit_GPS GPS(&mySerial);
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
bool gps_sentence_decoded=false;

/* BNO DEFINES */ // Add High G as : https://forums.adafruit.com/viewtopic.php?f=19&t=120348
Adafruit_BNO055 bno;
/* RF DEFINES */
#define RFM95_CS 9
#define RFM95_INT 29
#define RFM95_RST 24
#define RFM95_FREQ 433.0
#define EN_RF 8
RH_RF95 rf95(RFM95_CS, RFM95_INT);
extern uint32_t datagramSeqNumber = 0;
uint8_t datas[SENSOR_PACKET_SIZE];
uint8_t dataGPS[24];

/* BME DEFINES */
Adafruit_BME280 bme;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(EN_RF, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(EN_RF, HIGH);
//  digitalWrite(EN_RF, HIGH);
  /*
  SPI.setSCK(13);
  SPI.setMISO(12);
  SPI.setMOSI(11);
  */
  Blink_(LED, 50, 2);
  Serial.begin(9600);
  //while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer
  Blink_(LED, 50, 1);
  Serial.println("setup() START");
  Serial1.begin(9600);
//  Serial1.println(PMTK_Q_RELEASE);
//  gps.sendCommand(PGCMD_ANTENNA);
  Serial.println("BNO config");
  if (not bno.begin())
    Serial.println("Failed to initialize BNO055! Is the sensor connected?");

  Serial.println("BME config");
  if (not bme.begin(&Wire1))
    Serial.print("Failed to initialize BME280! Is the sensor connected?");

  /*Start RF */
  Serial.println("RF config");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (not rf95.init())
    Serial.println("LoRa radio int failed");

  if (!rf95.setFrequency(RFM95_FREQ))
  {
    Serial.println("setFrequency failed");
  }
  else
  {
    Serial.print("Set Freq to: \t");
    Serial.println(RFM95_FREQ);
  }
  rf95.setTxPower(23, false);

  Serial.println("setup() END");
  Blink_(LED, 50, 3);
}

void loop()
{
  while (Serial1.available() > 0){
    char c = Serial1.read();
    if (gps.encode(c)){
      gps_sentence_decoded=true;
    }
  }
  if(gps_sentence_decoded){//Note for GS : same gps as your
    displayInfo(gps);//Print on USB Serial
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      //while(true);
    }
//*/
  BARO_data baro = (BARO_data){bme.readTemperature(), bme.readPressure(), bme.readPressure()};//False for last one
//  CreateTelemetryDatagram_GPS(gps.location.lat(),gps.location.lng(),gps.altitude.meters(),0,dataGPS);
  createTelemetryDatagram(bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER),bno.getVector(Adafruit_BNO055::VECTOR_EULER), baro, 0, datas);
  Serial.print("Sending ");
  for(int i = 0; i<SENSOR_PACKET_SIZE; i++){
    Serial.print(datas[i], HEX);
  }Serial.println();

  rf95.send(datas, SENSOR_PACKET_SIZE);
  Blink_(LED,25,1);

//*/
  delay(3000);

}
