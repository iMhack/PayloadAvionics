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
  while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer
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
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  displayInfo(bno);

  float pres = bme.readPressure();
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  displayInfo(bme);

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
  char radiopacket[20] = "Hello World #      ";
  //itoa(datagramSeqNumber++, radiopacket + 13, 10);
  Serial.print("Sending ");
  Serial.println(radiopacket);
  radiopacket[19] = 0;
  Serial.println("Sending...");
  delay(10);
  if(rf95.waitPacketSent()){Serial.println("Error sending packet !");}//Is too too long !

  rf95.send((uint8_t *)radiopacket, 20);
  delay(10);
  Blink_(LED,25,1);

//*/
  delay(3000);

}
