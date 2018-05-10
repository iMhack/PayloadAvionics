
#include <TeensyThreads.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <utils.h>
#include <cmath>

//-----------------------------------------------------------------------------
//DEFINE
//-----------------------------------------------------------------------------

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

/*DATAGRAM ARRAYS*/

extern uint32_t datagramSeqNumber = 0;
uint8_t datas[SENSOR_PACKET_SIZE];
uint8_t dataGPS[GPS_PACKET_SIZE];
elapsedMillis time;
elapsedMillis sinceAccTest; //time since the last acceleration average

/* BME DEFINES */

Adafruit_BME280 bme;

/*SD CARD CHIPSELECT*/

const int chipSelect = BUILTIN_SDCARD;

/*LIFTOFF BOOL*/

bool liftoff=false;

/*SETUP FAILURE BOOL*/

bool setupFail=false; //boolean remains false unless a setup failure is detected

/* ACCELERATION BUFFER */

unsigned long thresholdLength=500; //length in ms during which the acceleration should be averaged to detect the liftoff
unsigned int nbAcc=0; //number of accelration values stored in sumAcc during thresholdLength
float sumAcc=0.0; //initialize the variable that will store the sum of the accelerations during the previous thresholdLength
float testAcc = 0.0; //average of the accleration over thresholdLength
float thresholdAcc = 9.81; //threshold on testAcc in m/s^2 (not in Gs !!)

//-----------------------------------------------------------------------------
//SETUP()
//-----------------------------------------------------------------------------

void setup()
{

  delay(10000); //10 seconds delay before the setup actually starts allow to display the messages correctly

  pinMode(LED, OUTPUT);
  pinMode(EN_RF, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(EN_RF, HIGH);

  /*
  SPI.setSCK(13);
  SPI.setMISO(12);
  SPI.setMOSI(11);
  */

  Blink_(LED, 50, 2);
  Serial.begin(9600);
//  while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer
  Blink_(LED, 50, 1);
  Serial.println("setup() START");

  /*SD card setup()*/

  Serial.println("SD card config");
  if (!SD.begin(chipSelect))
  {
    setupFail=true;
    Serial.println("Card failed, or not present");
  // don't do anything more:
  // return; //this line makes it directly jumps to loop - use the boolean setupFail instead
  } Serial.println("Card initialized.");

  Serial.println("BNO config");
  if (not bno.begin()) {
    setupFail=true;
    Serial.println("Failed to initialize BNO055! Is the sensor connected?");
  }

  Serial.println("BME config");
  if (not bme.begin(&Wire1))  {
    setupFail=true;
    Serial.println("Failed to initialize BME280! Is the sensor connected?");
  }

  /*START RF */

  Serial.println("RF config");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (not rf95.init()) {
    setupFail=true;
    Serial.println("LoRa radio int failed");
  }

  if (!rf95.setFrequency(RFM95_FREQ))
  {
    setupFail=true;
    Serial.println("setFrequency failed");
  }
  else
  {
    Serial.print("Set Freq to: \t");
    Serial.println(RFM95_FREQ);
  }
  rf95.setTxPower(23, false);

  while(setupFail==true)  //buzzer being extremely annoying when the setup has failed
  {
    Blink_(LED, 50, 1);
    delay(100);
  }

  Serial.println("setup() END");

//-----------------------------------------------------------------------------
//ACCELERATION TRIGGERS !
//-----------------------------------------------------------------------------

//  while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer

  sinceAccTest = 0; //set it back to 0 before actually starting to measure

  while(liftoff==false)
    {
    Serial.println("tempAcc : ");
    imu::Vector<3> accel=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //linear acceleration removes the gravity
    float tempAcc= pow(accel[0],2)+pow(accel[1],2)+pow(accel[2],2); //summing the acceleration to create an agnostic accelerometer value
    tempAcc= sqrt(tempAcc);
    Serial.println(tempAcc);

    sumAcc = sumAcc + tempAcc; //sum the current acceleration with the previous ones
    nbAcc = nbAcc + 1; //increment the number of accelerations stored in sumAcc

    if (sinceAccTest >= thresholdLength) { //average the sum and transfer it to the test value every thresholdLength
      sinceAccTest = sinceAccTest - thresholdLength; //decrement and adjust for latency
      testAcc = sumAcc / nbAcc; //perform the average
      sumAcc = 0; //empty the buffer
      nbAcc=0; //reinitialize the counter
    }

    Serial.println(testAcc);

    if(testAcc>thresholdAcc)//acceleration trigger, put 5.0 if you want to trigger manually, else, put 40~50. Those are m/s^2.
    {
      liftoff=true;
      Serial.println("liftoff!");
    }

  }

  Blink_(LED, 50, 3);

}

//-----------------------------------------------------------------------------
//LOOP
//-----------------------------------------------------------------------------

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

//datastring for sdcard
String dataString = "";

  BARO_data baro = (BARO_data){bme.readTemperature(), bme.readPressure()/100., bme.readPressure()/100.};//False for last one
  CreateTelemetryDatagram_GPS(gps.location.lat(),gps.location.lng(),gps.altitude.meters(),time,dataGPS);
  createTelemetryDatagram(bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER)/9.81,bno.getVector(Adafruit_BNO055::VECTOR_EULER), baro, 0, datas);

  Serial.println("sending and writing : ");
  Serial.print("gps datagram : ");

  for(int i = 0; i<=GPS_PACKET_SIZE; i++){
        dataString += String(dataGPS[i]);
  }Serial.println();
  dataString += " | ";

  Serial.print("telemetry datagram : ");

  for(int i = 0; i<SENSOR_PACKET_SIZE; i++){
    dataString += String(datas[i]);
  }Serial.println();
dataString += " - ";

File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

 rf95.send(datas,sizeof(datas));
 delay(250);
 rf95.send(dataGPS, GPS_PACKET_SIZE);
 Serial.println("end of send telemetry");
 Blink_(LED,25,1);
 delay(250);
}
