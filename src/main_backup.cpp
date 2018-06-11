
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
#include <String>

//-----------------------------------------------------------------------------
//DEFINE
//-----------------------------------------------------------------------------

//conso de 180mA average when active, 110mA when idle
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

/* BME DEFINES */

Adafruit_BME280 bme;

/*SD CARD CHIPSELECT*/

const int chipSelect = BUILTIN_SDCARD;

/*LIFTOFF BOOL*/

bool liftoff=false;

/*SETUP FAILURE BOOL*/

bool setupFail=false; //boolean remains false unless a setup failure is detected

/* TEST VARIABLES */

elapsedMillis sinceAccTest; //time since the last acceleration average
elapsedMillis sincePrCalib; //time since the last pressure calibration

/* ACCELERATION TRIGGER */

unsigned long thresholdLength=500; //length in ms during which the acceleration should be averaged to detect the liftoff
unsigned int nbAcc=0; //number of accelration values stored in sumAcc during thresholdLength
float sumAcc=0.0; //initialize the variable that will store the sum of the accelerations during the previous thresholdLength
float testAcc = 0.0; //average of the accleration over thresholdLength
float thresholdAccG = 3; //threshold on testAcc in G
float thresholdAcc = thresholdAccG*9.81; //threshold on testAcc in m/s^2

/* PRESSURE TRIGGER */ //detects the overpressure occurring at ejection

float thresholdPr = 150000; //value in Pa for the threshold
unsigned long calibPeriod = 60000; //time between 2 updates of the pressure calibration
float currentPr = 0.0; //current pressure
float prevPr = 0.0; //previous pressure, used to ensure that the value actually used to measure pressure was on the launchpad
float liftoffPr = 0.0; //last pressure measured before liftoff, should be passed to the altimeter as bme.readAltitude(liftoffPr)


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
  while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer

  Blink_(LED, 50, 1);
  Serial.println("setup() START");
  /*********************/

  char dcdcSerial_command[8+0]={0x02, 'H', 'P', 'O', 0x03, 'E', 'C', 0x0D}; //for send command, 8 Bytes + Data lengt
  int incomingByte = 0;   // for incoming serial data
  Serial.print("Char bit size recorded in CHAR_BIT."); Serial.println(CHAR_BIT);
  Serial4.begin(38400,SERIAL_8E1);
  Serial.println("I send: ");
  for(int i = 0; i<8+0; i++){
    Serial4.write(dcdcSerial_command[i]);//dcdcSerial_command,8+0);
    Serial.print(dcdcSerial_command[i]);
  }Serial.println();
  Serial4.flush();
  delay(505);
  Blink_(LED, 50, 3);
  Serial.println("I received: ");
  while (Serial4.available()>0) {
//    for(int i = 0; i<3; i++){
      Serial.println(Serial4.read(), HEX);
//    }
        }
  delay(200);
  Serial.println("I received: ");
  while (Serial4.available()>0) {
    for(int i = 0; i<3; i++){
      Serial.println(Serial4.read(), HEX);
    }
        }
        delay(200);
        Serial.println("I send: ");
        for(int i = 0; i<8+0; i++){
          Serial4.write(dcdcSerial_command[i]);//dcdcSerial_command,8+0);
          Serial.print(dcdcSerial_command[i]);
        }Serial.println();
        delay(505);
        Blink_(LED, 50, 3);
        Serial.println("I received: ");
        while (Serial4.available()>0) {
//          for(int i = 0; i<3; i++){
            Serial.println(Serial4.read(), HEX);
//          }
              }
              delay(200);
              Serial.println("I send: ");
              for(int i = 0; i<8+0; i++){
                Serial4.write(dcdcSerial_command[i]);//dcdcSerial_command,8+0);
                Serial.print(dcdcSerial_command[i]);
              }Serial.println();
              delay(505);
              Blink_(LED, 50, 3);
              Serial.println("I received: ");
              while (Serial4.available()>0) {
//                for(int i = 0; i<3; i++){
                  Serial.println(Serial4.read(), HEX);
//                }
                    }
  delay(200);

  /**************/
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
  bno.setGRange(Adafruit_BNO055::ACC_CONFIG_8G); //sets the range of the accelerometer, can be 2G, 4G, 8G or 16G

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
  sincePrCalib = 0; //idem

  while(liftoff==false) {

    /*Calibration of the pressure sensor for the altimeter (every minute) */

    if (sincePrCalib >= calibPeriod) { //average the sum and transfer it to the test value every thresholdLength
      sincePrCalib = sincePrCalib - calibPeriod; //decrement and adjust for latency
      prevPr=currentPr;
      currentPr=bme.readPressure();
    }

    /*Trigger creation*/

    Serial.println("tempAcc : ");
    imu::Vector<3> accel=bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //total acceleration (gravity included)
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

    Serial.println("testAcc : ");
    Serial.println(testAcc);

    Serial.println("pressure : ");
    Serial.println(bme.readPressure());

    if(testAcc>thresholdAcc || bme.readPressure()>thresholdPr)//threshold on both the acceleration and pressure
    {
      liftoff=true;
      liftoffPr=prevPr; //calibrate the pressure for the altimeter
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
