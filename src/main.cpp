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
#include <string>
#include <uart.h>

//-----------------------------------------------------------------------------
//DEFINE
//-----------------------------------------------------------------------------

//typedef bitset<8> BYTE;

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

  delay(2000); //10 seconds delay before the setup actually starts allow to display the messages correctly

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

  while(setupFail==true)  //buzzer being extremely annoying when the setup has failed
  {
    Blink_(LED, 50, 1);
    delay(100);
  }

  //Serial4.begin(38400,SERIAL_8E1);
  uart_init(38400);

  delay(1000);

  Serial.println("setup() END");

}

//-----------------------------------------------------------------------------
//LOOP
//-----------------------------------------------------------------------------

void loop()
{
  /*********************/

  unsigned char dcdcSerial_command[8]={0x02,'h','p','o',0x03,0x45,0x43,0x0d}; //for send command, 8 Bytes + Data lengt
  unsigned char incomingByte=0;   // for incoming serial data
  for (int i=0;i<8;i++) uart_putchar(dcdcSerial_command[i]);
  //Serial4.write(dcdcSerial_command);
  /*Serial4.write(0x02);
  Serial4.write('h');
  Serial4.write('p');
  Serial4.write('o');
  Serial4.write(0x03);
  Serial4.write(0x45);
  Serial4.write(0x43);
  Serial4.write(0x0d);*/
  //delay(50);
  //Serial.print("I send: ");Serial.println(dcdcSerial_command);
  delay(1000);
  //Serial4.flush();
  while (uart_available()) {
                // read the incoming byte:
                incomingByte = uart_getchar();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte,HEX);
                delay(100);
        }
  delay(3000);

  /**************/
}
