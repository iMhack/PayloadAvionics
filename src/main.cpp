#include "DMAChannel.h"
#include "ADC.h"
// and IntervalTimer
#include <IntervalTimer.h>
#include <SD.h>
#include <sstream>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <utils.h>
#include <cmath>

using namespace std;


/******************************************************************************/
/*************************** Flight parameters ********************************/

const int flightLength = 15; //length of the flight in min (for the registration of data)
const int flightLengthMs = flightLength*60*1000; //length of the flight in ms


/******************************************************************************/
/*********************** Muon recording definitions ***************************/

#define BUF_SIZE 8 //256 for 2 adc, 512 for one (max), 8 for 8 channel 1 ADC and a larger ADC
#define GLOBAL_BUF 4096 //32768 //size of the buffer that empties the adc buffer through an interrupt function
#define BUZZER 2

ADC *adc = new ADC(); // adc object

/*SD CARD CHIPSELECT*/

const int chipSelect = BUILTIN_SDCARD;

/*Callback for buffer emptying*/

IntervalTimer timerMuon;
int startTimerValueMuon = 0;
const int periodMuon = 200; // time between the callbacks in us (sampling freq)

/*Opening DMA channels for the transfer towards the buffers*/

DMAChannel* dma0 = new DMAChannel(false);
DMAChannel* dma1 = new DMAChannel(false);

//ChannelsCfg order must be {CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH0 }, adcBuffer output will be CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7
//Order must be {Second . . . . . . . . First} no matter the number of channels used.
const uint16_t ChannelsCfgMuon [] =  { 0x4E, 0x48, 0x49, 0x46, 0x47, 0x4F, 0x44, 0x45 };  //ADC0: CH0 ad6(A6), CH1 ad7(A7), CH2 ad15(A8), CH3 ad4(A9) works

/*ADC0: A0=0x45 A1=0x4E A2=0x48 A3=0x49 A6=0x46 A7=0x47A8=0x4F A9=0x44*/

const int ledPin = 13;

//Parameters for file recording

unsigned long filePeriod = 60000; //ms time before a new file is created (compared to an elapsedMillis)
unsigned long pseudoLoopPeriod = 50; //ms

const double nbFiles=floor(flightLengthMs/filePeriod); //number of files that should be created for a file storing filePeriod of data

volatile int pos=0; //current position within the global buffers
int stopBuff=0; //value at which we should stop to read the buffer
int setupIdx[]={48,48}; //ascii code for 0
int fileIdx[]={48,48};
char muonFileName[10];
char dataFileName[10];

//RF transmission

int RF=0; //switch between telemetry and GPS datagrams
unsigned long rfPeriod = 400; //ms (compared to an elapsedMillis)
unsigned long emissionTrigger = 30000; //time after liftoff before emission, ms (compared to an elapsedMillis)

//Initialize the buffers
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcBuffer[BUF_SIZE]; //(muons) direct emptying of the 8 dma channels
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalBuffer[GLOBAL_BUF]; //(muons) storage of the adc output
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalBufferCopy[GLOBAL_BUF];  //(muons) exact copy of the buffer at every loop
unsigned long timeBuffer[GLOBAL_BUF/BUF_SIZE]; //(muons) buffer storing the time at which the data are being written
unsigned long timeBufferCopy[GLOBAL_BUF/BUF_SIZE]; //(muons) buffer storing the time at which the data are being written
const uint16_t initBuff = 5000; //value used to initialize the buffer/as stop value
int thresholdMuon[] = {4095,4095,4095,4095,4095,4095,4095,4095}; //threshold over which the signal is considered to be the trace of a muon
int nSTD=5; //nb of std dev used to create the threshold

//Timing functions

elapsedMillis debounce;
elapsedMillis timeLog;
elapsedMillis timecheck;
elapsedMillis sinceFile; //allows the periodic creation of a new file
elapsedMillis sinceRF; //periodic RF transmission
elapsedMicros callbacktime; //reference (in us) for the writing of the (muon) datalog


/******************************************************************************/
/*********************** Data recording definitions ***************************/

/* GPS DEFINES */

#define GPSSerial Serial1
#define GPSECHO true
//Adafruit_GPS GPS(&mySerial);
static const uint32_t GPSBaud = 9600;
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
float liftoffPr = 98000; //last pressure measured before liftoff, should be passed to the altimeter as bme.readAltitude(liftoffPr)

/* ALTITUDE TRIGGER */

float thresholdAlt=1500; //the acquisition will be triggered anyway if we reach an altitude larger than this trigger

void dma0_isr(void);
void setup_buffers();
void setup_sd();
void setup_bno();
void setup_bme();
void setup_rf();
void setup_dma();
void setup_adc();
void setup_muonFileName();
void setup_files();
void increment_idx(int *idx);
void update_str(int *idx, char* s);
void thresholdMuonCalc(uint16_t *tab, int *threshold);
int myPow(int x, int p);
void callbackMuon(void);


/******************************************************************************/
/***************************** Start the setup ********************************/

void setup() {
  // initialize the digital pin as an output.

  pinMode(ledPin, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  delay(5000);

  Blink_(BUZZER,50,2);

  Serial.begin(9600);

  setup_buffers();
  setup_sd();
  setup_bno();
  setup_bme();
  Serial1.begin(GPSBaud);
  setup_rf();

  // clear buffers (adc output should never go higher than 4095 in 12 bits config)

  delay(100);

  setup_files(); //setup the directories and files that have to be created

  delay(100);

  setup_adc();
  setup_dma();

  while(setupFail==true)  //buzzer being extremely annoying when the setup has failed
  {
    Blink_(BUZZER, 50, 1);
    delay(100);
  }

  //-----------------------------------------------------------------------------
  //ACCELERATION TRIGGERS !
  //-----------------------------------------------------------------------------

/*
  sinceAccTest = 0; //set it back to 0 before actually starting to measure
  sincePrCalib = 0; //idem

  while(liftoff==false) {

    //Calibration of the pressure sensor for the altimeter (every minute)

    if (sincePrCalib >= calibPeriod) { //average the sum and transfer it to the test value every thresholdLength
      sincePrCalib = sincePrCalib - calibPeriod; //decrement and adjust for latency
      //sincePrCalib = 0; //decrement and adjust for latency
      prevPr=currentPr;
      currentPr=bme.readPressure();
    }

    //Trigger creation

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

    if(testAcc>thresholdAcc || bme.readAltitude(prevPr/100)>thresholdAlt)//threshold on both the acceleration and altitude
    {
      liftoff=true;
      liftoffPr=prevPr; //calibrate the pressure for the altimeter
      Serial.println("liftoff!");
    }
  }
*/
  timeLog=0;
  callbacktime=0; //reset the time before initiating the callbacks
  startTimerValueMuon = timerMuon.begin(callbackMuon, periodMuon); //timer on global buffering
  Serial.println("Setup end !");
}

/******************************************************************************/
/***************************** Start the loop *********************************/

void loop() {

  /*****************************************************************************/
  /**********************************MUONLOG************************************/
Serial.println("new loop");

  while (timecheck<pseudoLoopPeriod); //pseudo-synchronisation that doesn't require an interrupt/callback

  //Serial.println("new loop");

  //Serial.println(timecheck);

  timecheck=0; //check the duration of the loop

  File muonFile = SD.open(muonFileName, FILE_WRITE);

  String muonString = "";

  if (muonFile) {
    std::copy(std::begin(globalBuffer), std::end(globalBuffer), std::begin(globalBufferCopy));
    std::copy(std::begin(timeBuffer), std::end(timeBuffer), std::begin(timeBufferCopy));
    pos=0; //initialize back the position

    for (int k=0;k<GLOBAL_BUF-BUF_SIZE+1;k=k+BUF_SIZE) { //go through the buffer
      if (globalBufferCopy[k]!=initBuff) {
        for (int l=0;l<BUF_SIZE;l=l+1) { //go through each pin
          if (globalBufferCopy[k+l]>thresholdMuon[l]) { //only write if the buffer respects the threshold
            muonString = "A"; muonString += String(l); muonString += "  "; muonString += String(timeBufferCopy[k/BUF_SIZE]); muonString += "  "; muonString += String(globalBufferCopy[k+l]);muonString += "  ";
            //muonFile.println(muonString);
          }
        }
        if (k>1000) globalBuffer[k]=initBuff; //give some time to the SD writing to catch up so it doesn't overwrite new data
      }
      else {
        stopBuff=k+BUF_SIZE; //stop buffer for the median calculation
        break; //break the loop if the stop (init) value has been reached

      }
    }
  }
  else {
    Serial.println("error opening ");
    Serial.println(muonFileName);
  }
  muonFile.close();

  Serial.println("timeLogs:");
  Serial.println(timeLog);
  thresholdMuonCalc(&globalBufferCopy[0], &thresholdMuon[0]);
  Serial.println(timeLog);

/******************************************************************************/
/**********************************DATALOG*************************************/

  //GPS data first

  /*this might be needed*/ /*
  while (Serial1.available() > 0){
    char c = Serial1.read();
    if (gps.encode(c)){
      gps_sentence_decoded=true;
    }
  }
  if(gps_sentence_decoded){//Note for GS : same gps as your
    displayInfo(gps);//Print on USB Serial
  }*/

  //Keep commented until new version
  /*while (Serial1.available() > 0){
    char c = Serial1.read();
    if (gps.encode(c)){
      gps_sentence_decoded=true;
    }
  }
  if(gps_sentence_decoded){//Note for GS : same gps as your
    displayInfo(gps);//Print on USB Serial
  }
  double lat=gps.location.lat();
  double lng=gps.location.lng();
  double alt=gps.altitude.meters();
  Serial.println("gps");
  Serial.println(lat);
  Serial.println(lng);
  Serial.println(alt);*/

  //CreateTelemetryDatagram_GPS(lat,lng,hGPS,timeLog,dataGPS);

  //Altimeter data
  BARO_data baro = (BARO_data){bme.readTemperature(), bme.readPressure()/100., bme.readAltitude(liftoffPr/100)}; //good one

  //IMU data
  imu::Vector<3> accel=bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler=bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //Create the string

  String dataString = "";
  //dataString +=lat; dataString +=" "; dataString +=lng; dataString +=" "; dataString +=alt; dataString +=" "; //GPS data
  dataString +=accel[0]; dataString +=" ";  dataString +=accel[1]; dataString +=" ";  dataString +=accel[2]; dataString +=" ";  //write down the acceleration
  dataString +=euler[0]; dataString +=" ";  dataString +=euler[1]; dataString +=" ";  dataString +=euler[2]; dataString +=" ";  //angular position
  dataString +=baro.temperature; dataString +=" "; dataString +=baro.pressure; dataString +=" ";dataString +=baro.altitude; dataString +=" "; //barometer data

  //Open the file to write down the string

  File dataFile = SD.open(dataFileName, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
  }
  else {
    Serial.println("error opening ");
    Serial.println(muonFileName);
  }
  dataFile.close();


  //Periodically send the datagrams to the ground station
  if (timeLog> emissionTrigger)  { //start emitting 30s after liftoff
    if (sinceRF > rfPeriod) {
      sinceRF = sinceRF - rfPeriod; //decrement and adjust for latency
      if (RF==0) { //alternatively send GPS or telemetry
        while (Serial1.available() > 0){
          char c = Serial1.read();
          if (gps.encode(c))  {
            gps_sentence_decoded=true;
          }
        }
        if(gps_sentence_decoded){//Note for GS : same gps as your
          displayInfo(gps);//Print on USB Serial
        }
        double lat=gps.location.lat();
        double lng=gps.location.lng();
        double alt=gps.altitude.meters();
        CreateTelemetryDatagram_GPS(lat,lng,alt,timeLog,dataGPS);
        rf95.send(dataGPS, GPS_PACKET_SIZE);
        RF=1;
      }
      else {
        createTelemetryDatagram(accel/9.81, euler, baro, timeLog, datas);
        rf95.send(datas,sizeof(datas));
        RF=0;
      }
    }
  }

  //Periodically create new files
  if (sinceFile >= filePeriod) { //create a new file periodically to limit the weight on the ram
    sinceFile = sinceFile-filePeriod; //decrement and adjust for latency
    increment_idx(&(fileIdx[0]));
    update_str(&(fileIdx[0]),&(muonFileName[4])); //change the file name
    update_str(&(fileIdx[0]),&(dataFileName[4]));
  }


/****************************Flight termination********************************/
  while (timeLog > flightLengthMs) {
    Serial.println("finito !");
    Blink_(BUZZER,50,1);
    delay(200);/*
    while (Serial1.available() > 0){
      char c = Serial1.read();
      if (gps.encode(c))  {
        gps_sentence_decoded=true;
      }
    }
    if(gps_sentence_decoded){//Note for GS : same gps as your
      displayInfo(gps);//Print on USB Serial
    }
    double lat=gps.location.lat();
    double lng=gps.location.lng();
    double alt=gps.altitude.meters();
    CreateTelemetryDatagram_GPS(lat,lng,alt,timeLog,dataGPS);
    rf95.send(dataGPS, GPS_PACKET_SIZE);*/
    delay(200);
  }
}



/*******************************End of loop************************************/



/******************************************************************************/
/********************************Functions*************************************/

/*************************** Setup functions **********************************/

void setup_buffers()  { //setup the different buffers
  for (int i = 0; i < BUF_SIZE; ++i)  { //dma buffer
      adcBuffer[i]=initBuff;
  }
  for (int i = 0; i < GLOBAL_BUF; ++i)  { //global buffer to transfer adcBuffer
      globalBuffer[i]=initBuff;
  }
  for (int i = 0; i < GLOBAL_BUF/BUF_SIZE; ++i) { //corresponding buffer to record the time
      timeBuffer[i]=0;
  }
}

void setup_sd() {
  Serial.println("SD card config");
  if (!SD.begin(chipSelect))  {
    setupFail=true;
    Serial.println("Card failed, or not present");
  } Serial.println("Card initialized.");
}

void setup_bno() {
  Serial.println("BNO config");
  if (not bno.begin()) {
    setupFail=true;
    Serial.println("Failed to initialize BNO055! Is the sensor connected?");
  }
  bno.setGRange(Adafruit_BNO055::ACC_CONFIG_8G); //sets the range of the accelerometer, can be 2G, 4G, 8G or 16G
}

void setup_bme()  {
  Serial.println("BME config");
  if (not bme.begin(&Wire1))  {
    setupFail=true;
    Serial.println("Failed to initialize BME280! Is the sensor connected?");
  }
}

void setup_rf() {
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
}

void setup_dma() {
  dma0->begin(true);              // allocate the DMA channel
  dma0->TCD->SADDR = &ADC0_RA;    // where to read from
  dma0->TCD->SOFF = 0;            // source increment each transfer
  dma0->TCD->ATTR = 0x101;
  dma0->TCD->NBYTES = 2;     // bytes per transfer
  dma0->TCD->SLAST = 0;
  dma0->TCD->DADDR = &adcBuffer[0];// where to write to
  dma0->TCD->DOFF = 2;
  dma0->TCD->DLASTSGA = -2*BUF_SIZE;
  dma0->TCD->BITER = BUF_SIZE;
  dma0->TCD->CITER = BUF_SIZE;
  dma0->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0->disableOnCompletion();    // require restart in code
  dma0->interruptAtCompletion();
  dma0->attachInterrupt(dma0_isr);

  dma1->begin(true);              // allocate the DMA channel
  dma1->TCD->SADDR = &ChannelsCfgMuon[0];
  dma1->TCD->SOFF = 2;            // source increment each transfer (n bytes)
  dma1->TCD->ATTR = 0x101;
  dma1->TCD->SLAST = -16;          // num ADC0 samples * 2
  dma1->TCD->BITER = 8;           // num of ADC0 samples
  dma1->TCD->CITER = 8;           // num of ADC0 samples
  dma1->TCD->DADDR = &ADC0_SC1A;
  dma1->TCD->DLASTSGA = 0;
  dma1->TCD->NBYTES = 2;
  dma1->TCD->DOFF = 0;
  dma1->triggerAtTransfersOf(*dma0);
  dma1->triggerAtCompletionOf(*dma0);

  dma0->enable();
  dma1->enable();

}

void setup_adc() {
  //ADC0
  adc->setAveraging(4, ADC_0); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_0); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_0); // change the sampling speed

  ADC1_CFG2 |= ADC_CFG2_MUXSEL;

  adc->adc0->enableDMA(); //ADC0_SC2 |= ADC_SC2_DMAEN;  // using software trigger, ie writing to ADC0_SC1A
  adc->adc1->enableDMA();

}

void setup_fileName(char* s, char ch){
  *s=ch;
  *(s+1)='0';
  *(s+2)='0';
  *(s+3)='0';
  *(s+4)='0';
  *(s+5)='0';
  *(s+6)='.';
  *(s+7)='t';
  *(s+8)='x';
  *(s+9)='t';
}

void setup_files() {

  setup_fileName(&(muonFileName[0]),'M'); //'M' for muons
  setup_fileName(&(dataFileName[0]),'D'); //'D' for data

  for (int seti=0;seti<100;seti=seti+1) { //check if the file already exists, if so increament
    if (SD.exists(muonFileName)) {
        increment_idx(&(setupIdx[0]));
        update_str(&(setupIdx[0]), &(muonFileName[1]));
        update_str(&(setupIdx[0]), &(dataFileName[1]));
        Serial.println(seti);
    }
  }

  for (int setj=0;setj<nbFiles;setj=setj+1) { //create all the files already
    File setupFile = SD.open(muonFileName, FILE_WRITE);
    setupFile.close();
    setupFile = SD.open(dataFileName, FILE_WRITE);
    setupFile.close();
    increment_idx(&(fileIdx[0]));
    update_str(&(fileIdx[0]), &(muonFileName[4])); //change the file name
    update_str(&(fileIdx[0]), &(dataFileName[4])); //change the file name
  }

  fileIdx[0]=48; //reset the id code for future openings
  fileIdx[1]=48;
  muonFileName[4]=fileIdx[1]; dataFileName[4]=fileIdx[1];
  muonFileName[5]=fileIdx[0]; dataFileName[5]=fileIdx[0];

}

void dma0_isr(void) {
    dma0->TCD->DADDR = &adcBuffer[0];
    dma0->clearInterrupt();
    dma0->enable();
    digitalWriteFast(4, HIGH);
    digitalWriteFast(4, LOW);
}

void increment_idx(int *idx) {
  *(idx)=*(idx)+1;
  if (*(idx)>57) {
    *(idx)=48;
    *(idx+1)=*(idx+1)+1;
  }
}

void update_str(int *idx, char* s) {
  for (int k=0; k<2; k=k+1) {
    *(s+k)=*(idx+1-k);
  }
}

void thresholdMuonCalc(uint16_t *tab, int *threshold) {
  //calculate std device
  uint32_t sum[] = {0,0,0,0,0,0,0,0}, mean[] = {0,0,0,0,0,0,0,0};
  float standardDeviation[] = {0,0,0,0,0,0,0,0};
  int count=0;

  int tMCk,tMCl;
  for (tMCk=0;tMCk<GLOBAL_BUF-BUF_SIZE+1;tMCk+=BUF_SIZE) { //go through the buffer
    for (tMCl=0;tMCl<BUF_SIZE;tMCl+=1)  {
      if ((*(tab+tMCk+tMCl))!=initBuff) {
        sum[tMCl] += *(tab+tMCk+tMCl);
        count+=1;
      }
    }
  }

  count=floor(count/BUF_SIZE);

  for (tMCl=0;tMCl<BUF_SIZE;tMCl+=1)  {
    mean[tMCl]=sum[tMCl]/count;
  }

  for (tMCk=0;tMCk<GLOBAL_BUF-BUF_SIZE+1;tMCk+=BUF_SIZE) { //go through the buffer
    for (tMCl=0;tMCl<BUF_SIZE;tMCl+=1)  {
      if (*(tab+tMCk+tMCl)!=initBuff) {
        standardDeviation[tMCl] += myPow((*(tab+tMCk+tMCl) - mean[tMCl]), 2);
      }
    }
  }

  if ((count-1) != 0) {
    for (tMCl=0;tMCl<BUF_SIZE;tMCl+=1)  {
      standardDeviation[tMCl] = floor(sqrt(standardDeviation[tMCl]/(count-1)));
    }
  }

  for (tMCl=0;tMCl<BUF_SIZE;tMCl+=1)  {
    *(threshold+tMCl)=mean[tMCl]+nSTD*standardDeviation[tMCl];
  }
}


int myPow(int x, int p)
{
  if (p == 0) return 1;
  if (p == 1) return x;

  int tmp = myPow(x, p/2);
  if (p%2 == 0) return tmp * tmp;
  else return x * tmp * tmp;
}

void callbackMuon(void) {

  if ((pos + BUF_SIZE-1) < GLOBAL_BUF){
    timeBuffer[pos/BUF_SIZE]=callbacktime;
    for (int cbMi=0;cbMi<BUF_SIZE;cbMi++) {
      globalBuffer[pos+cbMi]=adcBuffer[cbMi];
      //Serial.println(adcBuffer[cbMi]);
    }
    pos = pos + BUF_SIZE;
  }
}
