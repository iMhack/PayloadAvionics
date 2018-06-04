//Almost working DMA multichannel

// Converted to only 1 buffer per ADC, reduced to a small example.
//Based on Example by SaileNav


#include "DMAChannel.h"
#include "ADC.h"
// and IntervalTimer
#include <IntervalTimer.h>
#include <SD.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

#define BUF_SIZE 8 //256 for 2 adc, 512 for one (max), 8 for 8 channel 1 ADC and a larger ADC
#define GLOBAL_BUF 8192 //32768 //size of the buffer that empties the adc buffer through an interrupt function

ADC *adc = new ADC(); // adc object

/*SD CARD CHIPSELECT*/

const int chipSelect = BUILTIN_SDCARD;

IntervalTimer timer0;
IntervalTimer timer1;
int startTimerValue0 = 0;
int startTimerValue1 = 0;

DMAChannel* dma0 = new DMAChannel(false);
DMAChannel* dma1 = new DMAChannel(false);

//ChannelsCfg order must be {CH1, CH2, CH3, CH0 }, adcbuffer output will be CH0, CH1, CH2, CH3
//Order must be {Second . . . . . . . . First} no matter the number of channels used.
const uint16_t ChannelsCfg_0 [] =  { 0x4E, 0x48, 0x49, 0x46, 0x47, 0x4F, 0x44, 0x45 };  //ADC0: CH0 ad6(A6), CH1 ad7(A7), CH2 ad15(A8), CH3 ad4(A9) works

/*ADC0: A0=0x45 A1=0x4E A2=0x48 A3=0x49 A6=0x46 A7=0x47A8=0x4F A9=0x44*/

const int ledPin = 13;

const int flightLength = 15; //length of the flight in min

const int period0 = 100; // us
unsigned long filePeriod = 30000; //ms (compared to an elapsedMillis)
unsigned long criticTime = 50; //to test

const double nbFiles=floor((flightLength*60*1000)/filePeriod);

//Initialize the buffers
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcbuffer[BUF_SIZE];
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalbuffer[GLOBAL_BUF];
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) copybuffer[GLOBAL_BUF];
unsigned long timebuffer[GLOBAL_BUF/BUF_SIZE];
const uint16_t initBuff = 5000; //value used to initialize the buffer/as stop value
const int thresholdMuon = 1800; //threshold over which the signal is considered to be the trace of a muon
volatile int pos=0; //current position within the global buffers
volatile bool BUFF=false; //choose which buffer should be filled, which one should be emptied (false: fill 1, empty 2 and vv)
int setupIdx[]={48,48};
int fileIdx[]={48,48};

elapsedMillis debounce;
elapsedMillis timeLog;
elapsedMillis timecheck;
elapsedMillis sinceFile; //allows the periodic creation of a new file
elapsedMicros callbacktime;

//void dma2_isr(void);
void dma0_isr(void);
void d2_isr(void);
void setup_adc() ;
void setup_dma();
void setup_files();
void increment_str(int *idx, char* s);
void increment_idx(int *idx);
void callback(void);
void fileManager(void);

char str[10];

/*
str[9]=c;
str[10]=c;
str[11]=c;*/

void setup() {
  // initialize the digital pin as an output.

  str[0]='M';
  str[1]='0';
  str[2]='0';
  str[3]='0';
  str[4]='0';
  str[5]='0';
  str[6]='.';
  str[7]='t';
  str[8]='x';
  str[9]='t';

  pinMode(ledPin, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);

  //attachInterrupt(2, d2_isr, FALLING);

  delay(5000);

  Serial.begin(9600);

  Serial.println("SD card config");
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
  // don't do anything more:
  // return; //this line makes it directly jumps to loop - use the boolean setupFail instead
  } Serial.println("Card initialized.");

  // clear buffers (adc output should never go higher than 4095 in 12 bits config)
  for (int i = 0; i < BUF_SIZE; ++i){
      adcbuffer[i]=initBuff;
  }

  delay(100);

  setup_files(); //setup the directories and files that have to be created

  delay(100);

  for (int i = 0; i < GLOBAL_BUF; ++i){
      globalbuffer[i]=initBuff;
  }

  for (int i = 0; i < GLOBAL_BUF/BUF_SIZE; ++i){
      timebuffer[i]=0;
  }

  setup_adc();
  setup_dma();

  callbacktime=0; //reset the time before initiating the callbacks
  startTimerValue0 = timer0.begin(callback, period0);
  //startTimerValue1 = timer1.begin(fileManager, 60000);
  //delay(1000);
  Serial.println("Setup end !");
}



//uint16_t analog=0; //variable that will store locally the value of the buffer

void loop() {

  while (timecheck<20); //pseudo-synchronisation that doesn't require an interrupt/callback

  Serial.println(timecheck);

  timecheck=0; //check the duration of the loop

  File testFile = SD.open(str, FILE_WRITE);

  String muonString = "";

  if (testFile) {
    std::copy(std::begin(globalbuffer), std::end(globalbuffer), std::begin(copybuffer));
    pos=0; //initialize back the position
    testFile.println(timeLog);
    for (int k=0;k<GLOBAL_BUF;k=k+BUF_SIZE) { //go through the buffer
      if (copybuffer[k]!=initBuff) { //stop when you reach the last written line
        for (int l=0;l<BUF_SIZE;l=l+1) { //go through each pin
          if (copybuffer[k+l]>thresholdMuon) { //only write if the buffer respects the threshold
            muonString = "A"; muonString += String(l); muonString += "  "; muonString += String(timebuffer[k/BUF_SIZE]); muonString += "  "; muonString += String(copybuffer[k+l]);muonString += "  ";
            testFile.println(muonString);
          }
        }
        globalbuffer[k]=initBuff;
      }
    }
  }
    // print to the serial port too:
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening ");
    Serial.println(str);
  }
  testFile.close();

  if (sinceFile >= filePeriod) { //create a new file periodically to limit the weight on the ram
    sinceFile = sinceFile - filePeriod; //decrement and adjust for latency
    increment_str(&(fileIdx[0]),&(str[4])); //change the file name
  }

/*
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(100);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(100);
  */
  //if (timecheck>criticTime) Serial.print("tu pues mec !");

}

void setup_dma() {
  dma0->begin(true);              // allocate the DMA channel
  dma0->TCD->SADDR = &ADC0_RA;    // where to read from
  dma0->TCD->SOFF = 0;            // source increment each transfer
  dma0->TCD->ATTR = 0x101;
  dma0->TCD->NBYTES = 2;     // bytes per transfer
  dma0->TCD->SLAST = 0;
  dma0->TCD->DADDR = &adcbuffer[0];// where to write to
  dma0->TCD->DOFF = 2;
  dma0->TCD->DLASTSGA = -2*BUF_SIZE;
  dma0->TCD->BITER = BUF_SIZE;
  dma0->TCD->CITER = BUF_SIZE;
  dma0->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0->disableOnCompletion();    // require restart in code
  dma0->interruptAtCompletion();
  dma0->attachInterrupt(dma0_isr);

  dma1->begin(true);              // allocate the DMA channel
  dma1->TCD->SADDR = &ChannelsCfg_0[0];
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

void setup_files() {

  for (int seti=0;seti<100;seti=seti+1) {
    if (SD.exists(str)) {
        increment_str(&(setupIdx[0]), &(str[1]));
        Serial.println(seti);
    }
  }

  for (int setj=0;setj<nbFiles;setj=setj+1) {
    File setupFile = SD.open(str, FILE_WRITE);
    increment_str(&(fileIdx[0]), &(str[4])); //change the file name
  }

  fileIdx[0]=48;
  fileIdx[1]=48;
  str[4]=fileIdx[1];
  str[5]=fileIdx[0];

}

void dma0_isr(void) {
    dma0->TCD->DADDR = &adcbuffer[0];
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

void increment_str(int *idx, char* s) {

  increment_idx(&(*(idx)));

  for (int k=0; k<2; k=k+1) {
    *(s+k)=*(idx+1-k);
  }

}

void callback(void) {

  if ((pos + BUF_SIZE - 1) < GLOBAL_BUF){
    timebuffer[pos/BUF_SIZE]=callbacktime;
    for (int i=0;i<BUF_SIZE;i++) {
      globalbuffer[pos+i]=adcbuffer[i];
    }
  }
  pos = pos + BUF_SIZE;
}

/*

void fileManager(void) {

  File testFile = SD.open(str, FILE_WRITE);

  String muonString = "";

  if (testFile) {
    std::copy(std::begin(globalbuffer), std::end(globalbuffer), std::begin(globalbuffer2));

    if (!BUFF) { //if BUFF==false then read buffer 1, write buffer 2
      BUFF = !BUFF;
      pos=0; //initialize back the position
      testFile.println(timeLog);
      for (int k=0;k<GLOBAL_BUF;k=k+BUF_SIZE) { //go through the buffer
        if (globalbuffer[k]!=initBuff) { //stop when you reach the last written line
          for (int l=0;l<BUF_SIZE;l=l+1) { //go through each pin
            if (globalbuffer[k+l]>thresholdMuon) { //only write if the buffer respects the threshold
              muonString = "A"; muonString += String(l); muonString += "  "; muonString += String(timebuffer1[k/BUF_SIZE]); muonString += "  "; muonString += String(globalbuffer[k+l]);muonString += "  ";
              testFile.println(muonString);
            }
          }
          globalbuffer[k]=initBuff;
        }
      }
    }
    else {
      BUFF = !BUFF;
      pos=0; //initialize back the position
      testFile.println(timeLog);
      for (int k=0;k<GLOBAL_BUF;k=k+BUF_SIZE) { //go through the buffer
        if (globalbuffer[k]!=initBuff) { //stop when you reach the last written line
          for (int l=0;l<BUF_SIZE;l=l+1) { //go through each pin
            if (globalbuffer2[k+l]>thresholdMuon) { //only write if the buffer respects the threshold
              muonString = "A"; muonString += String(l); muonString += "  "; muonString += String(timebuffer2[k/BUF_SIZE]); muonString += "  "; muonString += String(globalbuffer2[k+l]);muonString += "  ";
              testFile.println(muonString);
            }
          }
          globalbuffer2[k]=initBuff;
        }
      }
    }
  }
    // print to the serial port too:
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening ");
    Serial.println(str);
  }
  testFile.close();

  if (sinceFile >= filePeriod) { //create a new file periodically to limit the weight on the ram
    sinceFile = sinceFile - filePeriod; //decrement and adjust for latency
    increment_str(&(fileIdx[0]),&(str[4])); //change the file name
  }

  Serial.println(timecheck);

  timecheck=0; //check the duration of the loop
}*/
