//Almost working DMA multichannel

// Converted to only 1 buffer per ADC, reduced to a small example.
//Based on Example by SaileNav


#include "DMAChannel.h"
#include "ADC.h"
// and IntervalTimer
#include <IntervalTimer.h>
#include <SD.h>
//#include <cstdio>
//#include <iostream>
//#include <fstream>
//#include <string>
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

//ChannelsCfg order must be {CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH0 }, adcbuffer output will be CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7
//Order must be {Second . . . . . . . . First} no matter the number of channels used.
const uint16_t ChannelsCfg_0 [] =  { 0x4E, 0x48, 0x49, 0x46, 0x47, 0x4F, 0x44, 0x45 };  //ADC0: CH0 ad6(A6), CH1 ad7(A7), CH2 ad15(A8), CH3 ad4(A9) works

/*ADC0: A0=0x45 A1=0x4E A2=0x48 A3=0x49 A6=0x46 A7=0x47A8=0x4F A9=0x44*/

const int ledPin = 13;

const int flightLength = 15; //length of the flight in min

const int period0 = 100; // us
unsigned long filePeriod = 30000; //ms (compared to an elapsedMillis)
unsigned long criticTime = 50; //to test

const double nbFiles=floor((flightLength*60*1000)/filePeriod); //number of files that should be created for a file storing filePeriod of data

//Initialize the buffers
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcbuffer[BUF_SIZE]; //(muons) direct emptying of the 8 dma channels
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalbuffer[GLOBAL_BUF]; //(muons) storage of the adc output
static  uint16_t __attribute__((aligned(GLOBAL_BUF+0))) copybuffer[GLOBAL_BUF];  //(muons) exact copy of the buffer at every loop
unsigned long timebuffer[GLOBAL_BUF/BUF_SIZE]; //(muons) buffer storing the time at which the data are being written
const uint16_t initBuff = 5000; //value used to initialize the buffer/as stop value
const int thresholdMuon = 1800; //threshold over which the signal is considered to be the trace of a muon
volatile int pos=0; //current position within the global buffers
int setupIdx[]={48,48}; //ascii code for 0
int fileIdx[]={48,48};

elapsedMillis debounce;
elapsedMillis timeLog;
elapsedMillis timecheck;
elapsedMillis sinceFile; //allows the periodic creation of a new file
elapsedMicros callbacktime; //reference (in us) for the writing of the (muon) datalog

void dma0_isr(void);
void setup_adc() ;
void setup_dma();
void setup_muonFileName();
void setup_files();
void increment_idx(int *idx);
void update_str(int *idx, char* s);
void callback(void);

char muonFileName[10];
char dataFileName[10];

void setup() {
  // initialize the digital pin as an output.

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
  } Serial.println("Card initialized.");

  // clear buffers (adc output should never go higher than 4095 in 12 bits config)
  for (int i = 0; i < BUF_SIZE; ++i){
      adcbuffer[i]=initBuff;
  }

  delay(100);

  setup_files(); //setup the directories and files that have to be created

  delay(100);

  for (int i = 0; i < GLOBAL_BUF; ++i)  { // initialize the buffer
      globalbuffer[i]=initBuff;
  }

  for (int i = 0; i < GLOBAL_BUF/BUF_SIZE; ++i) {
      timebuffer[i]=0;
  }

  setup_adc();
  setup_dma();

  callbacktime=0; //reset the time before initiating the callbacks
  startTimerValue0 = timer0.begin(callback, period0); //timer on global buffering
  Serial.println("Setup end !");
}

void loop() {

  while (timecheck<20); //pseudo-synchronisation that doesn't require an interrupt/callback

  Serial.println(timecheck);

  timecheck=0; //check the duration of the loop

  File muonFile = SD.open(muonFileName, FILE_WRITE);

  String muonString = "";

  if (muonFile) {
    std::copy(std::begin(globalbuffer), std::end(globalbuffer), std::begin(copybuffer));
    pos=0; //initialize back the position
    muonFile.println(timeLog);
    for (int k=0;k<GLOBAL_BUF;k=k+BUF_SIZE) { //go through the buffer
      if (copybuffer[k]!=initBuff) { //stop when you reach the last written line
        for (int l=0;l<BUF_SIZE;l=l+1) { //go through each pin
          if (copybuffer[k+l]>thresholdMuon) { //only write if the buffer respects the threshold
            muonString = "A"; muonString += String(l); muonString += "  "; muonString += String(timebuffer[k/BUF_SIZE]); muonString += "  "; muonString += String(copybuffer[k+l]);muonString += "  ";
            muonFile.println(muonString);
          }
        }
        globalbuffer[k]=initBuff;
      }
    }
  }
  else {
    Serial.println("error opening ");
    Serial.println(muonFileName);
  }
  muonFile.close();

/******************************************************************************/
/**********************************DATALOG*************************************/

  File dataFile = SD.open(dataFileName, FILE_WRITE);

  String dataString = "";

  if (dataFile) {
    dataString +="Les muons furent découverts par Carl David Anderson et son assistant Seth Neddermeyer, au Caltech, en 1936, alors qu'ils travaillaient sur les rayons cosmiques. Ils remarquèrent des particules dont la trajectoire s'incurvait de manière distincte de celle des électrons et des autres particules connues, lorsqu'elles étaient soumises à un champ magnétique. Ces nouvelles particules portaient une charge électrique négative mais leur trajectoire était moins incurvée que celle des électrons mais plus incurvée que celle des protons à vitesse égale. On supposait que leur charge électrique négative était égale à celle de l'électron et qu'étant donné la différence de courbure de la trajectoire, on devait en déduire qu'elles avaient une masse intermédiaire à celle de l'électron et du proton.C'est pour cela qu'Anderson nomma d'abord cette particule mesotron, dont le préfixe meso- venant du grec signifie . Comme peu après d'autres particules de masses intermédiaires furent découvertes, le terme générique de meson fut adopté pour nommer de telles particules. Face au besoin de les différencier, le mesotron fut renommé mu meson (avec la lettre grecque μ (mu) utilisée pour ressembler au son de la lettre latine m).Cependant on découvrit bientôt que le mu meson différait de manière significative des autres mésons; par exemple ses produits de désintégration comprenaient un neutrino et un antineutrino, en lieu et place de l'un ou de l'autre, comme on l'observait pour les autres mésons, ceux-ci étant des hadrons, particules formées de quarks et donc sujettes à des interactions fortes. Dans le modèle de quark, un meson est composé d'exactement deux quarks (un quark et un anti-quark), à la différence des baryons qui sont composés de trois quarks. On découvrit, cependant, que les mu mesons étaient des particules fondamentales (leptons) comme les électrons, sans structure de quark. Ainsi les mu mesons n'étant pas du tout des mésons (au sens nouvellement défini du terme méson), le terme mu meson fut abandonné et remplacé par la nouvelle appellation de muon.";
    dataFile.println(dataString);
  }
  else {
    Serial.println("error opening ");
    Serial.println(muonFileName);
  }
  dataFile.close();

  if (sinceFile >= filePeriod) { //create a new file periodically to limit the weight on the ram
    sinceFile = sinceFile - filePeriod; //decrement and adjust for latency
    increment_idx(&(fileIdx[0]));
    update_str(&(fileIdx[0]),&(muonFileName[4])); //change the file name
    update_str(&(fileIdx[0]),&(dataFileName[4]));
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

void update_str(int *idx, char* s) {
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
