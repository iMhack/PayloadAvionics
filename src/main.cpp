//Almost working DMA multichannel

// Converted to only 1 buffer per ADC, reduced to a small example.
//Based on Example by SaileNav


#include "DMAChannel.h"
#include "ADC.h"
// and IntervalTimer
#include <IntervalTimer.h>
#include <SD.h>

#define BUF_SIZE 8 //256 for 2 adc, 512 for one (max), 8 for 8 channel 1 ADC and a larger ADC
#define GLOBAL_BUF 4096 //32768 //size of the buffer that empties the adc buffer through an interrupt function

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
//const uint16_t ChannelsCfg_0 [] =  { 0x49, 0x4F, 0x46, 0x48 };  //ADC0: CH0 ad6(A6), CH1 ad7(A7), CH2 ad15(A8), CH3 ad4(A9)
//const uint16_t ChannelsCfg_1 [] =  { 0x4E, 0x19, 0x1A, 0x45 };  //ADC1: CH0 ad4(A17), CH1 ad5(A16), CH2ad6(A18), CH3 ad7(A19)
/*ADC0:
A0=0x45
A1=0x4E
A2=0x48
A3=0x49
A6=0x46
A7=0x47
A8=0x4F
A9=0x44
*/
const int ledPin = 13;

const int period0 = 1; // us
const int periodFile = 10000000; // us, every 10 seconds a new file is opened

//Initialize the buffers
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcbuffer[BUF_SIZE];
static volatile uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalbuffer1[GLOBAL_BUF];
static volatile uint16_t __attribute__((aligned(GLOBAL_BUF+0))) globalbuffer2[GLOBAL_BUF];
const uint16_t initBuff = 50000;
volatile int pos=0; //current position within the global buffers
volatile bool BUFF=false; //choose which buffer should be filled, which one should be emptied (false: fill 1, empty 2 and vv)

volatile int d2_active;

elapsedMillis debounce;
elapsedMillis timecheck;
elapsedMicros callbacktime;

//void dma2_isr(void);
void dma0_isr(void);
void d2_isr(void);
void setup_adc() ;
void setup_dma();
void callback(void);
void fileManager(void);

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);

  d2_active = 0;

  pinMode(2, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);

  //attachInterrupt(2, d2_isr, FALLING);

  delay(5000);

  Serial.println("SD card config");
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
  // don't do anything more:
  // return; //this line makes it directly jumps to loop - use the boolean setupFail instead
  } Serial.println("Card initialized.");

  Serial.begin(9600);

  // clear buffers (adc output should never go higher than 4095 in 12 bits config)
  for (int i = 0; i < BUF_SIZE; ++i){
      adcbuffer[i]=initBuff;
  }

  for (int i = 0; i < GLOBAL_BUF; ++i){
      globalbuffer1[i]=initBuff;
      globalbuffer2[i]=initBuff;
  }

  setup_adc();
  setup_dma();
  startTimerValue0 = timer0.begin(callback, period0);
  startTimerValue1 = timer1.begin(fileManager, periodFile);
}

//uint16_t analog=0; //variable that will store locally the value of the buffer

void loop() {

  timecheck=0;
  /*
  File testFile = SD.open("testlog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (testFile) {
    if (BUFF==false) {
      BUFF=true; //switch to the second buffer before emptying the first
      pos=0; //initialize back the position
      for (int i=0;i<GLOBAL_BUF;i=i+8) {
        if (globalbuffer2[i]!=initBuff) {
          testFile.print(globalbuffer2[i]); testFile.print("  ");
          testFile.print(globalbuffer2[i+1]); testFile.print("  ");
          testFile.print(globalbuffer2[i+2]); testFile.print("  ");
          testFile.print(globalbuffer2[i+3]); testFile.print("  ");
          testFile.print(globalbuffer2[i+4]); testFile.print("  ");
          testFile.print(globalbuffer2[i+5]); testFile.print("  ");
          testFile.print(globalbuffer2[i+6]); testFile.print("  ");
          testFile.println(globalbuffer2[i+7]);
          if (timecheck>500) testFile.println("fuckup");
          globalbuffer2[i]=initBuff;
        }
      }
    }
    else {
      BUFF=false; //switch to the second buffer before emptying the first
      pos=0; //initialize back the position
      for (int i=0;i<GLOBAL_BUF;i=i+8) {
        if (globalbuffer1[i]!=initBuff) {
          testFile.print(globalbuffer2[i]); testFile.print("  ");
          testFile.print(globalbuffer2[i+1]); testFile.print("  ");
          testFile.print(globalbuffer2[i+2]); testFile.print("  ");
          testFile.print(globalbuffer2[i+3]); testFile.print("  ");
          testFile.print(globalbuffer2[i+4]); testFile.print("  ");
          testFile.print(globalbuffer2[i+5]); testFile.print("  ");
          testFile.print(globalbuffer2[i+6]); testFile.print("  ");
          testFile.println(globalbuffer2[i+7]);
          globalbuffer2[i]=initBuff;
        }
      }
    }
    // print to the serial port too:
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening testlog.txt");
  }
  testFile.close();*/
  //Serial.println(timecheck);

  /*
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(100);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(100);
  */
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

void d2_isr(void) {
  if(debounce > 200){
    d2_active = 1;
    debounce = 0;
    }
    else{return;}
}

void dma0_isr(void) {
    dma0->TCD->DADDR = &adcbuffer[0];
    dma0->clearInterrupt();
    dma0->enable();
    digitalWriteFast(4, HIGH);
    digitalWriteFast(4, LOW);
}

void callback(void) {/*
  if ((pos + BUF_SIZE - 1) < GLOBAL_BUF) {
    if (BUFF==false) {
      for (int i=0;i<BUF_SIZE;i++) {
        //globalbuffer1[pos+i]=adcbuffer[i];
        Serial.print(adcbuffer[i]);
      }
    }
    else {
      for (int i=0;i<BUF_SIZE;i++) {
        globalbuffer2[pos+i]=adcbuffer[i];
      }
    } pos = pos + BUF_SIZE;
  }*/

  Serial.print(adcbuffer[0]); Serial.print("  ");
  Serial.print(adcbuffer[1]); Serial.print("  ");
  Serial.print(adcbuffer[2]); Serial.print("  ");
  Serial.print(adcbuffer[3]); Serial.print("  ");
  Serial.print(adcbuffer[4]); Serial.print("  ");
  Serial.print(adcbuffer[5]); Serial.print("  ");
  Serial.print(adcbuffer[6]); Serial.print("  ");
  Serial.print(adcbuffer[7]); Serial.print("  ");
  Serial.println(callbacktime);
}

void fileManager(void) {

}
