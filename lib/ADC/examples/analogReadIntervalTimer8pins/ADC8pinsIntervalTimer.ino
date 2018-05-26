/*
    This example shows how to use the IntervalTimer library and the ADC library in the Teensy 3.0/3.1

    The three important objects are: the ADC, the (one or more) IntervalTimer and the same number of RingBuffer.

    The ADC sets up the internal ADC, you can change the settings as usual. Enable interrupts.
    The IntervalTimer (timerX) executes a function every 'period' microseconds.
    This function, timerX_callback, starts the desired ADC measurement, for example:
        startSingleRead, startSingleDifferential, startSynchronizedSingleRead, startSynchronizedSingleDifferential
    you can use the ADC0 or ADC1 (only for Teensy 3.1).

    When the measurement is done, the adc_isr is executed:
        - If you have more than one timer per ADC module you need to know which pin was measured.
        - Then you store/process the data
        - Finally, if you have more than one timer you can check whether the last measurement interruped a
          a previous one (using adc->adcX->adcWasInUse), and restart it if so.
          The settings of the interrupted measurement are stored in the adc->adcX->adc_config struct.


*/



#include "ADC.h"
// and IntervalTimer
#include <IntervalTimer.h>
#include <math.h>
#include <algorithm> //for std::fill_n
#include <vector>

const int ledPin = LED_BUILTIN;

/*const int readPin0 = A10;*/
const int emptyingPeriod = 100; // us

/*
const int readPin1 = A11;
const int period1 = 120; // us
*/
const int readPeriod = 10000; // us

ADC *adc = new ADC(); // adc object

IntervalTimer timer0; // timers

/* Replace the buffers by a 100entries table
RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
RingBuffer *buffer1 = new RingBuffer;
*/

#define PINS 8
uint8_t adc_pins[] = {A0,A1,A2,A3,A4,A7,A8,A9,A10};
int startTimerValue0 = 0;

bool buff = false; //set a global variable that says which buffer to use (false -> 0, true ->1)
int tIdx = 0; //set a global variable to give the current line to be written in the buffer

void callback();

void setup() {

    pinMode(ledPin, OUTPUT); // led blinks every loop

    pinMode(ledPin+1, OUTPUT); // timer0 starts a measurement
    pinMode(ledPin+2, OUTPUT); // timer1 starts a measurement
    pinMode(ledPin+3, OUTPUT); // adc0_isr, measurement finished for readPin0
    pinMode(ledPin+4, OUTPUT); // adc0_isr, measurement finished for readPin1

    for (int i=0;i<PINS;i++) { /*store the value in a buffer*/
      pinMode(adc_pins[i], INPUT);
    }

    Serial.begin(9600);

    delay(1000);

    ///// ADC0 ////
    // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT.
    //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->setAveraging(1); // set number of averages
    adc->setResolution(12); // set bits of resolution

    // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
    // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

    // always call the compare functions after changing the resolution!
    //adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
    //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V

    // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
    //adc->enableInterrupts(ADC_0);


    Serial.println("Starting Timers");

    // start the timers, if it's not possible, startTimerValuex will be false
    startTimerValue0 = timer0.begin(callback, emptyingPeriod);

    adc->enableInterrupts(ADC_0);

    if(startTimerValue0==false) {
            Serial.println("Timer0 setup failed");
    }

    Serial.println("Timers started");

    delay(500);
}

const int bufferSize = floor(1.1*emptyingPeriod/readPeriod); //define the size of the buffer according to the different periods of interest, with a 10% margin

int value = 0; //intermediary value for the reading (necessary ?)
int initValue = -1;
int buffer0[bufferSize][PINS]; //create 2 buffers, one to be filled and one to be emptied
int buffer1[bufferSize][PINS];
std::vector<int> second (bufferSize*PINS,initValue);
//std::fill_n(buffer0.begin(), bufferSize*PINS, initValue); //intialize the buffers at -1
//std::fill_n(buffer1.begin(), bufferSize*PINS, initValue);

void loop() {

    /* Open the file storing the content of the buffers and fill it */

    File muonFile = SD.open("muonData.txt", FILE_WRITE);

    if (muonFile) {
      if (buff) {
        //empty buff1 (buff0 writing ongoing) and fill it up with -1 until the newly accessed line is -1
        buff = !buff; //change the current buffer before starting to empty the previous one
        tIdx=0; //reset the time index for the bufferSize
      }
      else {
        //empty buff0 (buff1 writing ongoing) and fill it up with -1 until the newly accessed line is -1
        buff = !buff; //change the current buffer before starting to empty the previous one
        tIdx=0; //reset the time index for the bufferSize
      }
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datamuon.txt");
    }

    delayMicroseconds(readPeriod);
}

/* The callback */

void callback() {

  for (int i=0;i<PINS;i++) { /*store the value in a buffer*/
      value = adc->analogRead(adc_pins[i]); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
      Serial.print("A");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(value*3.3/adc->getMaxValue(ADC_0), 2);
      Serial.print(". ");/*
      if (buff) {
        buffer0[tIdx][i]=(value*3.3/adc->getMaxValue(ADC_0));
        //if it can be done at once buffer0[t][i]=(adc->analogRead(adc_pins[i])*3.3/adc->getMaxValue(ADC_0));
      }
      else {
        buffer1[tIdx][i]=(value*3.3/adc->getMaxValue(ADC_0));
        //if it can be done at once buffer0[t][i]=(adc->analogRead(adc_pins[i])*3.3/adc->getMaxValue(ADC_0));
      }*/
      //delayMicroseconds(25); //wait before starting another measurement
  }

}
