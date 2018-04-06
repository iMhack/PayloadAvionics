
#include <SPI.h>
#include <RF.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1
#define LED           2

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;

void setup()
{
  //setupRF();
  pinMode(LED, OUTPUT);
  Blink(LED,50,2);
  Serial.begin(9600);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("setup()");
  GPS.begin(9600);
  delay(1000);
  mySerial.begin(9600);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

}


void loop() {
  //loopRF();
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);

  if (Serial.available()) {
    char c = Serial1.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    Serial.println("GPSSerial.available()==TRUE");
    char c = GPSSerial.read();
    Serial.write(c);
  }

}
