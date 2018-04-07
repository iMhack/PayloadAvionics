
#include <SPI.h>
#include <RF.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

#define GPSSerial Serial1
#define LED           2

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BME280 bme = Adafruit_BME280();
void setup()
{
  //setupRF()
  pinMode(LED, OUTPUT);
  Blink(LED,50,2);
  Serial.begin(9600);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("setup() START");
  GPS.begin(9600);
  delay(1000);
  mySerial.begin(9600);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("BNO config");
  if (not bno.begin()){
    Serial.println('Failed to initialize BNO055! Is the sensor connected?');}
  Serial.println("BME config");
  if (not bme.begin()){
    Serial.println('Failed to initialize BME280! Is the sensor connected?');}
  Serial.println("setup() END");
}


void loop() {
  //loopRF();

  /* Get a new sensor event */
sensors_event_t event;
bno.getEvent(&event);

/* Display the floating point data */
Serial.print("X: ");
Serial.print(event.orientation.x, 4);
Serial.print("\tY: ");
Serial.print(event.orientation.y, 4);
Serial.print("\tZ: ");
Serial.print(event.orientation.z, 4);
Serial.println("");
Serial.print("Temperature: ");
Serial.print(bme.readTemperature());
Serial.print("\tPressure: ");
Serial.print(bme.readPressure());
Serial.print("\Humidity: ");
Serial.print(bme.readHumidity());
Serial.println("");
delay(1000);
/*
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);

  if (Serial.available()) {
    char c = Serial1.read();
    GPSSerial.write(c);
    Blink(LED,50,1);
  }
  if (GPSSerial.available()) {
//    Serial.println("GPSSerial.available()==TRUE");
    char c = GPSSerial.read();
    Serial.write(c);
  }
  */

}
