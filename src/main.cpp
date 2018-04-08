#include <TeensyThreads.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

#include <utils.h>

#define LED 2

/* GPS DEFINES */
#define GPSSerial Serial1
#define GPSECHO true
//Adafruit_GPS GPS(&mySerial);
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;

/* BNO DEFINES */ // Add High G as : https://forums.adafruit.com/viewtopic.php?f=19&t=120348
Adafruit_BNO055 bno;
/* RF DEFINES */
#define RFM95_CS 9
#define RFM95_INT 29
#define RFM95_RST 24
#define RFM95_FREQ 433.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetnum = 0;
/* BME DEFINES */
Adafruit_BME280 bme;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Blink_(LED, 50, 2);
  Serial.begin(9600);
  while (!Serial){ delay(1);} // wait until serial console is open, remove if not tethered to computer
  Blink_(LED, 50, 1);
  Serial.println("setup() START");
  Serial1.begin(9600);
//  Serial1.println(PMTK_Q_RELEASE);
//  gps.sendCommand(PGCMD_ANTENNA);
  Serial.println("BNO config");
  if (not bno.begin())
    Serial.println("Failed to initialize BNO055! Is the sensor connected?");

  Serial.println("BME config");
  if (not bme.begin(&Wire1))
    Serial.print("Failed to initialize BME280! Is the sensor connected?");

  /*Start RF */
  Serial.println("RF config");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (not rf95.init())
    Serial.println("LoRa radio int failed");

  if (!rf95.setFrequency(RFM95_FREQ))
  {
    Serial.println("setFrequency failed");
  }
  else
  {
    Serial.print("Set Freq to: \t");
    Serial.println(RFM95_FREQ);
  }
  rf95.setTxPower(23, false);

  Serial.println("setup() END");
  Blink_(LED, 50, 3);
}

void loop()
{
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
  Serial.print("\tHumidity: ");
  Serial.print(bme.readHumidity());
  Serial.println("");

  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo(gps);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  // Send a message to rf95_server
/*
  Serial.println("Sending to rf95_server");
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket + 13, 10);
  Serial.print("Sending ");
  Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  if(rf95.waitPacketSent()){Serial.print("Error sending packet !");}//Is too too long !
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(1000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
*/
  delay(1000);

}