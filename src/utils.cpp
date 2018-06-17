#include <utils.h>

extern uint32_t datagramSeqNumber;



void Blink_(int PIN, int DELAY_MS, int loops)
{
  for (int i = 0; i < loops; i++)
  {
    digitalWrite(PIN, 1);
    threads.delay(DELAY_MS);
    digitalWrite(PIN, 0);
    delay(DELAY_MS);
  }
}

void displayInfo(TinyGPSPlus &gps)
{
  Serial.print(F("Location: "));
  /*if (gps.location.isValid())
  {*/
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  /*}
  else
  {
    Serial.print(F("INVALID"));
  }*/
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}

void displayInfo(Adafruit_BNO055 &bno){

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("aX: ");
  Serial.print(accel[0]);
  Serial.print("\taY: ");
  Serial.print(accel[1]);
  Serial.print("\taZ: ");
  Serial.print(accel[2]);
  Serial.println("");
  /*
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  */
}

void displayInfo(Adafruit_BME280 &bme){
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.print("\tPressure: ");
  Serial.print(bme.readPressure());
  Serial.print("\tHumidity: ");
  Serial.print(bme.readHumidity());
  Serial.println("");
}

//-----------------------------------------------------------------------------
//datagram pour groundstation
//-----------------------------------------------------------------------------

void CreateTelemetryDatagram_GPS(float lat, float lng,float altitude,uint32_t measurement_time, uint8_t * datas){
//-----------------------------------------------------------------------------
//preamble setting
//-----------------------------------------------------------------------------
  int currentPos = 0;
  uint16_t datagramCrc = CRC_16_GENERATOR_POLY.initialValue;//checksum initialization

  for(int i = 0; i<PREAMBLE_SIZE;i++)
  {
     write8(HEADER_PREAMBLE_FLAG,datas,currentPos); //currentPos==4
  }//Preamble flags

  write32u(datagramSeqNumber++,datas,currentPos);//Sequence number, currentPos==8
  write8(GPS,datas,currentPos);//Payload type, currentPos==9

  for (int i = currentPos - 5; i < currentPos; i++)
  {
    //Calculate checksum for datagram and payload fields
    datagramCrc = CalculateRemainderFromTable (datas[i], datagramCrc);
  }
//curentPos==9

  write8(CONTROL_FLAG,datas,currentPos);//Control flag, currentPos==10
    //  Serial.println("TableInHex");
    //  Serial.println(accel[2]);
  write32u(measurement_time,datas,currentPos);//Timestamp, currentPos==14

  //-----------------------------------------------------------------------------
  //GPS data
  //-----------------------------------------------------------------------------
  write8(0,datas,currentPos);//satellite count,currentPos==15
  write32f(0.0f,datas,currentPos);//rssi, currentpos==19
  write32f(lat,datas,currentPos);//currentPos==23
  write32f(lng,datas,currentPos);//currentPos==27
  write32f(altitude,datas, currentPos);//currentPos==31
  //serial output for checking
  /*Serial.println("lat, lng, altitude :");
  //POURUOI CA NE DETECTE PAS LE FOUTUS STRING
  std::stringstream var;
  var<<lat<<" "<<lng<<" "<<altitude;
  Serial.println(var.str()); */

  //-----------------------------------------------------------------------------
  //CHECKSUM
  //-----------------------------------------------------------------------------

  for (int i = (PREAMBLE_SIZE + HEADER_SIZE + CONTROL_FLAG_SIZE); i < currentPos; i++)
   {
     //Calculate checksum for datagram and payload fields
     datagramCrc = CalculateRemainderFromTable (datas[i], datagramCrc);
   }
   datagramCrc = FinalizeCRC (datagramCrc);
  write16 (datagramCrc,datas,currentPos);
  Serial.print("Checksum GPS: ");Serial.print(datagramCrc,HEX); Serial.println();
  //-----------------------------------------------------------------------------
  //HEX test
  //-----------------------------------------------------------------------------
  for(int i = 0; i<SENSOR_PACKET_SIZE; i++){
    Serial.print(datas[i],HEX);
  }Serial.println();

}

void createTelemetryDatagram (imu::Vector<3> accel, imu::Vector<3> euler, BARO_data baro, uint32_t measurement_time, uint8_t * datas)
{
  int currentPos = 0;
  uint16_t datagramCrc = CRC_16_GENERATOR_POLY.initialValue;
  for(int i = 0; i<PREAMBLE_SIZE;i++){write8(HEADER_PREAMBLE_FLAG,datas, currentPos);}//Preamble flags, POS=4
  write32u(datagramSeqNumber++,datas,currentPos);//Sequence number, POS=4-7 -> 8
  write8(TELEMETRY_ERT18,datas,currentPos);//Payload type, POS=8 -> 9
  //Serial.println(datagramCrc,HEX);

  for (int i = 4; i < currentPos; ++i)
  {
    //Calculate checksum for datagram and payload fields
    datagramCrc = CalculateRemainderFromTable (datas[i], datagramCrc);
    //Serial.println(datagramCrc,HEX);
  }


  write8(CONTROL_FLAG,datas,currentPos);//Control flag, POS=9, -> 10
//  Serial.println("TableInHex");
//  Serial.println(accel[2]);
  write32u(measurement_time,datas,currentPos);// POS -> 10-13 -> 14
  write32f(accel[0],datas,currentPos);
  write32f(accel[1],datas,currentPos);
  write32f(accel[2],datas,currentPos);
  write32f(euler[0],datas,currentPos);
  write32f(euler[1],datas,currentPos);
  write32f(euler[2],datas,currentPos);
  write32f(baro.temperature,datas,currentPos);
  write32f(baro.pressure,datas,currentPos);
  write32f(0,datas,currentPos);

  for (int i = 10; i < currentPos; ++i)
   {
     //Calculate checksum for datagram and payload fields
     datagramCrc = CalculateRemainderFromTable (datas[i], datagramCrc);
     //Serial.println(datagramCrc,HEX);

   }

   datagramCrc = FinalizeCRC (datagramCrc);
  write16 (datagramCrc,datas,currentPos);
  Serial.print("Checksum telemetry: ");Serial.print(datagramCrc,HEX); Serial.println();

}

//-----------------------------------------------------------------------------
//write# definition
//-----------------------------------------------------------------------------

inline void write8 (uint8_t v, uint8_t* datas, int &currentPos){
  datas[currentPos++]=v;
}

inline void write16 (uint16_t v, uint8_t* datas, int &currentPos){
  datas[currentPos++]=(v&0xFF00) >> 8;
  datas[currentPos++]=(v&0x00FF) >> 0;
}

inline void write32u (uint32_t v, uint8_t* datas, int &currentPos){
  datas[currentPos++]=(v&0xFF000000) >> 24;
  datas[currentPos++]=(v&0x00FF0000) >> 16;
  datas[currentPos++]=(v&0x0000FF00) >> 8;
  datas[currentPos++]=(v&0x000000FF) >> 0;
}

inline void write32f (float v, uint8_t* datas, int &currentPos){
  uint8_t *p = (uint8_t*)&v;
  for(int i = 3; i>=0; i--){
    datas[currentPos++]=p[i];
  }
}
