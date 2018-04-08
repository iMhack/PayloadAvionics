#include <utils.h>

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
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

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
/*
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
*/
// static void printFloat(float val, bool valid, int len, int prec)
// {
//   if (!valid)
//   {
//     while (len-- > 1)
//       Serial.print('*');
//     Serial.print(' ');
//   }
//   else
//   {
//     Serial.print(val, prec);
//     int vi = abs((int)val);
//     int flen = prec + (val < 0.0 ? 2 : 1); // . and -
//     flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//     for (int i = flen; i < len; ++i)
//       Serial.print(' ');
//   }
//   smartDelay(0);
// }

void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

// static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
// {
//   if (!d.isValid())
//   {
//     Serial.print(F("********** "));
//   }
//   else
//   {
//     char sz[32];
//     sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
//     Serial.print(sz);
//   }

//   if (!t.isValid())
//   {
//     Serial.print(F("******** "));
//   }
//   else
//   {
//     char sz[32];
//     sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
//     Serial.print(sz);
//   }

//   printInt(d.age(), d.isValid(), 5);
//   smartDelay(0);
// }

// static void printStr(const char *str, int len)
// {
//   int slen = strlen(str);
//   for (int i = 0; i < len; ++i)
//     Serial.print(i < slen ? str[i] : ' ');
//   smartDelay(0);
// }
