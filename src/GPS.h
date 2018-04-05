#include <avr/pgmspace.h>


/*
GPS configuration:
$PMTK251,115200*1F              -> set rate to 115200 baud/s
$PMTK300,100,0,0,0,0*2C         -> set fix rate to 100ms, 10Hz
$PMTK500,100,0,0,0,0*2A         -> set position fix to 200ms, 5Hz
(optionally) NMEA output 514 to reduce amount of data sent by GPS unit

*/

const char GPS_set_baud_rate[] PROGMEM = "$PMTK251,115200*1F";
const char GPS_fix_rate[] PROGMEM = "$PMTK300,100,0,0,0,0*2C";
const char GPS_pos_fix_rate[] PROGMEM = "$PMTK500,200,0,0,0,0*29";
