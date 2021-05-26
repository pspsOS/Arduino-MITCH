#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <string.h>
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define MAX_NMEA (80)
#define gpsI2CAddress 0x12
#define BUF_SIZE (8)

char gpsString[BUF_SIZE][MAX_NMEA];
char buff;
int n = 0;

void setup()
{
  Serial.begin(115200); //for debugging purposes
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
}

void loop()
{ 
  int c = GPS.read();
  if(GPS.newNMEAreceived()){
    strncpy(gpsString[n++], GPS.lastNMEA(), MAX_NMEA);
  }
  Serial.print(gpsString[BUF_SIZE][MAX_NMEA]);
}
