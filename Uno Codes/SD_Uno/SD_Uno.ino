/*
 * Code to have a slave read from two master arduinos
 * each passing different data packets. Arbitrary I2C addresses are used
 * and certain  pin assumptions are made and defined in the #defines.
 * There is an open question as to if constantly switching using the Wire library
 * will cause problems, and if instead we should use recieveEvents or something
 * like that. Multi-master I2C among arduinos with Wire.h is kind of a grey area.
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
// SDA - A4
// SCL - A5

#define masterRead 3 // pin connected to the timer uno which dictates when to read from imu/bmp or gps
#define cardSelect 4 // pin for sd card interface
#define bmpimuI2CAddress 0x11 // chosen at random
#define gpsI2CAddress 0x12
#define loggingLED 11 // optional green status indication LED, could be a buzzer etc
#define redLED 13 // optional red status indication LED, could be a buzzer etc
#define BAUD 115200

File logfile;
char filename[20] = { '\0' };

// data definitions for gps
char gpsString[81] = { '\0' };
// data definitions for imu/bmp
enum incoming {
  bmp = -4,
  imu,
  endline,
  endtransmit
};
float flagNum = -3;
float bmpString[3];
float imuString[10];

boolean isread = false;

void setup() {
  Serial.begin(BAUD);
  
  // set pins
  pinMode(redLED, OUTPUT);
  pinMode(loggingLED, OUTPUT);
  digitalWrite(loggingLED, LOW);
  digitalWrite(redLED, LOW);
  pinMode(masterRead, INPUT);

  // see if card can be initialized
  if (!SD.begin(cardSelect)) {
    // if not, set status LED and return
    // the script *should* start over because an arduino can only
    // really 'exit' by going into an infinite while loop
    digitalWrite(redLED, HIGH);
    return;
  }

  // create filename that will not overwrite other flight data
  strncpy(filename, "/FLIGHT00.csv", 13);
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }

  // attempt to open the file, restart otherwise
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    digitalWrite(redLED, HIGH);
    return;
  }
  // write data schema on top line of file
  logfile.println("GPS_str,BMP_temp,BMP_press,BMP_alt,IMU_xaccel,IMU_yaccel,IMU_zaccel,IMU_xgyro,IMU_ygyro,IMU_zgyro,IMU_xmag,IMU_ymag,IMU_zmag,IMU_temp");
  logfile.close();

  // set LED to 'logging mode' (aka on)
  digitalWrite(loggingLED, HIGH);
}

/*
 * As a disclaimer, multimaster I2C with the Wire.h library is kind of a 
 * grey area but if everything is timed this *should* work.
 */
void loop() {

  logfile = SD.open(filename, FILE_WRITE);
  Serial.println(logfile);
  if(logfile) {
    int readType = digitalRead(masterRead);
  Serial.println(readType);
    if (readType == HIGH) {
      Serial.println("Jeff");
      // read from the gps
      // establish i2c with gps
      Wire.begin(gpsI2CAddress);
      isread = false;
      int n = 0;
      while (isread == false) {
        if (Wire.available()) {
          gpsString[n] = (char) Wire.read();
          n++;
          Serial.println(n);
        }
        if (n == 80) {
          isread = true;
        }
      }

      // once 80 characters is collected, write to csv along with empty after-lines
      logfile.print(gpsString);
      logfile.print(", , , , , , , , , , , , , \n");
    } else {
      // read from the imu/bmp
      // establish i2c with imu/bmp
      Wire.begin(bmpimuI2CAddress);
      isread = false;
      eraseContents();
      while (isread == false) {
        if (Wire.available()) {
          incoming rt = Wire.read();
          if (rt == imu) {
            for (int i = 0; i < 10; i++) {
              imuString[i] = (float) Wire.read();
            }
          } else if (rt == bmp) {
            for (int i = 0; i < 3; i++) {
              bmpString[i] = (float) Wire.read();
            }
          } else if (rt == endline) {
            logfile.print(" ,"); // for empty gps
            if (bmpString[0] != -1) {
              logfile.print(bmpString[0]);
              logfile.print(",");
              logfile.print(bmpString[1]);
              logfile.print(",");
              logfile.print(bmpString[2]);
              logfile.print(",");
            } else {
              logfile.print(" , , ,");
            }
            if (imuString[0] != -1) {
              for (int i = 0; i < 9; i++) {
                logfile.print(imuString[i]);
                logfile.print(",");
              }
              logfile.print(imuString[9]);
              logfile.print("\n");
            } else {
              logfile.print(" , , , , , , , , , \n");
            }
            eraseContents();
          } else if (rt == endtransmit) {
            isread = true;
          }
        }
      }
    }
    logfile.close();
  }
  
}

void eraseContents() {
  memcpy(imuString, -1, sizeof(imuString));
  memcpy(bmpString, -1, sizeof(bmpString));
}
