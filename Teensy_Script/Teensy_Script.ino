#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <SD.h>

// ---------------------------------------------------------------------------------------------------------------- //
// GPS def
#define MAX_NMEA (80)
// BMP def
// IMU def
//#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 10         // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire1 // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0
#define redLED 3 // optional red status indication LED, could be a buzzer etc
// SD def
// Nand def

// ---------------------------------------------------------------------------------------------------------------- //
// GPS handle
SoftwareSerial mySerial(0, 1);
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
TinyGPS gps;
#define gpsPort Serial1
// BMP handle
Adafruit_BMP280 bmp;
uint32_t timeStartup = 0;
// IMU handle
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
// SD handle
//File logfile;
//char filename[20] = { '\0' };
// Nand handle

// ---------------------------------------------------------------------------------------------------------------- //
// GPS Struct
typedef struct gpsData {
  uint32_t timeStamp;
  char packets;
};
struct gpsData gpsData_t;
// BMP Struct
typedef struct bmpData {
  uint32_t timeStamp;
  int32_t raw_tFine;
  int64_t raw_p;
};
struct bmpData bmpData_t;
// IMU Struct
typedef struct ICM20948 {
  uint32_t timeStamp;
  int16_t accel_xout;
  int16_t accel_yout;
  int16_t accel_zout;
  int16_t gyr_xout;
  int16_t gyr_yout;
  int16_t gyr_zout;
  int16_t mag_xout;
  int16_t mag_yout;
  int16_t mag_zout;
  int16_t tmp_out;
};
struct ICM20948 ICM20948_t;

// ---------------------------------------------------------------------------------------------------------------- //
void setup()
{
  // GPS Setup
  Serial.begin(9600); //for debugging purposes
  while(!Serial);
  gpsPort.begin(9600);
  Serial.println("GPS Setup Complete");
  
  // BMP Setup
  if (!bmp.begin()) {
    Serial.println(F("Could not connect to BMP280 :( sadge"));

    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("BMP Setup Complete");

  // IMU Setup
  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

  #ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
  #else
    myICM.begin(WIRE_PORT, AD0_VAL);
  #endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
  
  // SD Setup
//  strncpy(filename, "/FLIGHT00.csv", 13);
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[7] = '0' + i/10;
//    filename[8] = '0' + i%10;
//    if (! SD.exists(filename)) {
//      break;
//    }
//  }
//
//  // attempt to open the file, restart otherwise
//  logfile = SD.open(filename, FILE_WRITE);
//  if(!logfile) {
//    digitalWrite(redLED, HIGH);
//    return;
//  }
//  // write data schema on top line of file
//  logfile.println("GPS_str,BMP_temp,BMP_press,BMP_alt,IMU_xaccel,IMU_yaccel,IMU_zaccel,IMU_xgyro,IMU_ygyro,IMU_zgyro,IMU_xmag,IMU_ymag,IMU_zmag,IMU_temp");
//  logfile.close();
  
  // Nand Setup
}

// ---------------------------------------------------------------------------------------------------------------- //
void loop()
{ 
  // GPS loop
  gpsData_t.timeStamp = (millis() - timeStartup);
  char c;
  if (gpsPort.available()) {
      gpsData_t.packets = gpsPort.read();
      c = gpsPort.read();
  }
  Serial.write(c);

    Serial.print(F("Time Stamp = "));
    Serial.print(gpsData_t.timeStamp);
    Serial.println(" ms");
    
    Serial.print(F("Packets = "));
    Serial.write(gpsData_t.packets);
    Serial.println("");

  // BMP loop
  bmpData_t.timeStamp = (millis() - timeStartup);
  bmpData_t.raw_tFine = bmp.readTemperature();
  bmpData_t.raw_p = bmp.readPressure();
    
    Serial.print(F("Time Stamp = "));
    Serial.print(bmpData_t.timeStamp);
    Serial.println(" ms");
    
    Serial.print(F("Temperature = "));
    Serial.print(bmpData_t.raw_tFine);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(int(bmpData_t.raw_p));
    Serial.println(" Pa");

    Serial.println();
    Serial.flush();

  // IMU loop
  if (myICM.dataReady())
  {
    myICM.getAGMT();              // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    //Timestamp
    ICM20948_t.timeStamp = (millis() - timeStartup);
    
    //Accel Data
    ICM20948_t.accel_xout = myICM.accX();
    ICM20948_t.accel_yout = myICM.accY();
    ICM20948_t.accel_zout = myICM.accZ();

    //Gyr Data
    ICM20948_t.gyr_xout = myICM.gyrX();
    ICM20948_t.gyr_yout = myICM.gyrY();
    ICM20948_t.gyr_zout = myICM.gyrZ();

    //Mag Data
    ICM20948_t.mag_xout = myICM.magX();
    ICM20948_t.mag_yout = myICM.magY();
    ICM20948_t.mag_zout = myICM.magZ();

    //Temp Data
    ICM20948_t.tmp_out = myICM.temp();

    Serial.print(F("Time Stamp = "));
    Serial.print(ICM20948_t.timeStamp);
    Serial.println(" ms");

    Serial.print(F("AccelX = "));
    Serial.print(ICM20948_t.accel_xout);
    Serial.println("");

    Serial.print(F("AccelY = "));
    Serial.print(ICM20948_t.accel_yout);
    Serial.println("");

    Serial.print(F("AccelZ = "));
    Serial.print(ICM20948_t.accel_zout);
    Serial.println("");

    Serial.print(F("GyrX = "));
    Serial.print(ICM20948_t.gyr_xout);
    Serial.println("");

    Serial.print(F("GyrY = "));
    Serial.print(ICM20948_t.gyr_yout);
    Serial.println("");

    Serial.print(F("GyrZ = "));
    Serial.print(ICM20948_t.gyr_zout);
    Serial.println("");

    Serial.print(F("MagX = "));
    Serial.print(ICM20948_t.mag_xout);
    Serial.println("");

    Serial.print(F("MagY = "));
    Serial.print(ICM20948_t.mag_yout);
    Serial.println("");

    Serial.print(F("MagZ = "));
    Serial.print(ICM20948_t.mag_zout);
    Serial.println("");

    Serial.print(F("Temp = "));
    Serial.print(ICM20948_t.tmp_out);
    Serial.println("");
  }
  
  // Write SD

  // Write Nand
}
