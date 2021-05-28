

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

//Adafruit_BMP280 bmp(BMP_CS);
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
uint32_t timeStartup = 0;

typedef struct bmpData {
  uint32_t timeStamp;
  int32_t raw_tFine;
  int64_t raw_p;
} bmpData_t;


void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 Teensy test"));

  if(!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    Serial.println(F("Could not connect to BMP280 :( sadge"));

    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  timeStartup = millis();
}


void loop() {
    bmpData_t thisBMPData = {(millis() - timeStartup), bmp.readTemperature(), bmp.readPressure()};
    
    Serial.print(F("Time Stamp = "));
    Serial.print(thisBMPData.timeStamp);
    Serial.println(" ms");
    
    Serial.print(F("Temperature = "));
    Serial.print(thisBMPData.raw_tFine);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    //Serial.write(thisBMPData.raw_p);
    Serial.println(" Pa");

    Serial.println();
    Serial.flush();
    delay(2000);
}
