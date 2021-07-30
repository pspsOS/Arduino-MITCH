//#define __arm__
//#define TEENSYDUINO
//#define __IMXRT1052__

#include <SoftwareSerial.h>
#include <string.h>
#include <Wire.h>
#include "nandInterface.h"



//global var
bool nandNom = false;

void setup() {
  // put your setup code here, to run once:
  // Comment out the setFeature() calls in the nandInit() function of nandInterface.cpp
  // Should expect the print function to return 00111000b or 56 
  nandInit(&nandNom);
  uint8_t featA = getFeature(FEATURE_ADDR_A);
  uint8_t featAbit3 = featA & 0x08;
  Serial.println(featA);
  //Serial.println(featAbit3);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
