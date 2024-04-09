#include <SPL06-007.h>  // Baro
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>
#include "avionics.h"

// Define the chip select
#define CSPIN 10

// Timer for Sync 
unsigned long previousMicros = 0;
const unsigned long syncMicros = 249000; // 249ms interval in microseconds

void setup() {
  // i2c at 400kHz
  Wire.begin();
  Wire.setClock(400000UL);

  // Initialise sensors
  initGyro();
  initAccel();
  initMagnet();
  initBarometer();

  Serial.begin(115200);          // begin Serial

  // Initialise flash
  if (!SerialFlash.begin(CSPIN)) {
    while (1) {
      Serial.println(F("Unable to access SPI Flash chip"));
      delay(1000);
    }
  }
}

void loop() {
  Serial.println("Erasing");
  SerialFlash.eraseAll();
  while(!SerialFlash.ready()) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println("Erase complete.");
  while(1);
}

