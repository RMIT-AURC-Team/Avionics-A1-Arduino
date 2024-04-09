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

unsigned long timeSensor(void(*f)(int16_t*), int16_t* data) {
  unsigned long startTime = micros(); 
  (*f)(data); 
  unsigned long endTime = micros(); 
  return endTime - startTime;
}

void loop() {
  
  // Gyro ---------------------------------------------------------------------------------------  
  int16_t gyro[3];  
  unsigned long gyroTime = timeSensor(readGyro, gyro);

  Serial.println("Gyro Sensor Time in microseconds: "); 
  Serial.println(gyroTime);   

  // Accel ---------------------------------------------------------------------------------------  
  int16_t accel[3];  
  unsigned long accelTime = timeSensor(readAccel, accel);

  Serial.println("Accel Sensor Time in microseconds: "); 
  Serial.println(accelTime);   

  // Baro ---------------------------------------------------------------------------------------  
  int32_t baro[2];  

  unsigned long startTime = micros(); 
  readBaro(baro);
  unsigned long endTime = micros(); 

  Serial.println("Barometer Sensor Time in microseconds: "); 
  Serial.println(endTime - startTime);   

  while(1); 
}

