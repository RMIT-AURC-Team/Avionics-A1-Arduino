#include <SPL06-007.h>  // Baro
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>
#include "avionics.h"

// Define the chip select
#define CSPIN 10

// Define terminal pin connections
#define TERMINAL_4 4  // Blue Raven takeoff
#define TERMINAL_M 5  // Blue Raven apogee

double groundpressure;
int flagTakeoff = 0;

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
  groundpressure = get_pcomp();  // provides initial ground pressure to reference

  // Initialise flash
  if (!SerialFlash.begin(CSPIN)) {
    while (1) {
      Serial.println(F("Unable to access SPI Flash chip"));
      delay(1000);
    }
  }
}

// Flash setup address
uint32_t pageAddr = 0;  // Starting page address

// Buffer size for storing sensor data
const size_t BUFFER_SIZE = 256;   // 256 byte per pages
uint8_t readBuffer[BUFFER_SIZE];  // Buffer to store sensor data
char str[50];

void loop() {

  SerialFlash.read(pageAddr, readBuffer, sizeof(readBuffer)); 

  // Print each byte read into buffer
  for(int i = 0; i < BUFFER_SIZE; i++) { 

    uint8_t dfID = ((readBuffer[i] >> 6) & 0x02);
    uint8_t dfLength = (readBuffer[i] & 0x3F);
    uint8_t dfSync = readBuffer[i++];

    bool isValidHighRes = (dfID == 1) && (dfLength == 0x12);
    bool isValidLowRes = (dfID == 2) && (dfLength == 0x08);

    if(isValidHighRes) {
      int16_t accelData[3] = {readBuffer[i++], readBuffer[i++], readBuffer[i++]};
      int16_t gyroData[3] = {readBuffer[i++], readBuffer[i++], readBuffer[i++]};
      int16_t magnetData[3] = {readBuffer[i++], readBuffer[i++], readBuffer[i++]};

      // Print out high res sensor data from frame
      Serial.println(pageAddr, HEX);
      Serial.print("Read high res dataframe, sync: ");
      Serial.println(dfSync);
      sprintf(str, "Accel: x=%d, y=%d, z=%d", accelData[0], accelData[1], accelData[2]);
      Serial.println(str);
      sprintf(str, "Gyro: x=%d, y=%d, z=%d", gyroData[0], gyroData[1], gyroData[2]);
      Serial.println(str);
      sprintf(str, "Magnet: x=%d, y=%d, z=%d", magnetData[0], magnetData[1], magnetData[2]);
      Serial.println(str);
      Serial.println("-------------------------------------------------------------------");
    } else if(isValidLowRes) {
      int32_t baroData[2] = {readBuffer[i++], readBuffer[i++]};
      // Print out low res sensor data from frame
      Serial.println(pageAddr, HEX);
      Serial.print("Read low res dataframe, sync: ");
      Serial.println(dfSync);
      sprintf(str, "Baro: pressure=%d, temp=%d", baroData[0], baroData[1]);
      Serial.println(str);
      Serial.println("-------------------------------------------------------------------");
    } else {
      continue;
    }

  }
  pageAddr += 0x100;
}