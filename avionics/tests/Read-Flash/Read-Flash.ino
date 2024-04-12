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

  // Hold at last address
  if (pageAddr >= 0x1000000)
    while (1)
      ;

  SerialFlash.read(pageAddr, readBuffer, sizeof(readBuffer));

  // Read and validate each frame from buffer
  // Determines if first two bytes read are a valid header and prints out
  //    payload data according to the frame ID
  for (int i = 0; i < BUFFER_SIZE; i++) {

    uint8_t dfID = ((readBuffer[i] >> 6) & 0x03);
    uint8_t dfLength = (readBuffer[i] & 0x3F);

    bool isValidHighRes = (dfID == 1) && (dfLength == 0x14);  // High res frame header ID=01 LENGTH=20B
    bool isValidLowRes = (dfID == 2) && (dfLength == 0x08);   // Low res frame header ID=10 LENGTH=8B

    if (isValidHighRes) {
      uint8_t dfSync = readBuffer[i + 1];

      // Convert bytes to 16 bit signed int
      int16_t accelX = (readBuffer[i + 2] << 8) | readBuffer[i + 3];
      int16_t accelY = (readBuffer[i + 4] << 8) | readBuffer[i + 5];
      int16_t accelZ = (readBuffer[i + 6] << 8) | readBuffer[i + 7];

      int16_t gyroX = (readBuffer[i + 8] << 8) | readBuffer[i + 9];
      int16_t gyroY = (readBuffer[i + 10] << 8) | readBuffer[i + 11];
      int16_t gyroZ = (readBuffer[i + 12] << 8) | readBuffer[i + 13];

      int16_t magnetX = (readBuffer[i + 14] << 8) | readBuffer[i + 15];
      int16_t magnetY = (readBuffer[i + 16] << 8) | readBuffer[i + 17];
      int16_t magnetZ = (readBuffer[i + 18] << 8) | readBuffer[i + 19];
      i += 19;

      // Print out high res sensor data from frame
      Serial.println(pageAddr, HEX);
      Serial.print("Read high res dataframe, sync: ");
      Serial.println(dfSync);
      sprintf(str, "Accel: x=%d, y=%d, z=%d", accelX, accelY, accelZ);
      Serial.println(str);
      Serial.println(accelX * A_SENSITIVITY);
      Serial.println(accelY * A_SENSITIVITY);
      Serial.println(accelZ * A_SENSITIVITY);
      sprintf(str, "Gyro: x=%d, y=%d, z=%d", gyroX, gyroY, gyroZ);
      Serial.println(str);
      Serial.println(gyroX / G_SENSITIVITY);
      Serial.println(gyroY / G_SENSITIVITY);
      Serial.println(gyroZ / G_SENSITIVITY);
      sprintf(str, "Magnet: x=%d, y=%d, z=%d", magnetX, magnetY, magnetZ);
      Serial.println(str);
      Serial.println("-------------------------------------------------------------------");
    } else if (isValidLowRes) {
      uint8_t dfSync = readBuffer[i + 1];

      // Convert bytes to 32 bit signed int
      // TODO: add sign bit conversion
      int32_t pressure = (readBuffer[i] << 16) | (readBuffer[i + 1] << 8) | readBuffer[i + 2];
      int32_t temperature = (readBuffer[i + 3] << 16) | (readBuffer[i + 4] << 8) | readBuffer[i + 5];
      i += 9;

      // Print out low res sensor data from frame
      Serial.println(pageAddr, HEX);
      Serial.print("Read low res dataframe, sync: ");
      Serial.println(dfSync);
      sprintf(str, "Baro: pressure=%d, temp=%d", pressure, temperature);
      Serial.println(str);
      Serial.println("-------------------------------------------------------------------");
    } else {
      continue;
    }
  }

  pageAddr += 0x100;
}
