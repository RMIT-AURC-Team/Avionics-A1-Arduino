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

// Converts byte array to 32 bit signed integer
template <typename T> T bytesToInt(uint8_t* bytes, int length) {
  // Early return if invalid length
  if(length <= 0)
    return 0;

  // Return typecast byte if length is 1
  if(length == 1)
    return bytes[0];

  // Otherwise shift in each byte and return result
  T result;
  for(int i = 0; i < length; i++) 
    result = (bytes[i] << 8) | bytes[i+1];

  return result;
}

uint8_t* readFromBuffer(uint8_t* buffer, int* i, int length) {
  uint8_t data[length];
  for(int j = 0; j < length; j++) {
    data[j] = buffer[*i++];
  }
  return data;
}

// Flash setup address
uint32_t pageAddr = 0;  // Starting page address

// Buffer size for storing sensor data
const size_t BUFFER_SIZE = 256;   // 256 byte per pages
uint8_t readBuffer[BUFFER_SIZE];  // Buffer to store sensor data
char str[50];

void loop() {

  SerialFlash.read(pageAddr, readBuffer, sizeof(readBuffer)); 

  // Read and validate each frame from buffer
  // Determines if first two bytes read are a valid header and prints out
  //    payload data according to the frame ID
  for(int i = 0; i < BUFFER_SIZE; i++) { 

    uint8_t dfID = ((readBuffer[i] >> 6) & 0x03);
    uint8_t dfLength = (readBuffer[i] & 0x3F);
    uint8_t dfSync = readBuffer[i+1];

    bool isValidHighRes = (dfID == 1) && (dfLength == 0x14);  // High res frame header ID=01 LENGTH=20B
    bool isValidLowRes = (dfID == 2) && (dfLength == 0x08);   // Low res frame header ID=10 LENGTH=8B

    if(isValidHighRes) {
      i++;
      uint8_t* accelData = readFromBuffer(readBuffer, &i, 6);
      uint8_t* gyroData = readFromBuffer(readBuffer, &i, 6);
      uint8_t* magnetData = readFromBuffer(readBuffer, &i, 6);

      // Convert bytes to 16 bit signed int
      int16_t accel = bytesToInt<int16_t>(accelData, 6);
      int16_t gyro = bytesToInt<int16_t>(gyroData, 6);
      int16_t magnet = bytesToInt<int16_t>(magnetData, 6);

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
      i++;
      uint8_t* pressureData = readFromBuffer(readBuffer, &i, 3);     // First 3 bytes are pressure
      uint8_t* temperatureData = readFromBuffer(readBuffer, &i, 3);  // Second 3 bytes are temperature
      
      // Convert bytes to 32 bit signed int
      int32_t pressure = bytesToInt<int32_t>(pressureData, 3);
      int32_t temperature = bytesToInt<int32_t>(temperatureData, 3);

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
