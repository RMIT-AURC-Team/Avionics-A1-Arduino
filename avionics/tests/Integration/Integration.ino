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

// Timers for high and low res
unsigned long previousHighResolutionMicro = 0;
unsigned long previousLowResolutionMicro = 0;
// const unsigned long highResolutionInterval = 2000;  // Interval for high resolution (500Hz) in microseconds (1000000 microseconds / 500Hz)
const unsigned long highResolutionInterval = 4000;  // Interval for high resolution (250Hz) in microseconds (1000000 microseconds / 250Hz)
const unsigned long lowResolutionInterval = 20000;  // Interval for low resolution (50Hz) in microseconds (1000000 microseconds / 50Hz)

// Timer for Sync
uint8_t sync = 0;  
const unsigned long syncMicros = 249000;  // 249ms interval in microseconds

// Buffer size for storing sensor data
const size_t BUFFER_SIZE = 256;   // 256 byte per pages
uint8_t dataBuffer[BUFFER_SIZE];  // Buffer to store sensor data
uint8_t bufferIndex = 0;

void loop() {

  // Current time in microseconds
  unsigned long currentHighResMicro = micros();

  // High resolution loop (500Hz)
  if (currentHighResMicro - previousHighResolutionMicro >= highResolutionInterval) {

    // Check if 249ms has elapsed for sync
    sync = currentHighResMicro - previousHighResolutionMicro;
    if (sync >= syncMicros) {
      // Reset the timer
      sync = 0;
    }

    // Accel ---------------------------------------------------------------------------------------
    int16_t accel[3];
    readAccel(accel);  //read the accelerometer values and store them in variables  x,y,z
    uint16_t ax = (uint16_t)accel[0];
    uint16_t ay = (uint16_t)accel[1];
    uint16_t az = (uint16_t)accel[2];

    char str[50];
    sprintf(str, "Accel: x=%d y=%d z=%d", ax, ay, az);
    Serial.println(str);

    // Gyro ---------------------------------------------------------------------------------------
    int16_t gyro[3];
    readGyro(gyro);
    uint16_t gx = (uint16_t)gyro[0];
    uint16_t gy = (uint16_t)gyro[1];
    uint16_t gz = (uint16_t)gyro[2];

    sprintf(str, "Gyro: x=%d y=%d z=%d", gx, gy, gz);
    Serial.println(str);

    // Magnetometer/compass ------------------------------------------------------------------------
    int16_t magnet[3];
    readMagnet(magnet);
    // Cast to uint16_t
    uint16_t mx = (uint16_t)magnet[0];
    uint16_t my = (uint16_t)magnet[1];
    uint16_t mz = (uint16_t)magnet[2];

    sprintf(str, "Mag: x=%d y=%d z=%d", mx, my, mz);
    Serial.println(str);

    uint8_t syncMillis = sync / 1000;  // Convert microseconds to milliseconds and truncate

    // Create Buffer
    dataBuffer[bufferIndex++] = 0b01010100;  // 01 010100 (ID and length)

    dataBuffer[bufferIndex++] = syncMillis;

    dataBuffer[bufferIndex++] = (uint8_t)((ax >> 8) & 0xFF);  // Store the high byte of ax
    dataBuffer[bufferIndex++] = (uint8_t)(ax & 0xFF);         // Store the low byte of ax
    dataBuffer[bufferIndex++] = (uint8_t)((ay >> 8) & 0xFF);  // Store the high byte of ay
    dataBuffer[bufferIndex++] = (uint8_t)(ay & 0xFF);         // Store the low byte of ay
    dataBuffer[bufferIndex++] = (uint8_t)((az >> 8) & 0xFF);  // Store the high byte of az
    dataBuffer[bufferIndex++] = (uint8_t)(az & 0xFF);         // Store the low byte of az

    dataBuffer[bufferIndex++] = (uint8_t)((gx >> 8) & 0xFF);  // Store the high byte of gx
    dataBuffer[bufferIndex++] = (uint8_t)(gx & 0xFF);         // Store the low byte of gx
    dataBuffer[bufferIndex++] = (uint8_t)((gy >> 8) & 0xFF);  // Store the high byte of gy
    dataBuffer[bufferIndex++] = (uint8_t)(gy & 0xFF);         // Store the low byte of gy
    dataBuffer[bufferIndex++] = (uint8_t)((gz >> 8) & 0xFF);  // Store the high byte of gz
    dataBuffer[bufferIndex++] = (uint8_t)(gz & 0xFF);         // Store the low byte of gz

    dataBuffer[bufferIndex++] = (uint8_t)((mx >> 8) & 0xFF);  // Store the high byte of mx
    dataBuffer[bufferIndex++] = (uint8_t)(mx & 0xFF);         // Store the low byte of mx
    dataBuffer[bufferIndex++] = (uint8_t)((my >> 8) & 0xFF);  // Store the high byte of my
    dataBuffer[bufferIndex++] = (uint8_t)(my & 0xFF);         // Store the low byte of my
    dataBuffer[bufferIndex++] = (uint8_t)((mz >> 8) & 0xFF);  // Store the high byte of mz
    dataBuffer[bufferIndex++] = (uint8_t)(mz & 0xFF);         // Store the low byte of mz

    // reset high res timer
    previousHighResolutionMicro = currentHighResMicro;
  }

  // Low resolution loop (50Hz)
  unsigned long currentLowResMicro = micros();

  if (currentLowResMicro - previousLowResolutionMicro >= lowResolutionInterval) {

    // Check if 249ms has elapsed for sync
    sync = currentHighResMicro - previousHighResolutionMicro;
    if (sync >= syncMicros) {
      // Reset the timer
      sync = 0;
    }

    //Barometer ---------------------------------------------------------------------------------------------------
    int32_t baro[2];
    readBaro(baro);

    char str[50];
    sprintf(str, "Baro: pressure=%d temp=%d", baro[0], baro[1]);
    Serial.println(str);

    // convert currentMicro from mirco to millis
    uint8_t syncMillis = sync-previousLowResolutionMicro / 1000;  // Convert microseconds to milliseconds and truncate

    // Shift to 3 bytes and add to Buffer
    dataBuffer[bufferIndex++] = 0b10001000;  // 10 001000 (ID and length)
    dataBuffer[bufferIndex++] = syncMillis;

    dataBuffer[bufferIndex++] = (uint8_t)((baro[0] >> 16) & 0xFF);  // Store the first byte of pressure
    dataBuffer[bufferIndex++] = (uint8_t)((baro[0] >> 8) & 0xFF);   // Store the second byte of pressure
    dataBuffer[bufferIndex++] = (uint8_t)(baro[0] & 0xFF);          // Store the third byte of pressure

    dataBuffer[bufferIndex++] = (uint8_t)((baro[1] >> 16) & 0xFF);  // Store the first byte of temperature
    dataBuffer[bufferIndex++] = (uint8_t)((baro[1] >> 8) & 0xFF);   // Store the second byte of temperature
    dataBuffer[bufferIndex++] = (uint8_t)(baro[1] & 0xFF);          // Store the third byte of temperature

    // Write to flash
    Serial.println("Waiting for flash...");
    while (!SerialFlash.ready())
      ;
    Serial.println("Flash ready.");
    SerialFlash.write(pageAddr, dataBuffer, sizeof(dataBuffer));
    Serial.println("Page successfully written to flash.");
    Serial.println("");

    pageAddr += 0x100;
    bufferIndex = 0;
    previousLowResolutionMicro = currentLowResMicro;
  }
}
