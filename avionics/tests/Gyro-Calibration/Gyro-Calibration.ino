#include <SPL06-007.h>  // Baro
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>
#include "avionics.h"

// Define the chip select
#define CSPIN 10

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

  Serial.begin(115200);  // begin Serial

  // Initialise flash
  if (!SerialFlash.begin(CSPIN))
    while (1)
      ;
}

// Flash setup address
uint32_t pageAddr = 0;  // Starting page address

// Buffer size for storing sensor data
const size_t BUFFER_SIZE = 256;   // 256 byte per pages
uint8_t dataBuffer[BUFFER_SIZE];  // Buffer to store sensor data
uint8_t bufferIndex = 0;

int16_t _accel[3];
bool isLaunch = false;
const double launchAccel = 2;

// Variables for gyro averaging
const int numGyroSamples = 100; // Number of samples to avg
int16_t gyroReadings[3][numGyroSamples]; // array to store gyro data
int gyroSampleIndex = 0; // Init gyroSampleIndex 
int16_t gyroOffsets[3] = {0, 0, 0}; //array for gyro offsets
// Drift threshold for gyro offset 
const int16_t Gyro_Drift_Threshold = 1; // (raw value) 1/13.135


void loop() {
  while (!isLaunch) {
    
    // Read gyro and calculate avg
    int16_t gyro[3];
    readGyro(gyro);

    // Store gyro readings in array
    gyroReadings[0][gyroSampleIndex] = gyro[0];
    gyroReadings[1][gyroSampleIndex] = gyro[1];
    gyroReadings[2][gyroSampleIndex] = gyro[2];
    gyroSampleIndex = (gyroSampleIndex + 1) % numGyroSamples; // to wrap around

    // Calculate avg gyro reading
    long sumGyro[3] = {0, 0, 0};
    for (int i = 0; i < numGyroSamples; i++) {
        sumGyro[0] += gyroReadings[0][i];
        sumGyro[1] += gyroReadings[1][i];
        sumGyro[2] += gyroReadings[2][i];
    }
    
    // Calculate new gyro offsets
    int16_t newGyroOffsets[3] = {
      sumGyro[0] / numGyroSamples,
      sumGyro[1] / numGyroSamples,
      sumGyro[2] / numGyroSamples
    };

    // check for within threshold
     bool Drift_Within_Threshold = true;
    for (int i = 0; i < 3; i++) {
      if (abs(newGyroOffsets[i] - gyroOffsets[i]) > Gyro_Drift_Threshold) {
        Drift_Within_Threshold = false;
        break;
      }
    }

    // Update gyroOffsets only if change is within threshold
    if (Drift_Within_Threshold) {
      for (int i = 0; i < 3; i++) {
        gyroOffsets[i] = newGyroOffsets[i];
      }
    }

    // Print raw gyro readings
    Serial.print("Raw Gyro (x, y, z): ");
    Serial.print(gyro[0]);
    Serial.print(", ");
    Serial.print(gyro[1]);
    Serial.print(", ");
    Serial.println(gyro[2]);

    // Print average gyro offsets
    Serial.print("Gyro Offsets (x, y, z): ");
    Serial.print(gyroOffsets[0]);
    Serial.print(", ");
    Serial.print(gyroOffsets[1]);
    Serial.print(", ");
    Serial.println(gyroOffsets[2]);

    
    readAccel(_accel);
    double _accelX = _accel[0] * A_SENSITIVITY;
    Serial.println(_accelX);
    if (abs(_accelX) > launchAccel) {
      isLaunch = true;
      
      // Write gyro offsets to flash as the first data frame
      uint8_t syncMillis = 0; // Sync time is 0 for the first frame
      dataBuffer[bufferIndex++] = 0b01101011; // ID and length for gyro data (0x6B)
      dataBuffer[bufferIndex++] = syncMillis;
      
      for (int i = 0; i < 3; i++) {
          dataBuffer[bufferIndex++] = (uint8_t)((gyroOffsets[i] >> 8) & 0xFF);
          dataBuffer[bufferIndex++] = (uint8_t)(gyroOffsets[i] & 0xFF);
      }
      
      while (!SerialFlash.ready());
      SerialFlash.write(pageAddr, dataBuffer, bufferIndex); // Write calibration as first data frame


    }
  }


}
