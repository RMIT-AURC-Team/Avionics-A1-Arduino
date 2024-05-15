#include <SPL06-007.h> // Barometer (unused in this version)
#include <Wire.h>
#include "avionics.h"

void setup() {
    // I2C at 400kHz
    Wire.begin();
    Wire.setClock(400000UL);

    // Initialize gyro
    initGyro();

    Serial.begin(115200); // Begin Serial
}

void loop() {
    int16_t gyro[3]; // Array to store gyro readings
    readGyro(gyro);  // Read gyro data

    // Extract X, Y, and Z components with two decimal places
    double gyroX = (double)gyro[0]/ 13.375;
    double gyroY = (double)gyro[1]/ 13.375;;
    double gyroZ = (double)gyro[2]/ 13.375;;

    // Print gyro readings to serial with two decimal places
    Serial.print("Gyro (X, Y, Z): ");
    Serial.print(gyroX, 5);  
    Serial.print(", ");
    Serial.print(gyroY, 5);
    Serial.print(", ");
    Serial.println(gyroZ, 5);

    // Add a short delay to control the print rate
    delay(20); // Adjust this delay to get your desired sample rate (e.g., 50 for ~20Hz)
}
