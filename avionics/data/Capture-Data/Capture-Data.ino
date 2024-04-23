#include <SPL06-007.h>  // Baro
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>

// Define the chip select
#define CSPIN 10

void setup() {
  Serial.begin(115200);  // begin Serial

  // Initialise flash
  if (!SerialFlash.begin(CSPIN)) {
    while (1) {
      Serial.println(F("Unable to access SPI Flash chip"));
      delay(1000);
    }
  }
  delay(10000);
}

// Flash setup address
uint32_t pageAddr = 0;  // Starting page address

// Buffer size for storing sensor data
const size_t BUFFER_SIZE = 256;   // 256 byte per pages
uint8_t readBuffer[BUFFER_SIZE];  // Buffer to store sensor data


void loop() {

  // Hold at last address
  if (pageAddr >= 0x1000000) {
    Serial.end();
    while (1)
      ;
  }

  SerialFlash.read(pageAddr, readBuffer, sizeof(readBuffer));

  // Dump out buffer data to serial
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print((char)readBuffer[i]);
  }

  pageAddr += 0x100;
}
