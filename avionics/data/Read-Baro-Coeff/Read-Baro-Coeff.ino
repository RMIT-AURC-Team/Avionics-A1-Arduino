#include <SPL06-007.h>  // Baro
#include <SerialFlash.h>
#include <SPI.h>
#include <Wire.h>

// Define the chip select
#define CSPIN 10

void setup() {
  Wire.begin();
  Serial.begin(115200);  // begin Serial
  delay(5000);
}

void loop() {
  Serial.print("c00: ");
  Serial.println(get_c00());
  Serial.print("c10: ");
  Serial.println(get_c10());
  Serial.print("c01: ");
  Serial.println(get_c01());
  Serial.print("c11: ");
  Serial.println(get_c11());
  Serial.print("c20: ");
  Serial.println(get_c20());
  Serial.print("c21: ");
  Serial.println(get_c21());
  Serial.print("c30: ");
  Serial.println(get_c30());
  while (1)
    ;
}
