#include "avionics.h"
#include <Wire.h>

/******** GYRO ********/
//void initGyro(){
//  return;
//}
void readGyro(int16_t *data){}
void readGyro(int16_t *x, int16_t *y, int16_t *z){}

/******** Accelerometer ********/
void initAccel(){}
void readAccel(int16_t *data){}
void readAccel(int16_t *x, int16_t *y, int16_t *z){}

/******** Magnetometer ********/
void initMagnet(){}
void readMagnet(int16_t *data){}
void readMagnet(int16_t *x, int16_t *y, int16_t *z, int16_t *t){}

/******** Barometer ********/
void initBarometer(){}
void readPressure(int32_t *pressure){}
void readTemp(int32_t *temp){}

/******** i2c ********/
void i2cWrite(int device, int address, int value) {
  Wire.beginTransmission(device);   //start transmission 
  Wire.write(address);              // send register address
  Wire.write(value);                // send value to write
  Wire.endTransmission(); 
}

void i2cRead(int device, int address, int bytes, char* buff) {
  Wire.beginTransmission(device);   //start transmission 
  Wire.write(address);              //sends address to read from
  Wire.endTransmission(false);      //end transmission

  Wire.beginTransmission(device);   //start transmission 
  Wire.requestFrom(device, bytes);  // request bytes from device

  // Read each available byte to buffer
  int i = 0;
  while(Wire.available()) { 
   buff[i] = Wire.read(); 
   i++;
  }
  Wire.endTransmission(); 
}
