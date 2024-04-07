#ifndef __AVIONICS_H
#define __AVIONICS_H
#include <stdint.h>

/******** GYRO ********/
void initGyro();
void readGyro(int16_t *data);
void readGyro(int16_t *x, int16_t *y, int16_t *z);

/******** Accelerometer ********/
void initAccel();
void readAccel(int16_t *x, int16_t *y, int16_t *z);

/******** Magnetometer ********/
void initMagnet();
void readMagnet(int16_t *x, int16_t *y, int16_t *z, int16_t *t);

/******** Barometer ********/
void initBarometer();
void readPressure(int32_t *pressure);
void readTemp(int32_t *temp);

/******** i2c ********/
void i2cWrite(int device, int address, int value);
void i2cRead(int device, int address, int bytes, char *buff);

#endif
