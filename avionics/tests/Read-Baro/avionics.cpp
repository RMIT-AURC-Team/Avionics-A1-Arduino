#include "avionics.h"
#include <Wire.h>

/******** GYRO ********/
                     
void initGyro(){
/*****************************************
 * ITG 3200
 * power management set to:
 * clock select = internal oscillator
 * no reset, no sleep mode
 * no standby mode
 * sample rate to = 125Hz
 * parameter to +/- 2000 degrees/sec
 * low pass filter = 5Hz
 * no interrupt
 ******************************************/
  i2cWrite(GYRO, G_PWR_MGM, 0x05);
  i2cWrite(GYRO, G_SMPLRT_DIV, 0x07);  // EB, 50, 80, 7F, DE, 23, 20, FF
  i2cWrite(GYRO, G_DLPF_FS, 0x1E);     // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  i2cWrite(GYRO, G_INT_CFG, 0x00);
}

void readGyro(int16_t *data){
  readGyro(data, data+1, data+2);
}

void readGyro(int16_t *x, int16_t *y, int16_t *z){
/**************************************
 Gyro ITG-3200 I2C
 registers:
 temp MSB = 1B, temp LSB = 1C
 x axis MSB = 1D, x axis LSB = 1E
 y axis MSB = 1F, y axis LSB = 20
 z axis MSB = 21, z axis LSB = 22
 *************************************/
  int g_offx = 0;
  int g_offy = 0;
  int g_offz = 0;
  int regAddress = 0x1B;
  uint8_t buff[G_TO_READ];
  i2cRead(GYRO, regAddress, G_TO_READ, buff); // read the gyro data from the ITG3200
  *x = ((buff[2] << 8) | buff[3]) + g_offx;   // they are 2 byte registers within the sensor
  *y = ((buff[4] << 8) | buff[5]) + g_offy;
  *z = ((buff[6] << 8) | buff[7]) + g_offz;

  // Temperature
  // Currently unneeded
  int16_t *t;
  *t = (buff[0] << 8) | buff[1];              
}

/******** Accelerometer ********/
void initAccel(){
    // Power control
    i2cWrite(ACCEL, ADXL345_POWER_CTL, 0);  
    i2cWrite(ACCEL, ADXL345_POWER_CTL, ADXL345_POWER_CTL_MEASURE); 
    i2cWrite(ACCEL, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_RANGE_16);

    // Setup interrupts
    // For now, don't use interrupts
    i2cWrite(ACCEL, ADXL345_INT_ENABLE, 0);

    /* Alternatively, enable interrupts. Activity may be handy
     * for detecting launch event in absence of Blue Raven. Freefall
     * can be likewise be used for apogee.
     *
      uint8_t intMap = ADXL345_INT_ENABLE_ACTIVITY
                     | ADXL345_INT_ENABLE_INACTIVITY
                     | ADXL345_INT_ENABLE_FREEFALL
      i2cWrite(ACCEL, ADXL345_INT_ENABLE, intMap);
     */
}

void readAccel(int16_t *data){
  readAccel(data, data+1, data+2);
}

void readAccel(int16_t *x, int16_t *y, int16_t *z){
  uint8_t buff[A_TO_READ];
  i2cRead(ACCEL, ADXL345_DATAX0, A_TO_READ, buff); 
  *x = (int16_t)((((uint16_t)buff[1]) << 8) | buff[0]);   
  *y = (int16_t)((((uint16_t)buff[3]) << 8) | buff[2]);
  *z = (int16_t)((((uint16_t)buff[5]) << 8) | buff[4]);
}

/******** Magnetometer ********/

void initMagnet(){
  uint8_t oversampling = QMC5883L_CONFIG_OS512;
  uint8_t range = QMC5883L_CONFIG_2GAUSS;
  uint8_t rate = QMC5883L_CONFIG_200HZ;
  uint8_t mode = QMC5883L_CONFIG_CONT;

  // Reset device and set configuration
  i2cWrite(MAGNET,QMC5883L_RESET,0x01);
  i2cWrite(MAGNET,QMC5883L_CONFIG,oversampling|range|rate|mode);  
}

void readMagnet(int16_t *data){
  readMagnet(data, data+1, data+2);;
}

void readMagnet(int16_t *x, int16_t *y, int16_t *z){
  uint8_t buff[M_TO_READ];
  int regAddress = QMC5883L_X_LSB;
  i2cRead(MAGNET, regAddress, M_TO_READ, buff); // read the gyro data from the ITG3200
  *x = (buff[1] << 8) | buff[0];
  *y = (buff[3] << 8) | buff[2];
  *z = (buff[5] << 8) | buff[4];
}

/******** Barometer ********/
void initBarometer(){
	i2cWrite(BARO, SPL06_PRS_CFG, 0x71);	// Pressure single sample
	i2cWrite(BARO, SPL06_TMP_CFG, SPL06_TMP_RATE_128
	                            | SPL06_TMP_EXT);	      // Temperature single sample with external temp
	i2cWrite(BARO, SPL06_MEAS_CFG, 0x07);	              // continuous temp and pressure measurement
	i2cWrite(BARO, SPL06_CFG_REG, 0x04);	              // FIFO Pressure measurement  
}

void readBaro(int32_t *data) {
  uint8_t buff[B_TO_READ];
  int regAddress = SPL06_PRS_B2;
  i2cRead(BARO, regAddress, B_TO_READ, buff); 

  // Read in and format pressure data to two's complement
  data[0] = (buff[0] << 8) | buff[1];
  data[0] = data[0] << 8 | buff[2];

  if(data[0] & (1 << 23))
    data[0] = data[0] | 0XFF000000;   

  // Read in and format temperature data to two's complement
  data[1] = (buff[3] << 8) | buff[4];
  data[1] = (data[1] << 8) | buff[5];

  if(data[1] & (1 << 23))
    data[1] = data[1] | 0xFF000000;   
}

/******** i2c ********/
void i2cWrite(int device, int address, int value) {
  Wire.beginTransmission(device);   // start transmission 
  Wire.write(address);              // send register address
  Wire.write(value);                // send value to write
  Wire.endTransmission(); 
}

void i2cRead(int device, int address, int bytes, char* buff) {
  Wire.beginTransmission(device);   // start transmission 
  Wire.write(address);              // sends address to read from
  Wire.endTransmission(false);      // end transmission

  Wire.requestFrom(device, bytes);  // request bytes from device

  // Read each available byte to buffer
  int i = 0;
  while(Wire.available()) { 
   buff[i] = Wire.read(); 
   i++;
  }
  Wire.endTransmission(); 
}
