#ifndef __AVIONICS_H
#define __AVIONICS_H
#include <stdint.h>

/******** GYRO ********/

#define GYRO 0x68  //  when AD0 is connected to GND ,gyro address is 0x68.
//#define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8  // 2 bytes for each axis x, y, z
#define G_SENSITIVITY 14.375
                     
void initGyro();
void readGyro(int16_t *data);
void readGyro(int16_t *x, int16_t *y, int16_t *z);

/******** Accelerometer ********/

#define ACCEL (0x53)           // ADXL345 device address
#define A_TO_READ (6)          // num of bytes we are going to read each time (two bytes for each axis)
#define A_SENSITIVITY ADXL345_SENSITIVITY_16

#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110
                            //
// DATA_FORMAT register bits
#define ADXL345_DATA_FORMAT_RANGE_16 0x03
#define ADXL345_SENSITIVITY_16 0.0312

// POWER_CTL register bits
#define ADXL345_POWER_CTL_LINK 0x20
#define ADXL345_POWER_CTL_AUTOSLEEP 0x10
#define ADXL345_POWER_CTL_MEASURE 0x08
#define ADXL345_POWER_CTL_SLEEP 0x04

// INT_ENABLE register bits
#define ADXL345_INT_ENABLE_ACTIVITY 0x10
#define ADXL345_INT_ENABLE_INACTIVITY 0x08
#define ADXL345_INT_ENABLE_FREEFALL 0x04

/* 
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/*Interrupt bit position*/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

#define ADXL345_DATA_READY 0x07
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05
#define ADXL345_ACTIVITY   0x04
#define ADXL345_INACTIVITY 0x03
#define ADXL345_FREE_FALL  0x02
#define ADXL345_WATERMARK  0x01
#define ADXL345_OVERRUNY   0x00

#define ADXL345_OK    1 // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR   0 // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG    2 // bad method argument
                             //
void initAccel();
void readAccel(int16_t *data);
void readAccel(int16_t *x, int16_t *y, int16_t *z);

/******** Magnetometer ********/

/* The default I2C address of this chip */
#define MAGNET 0x0D
#define M_TO_READ 6

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

void initMagnet();
void readMagnet(int16_t *data);
void readMagnet(int16_t *x, int16_t *y, int16_t *z);

/******** Barometer ********/

#define BARO 0x76
#define B_TO_READ 6

#define SPL06_PRS_CFG 0x06
#define SPL06_TMP_CFG 0x07
#define SPL06_MEAS_CFG 0x08
#define SPL06_CFG_REG 0x09

#define SPL06_PRS_B2 0x00
#define SPL06_PRS_B1 0x01
#define SPL06_PRS_B0 0x02

#define SPL06_TMP_B2 0x03
#define SPL06_TMP_B1 0x04
#define SPL06_TMP_B0 0x05

#define SPL06_PRS_RATE_128 0b01110000
#define SPL06_TMP_RATE_128 0b01110000
#define SPL06_TMP_EXT      0b10000000

void initBarometer();
void readBaro(int32_t *data);
void readPressure(int32_t *pressure);
void readTemp(int32_t *temp);

/******** i2c ********/
void i2cWrite(int device, int address, int value);
void i2cRead(int device, int address, int bytes, char *buff);

#endif
