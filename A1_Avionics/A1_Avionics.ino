#include <SPL06-007.h> // Baro 
#include <Wire.h>
#include <QMC5883L.h> // Mag
#include <ADXL345.h> // Accel 
#include <SerialFlash.h>
#include <SPI.h>

QMC5883L compass;
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

// Gyroscope ITG3200 

#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
//#define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69  
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z

// Define the chip select 
#define CSPIN 10

// offsets are chip specific. 

int hx, hy, hz, turetemp;

double groundpressure;

void initGyro()
{
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
 writeTo(GYRO, G_PWR_MGM, 0x05);
 writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
 writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
 writeTo(GYRO, G_INT_CFG, 0x00);
}
void getGyroscopeData(int * result)
{
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
 int temp, x, y, z;
 byte buff[G_TO_READ];
 readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
 result[0] = ((buff[2] << 8) | buff[3]) + g_offx; // they are 2 byte registers within the sensor
 result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
 result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
 result[3] = (buff[0] << 8) | buff[1]; // temperature
 }

void setup() {
  Wire.begin();    // begin Wire(I2C)
  initGyro();
  Serial.begin(115200); // begin Serial
  SPL_init(); // Setup initial SPL chip registers   
  groundpressure = get_pcomp(); // provides initial ground pressure to reference
  compass.init();
  compass.setSamplingRate(50);


  //Set up SPI  
  if (!SerialFlash.begin(CSPIN)) {
    while (1) {
      Serial.println(F("Unable to access SPI Flash chip"));
      delay(1000);
    }
  }

  // not sure if we need this. 
  uint8_t id[5];
  SerialFlash.readID(id);



  //accel - we can go through if and delete a lot of these as they are to do with tap sensing
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
  //end of accel
  
}
  
  // Flash setupa address
  uint32_t pageaddr = 0; // Starting page address 

  // Timers for high and low res
  unsigned long previousHighResolutionMicro = 0;
  unsigned long previousLowResolutionMicro = 0;
  const unsigned long highResolutionInterval = 2000; // Interval for high resolution (500Hz) in microseconds (1000000 microseconds / 500Hz)
  const unsigned long lowResolutionInterval = 20000; // Interval for low resolution (50Hz) in microseconds (1000000 microseconds / 50Hz)
  
  // Timer for Scync 
  unsigned long previousMicros = 0;
  const unsigned long syncMicros = 249000; // 249ms interval in microseconds
  
  // Buffer size for storing sensor data
  const size_t BUFFER_SIZE = 256;  // 256 byte per pages  
  uint8_t dataBuffer[BUFFER_SIZE]; // Buffer to store sensor data
  uint8_t BufferIndex = 0; 


void loop() {
  
  // Current time in microseconds
  unsigned long currentMicro = micros(); 

   // Check if 249ms has elapsed for sync  
  if (currentMicro - previousMicros >= syncMicros) {
    // Reset the timer
    previousMicros = currentMicro;
  }

  // High resolution loop (500Hz)
  if (currentMicro - previousHighResolutionMicro >= highResolutionInterval) {
    
      //accel ---------------------------------------------------------------------------------------
      int16_t x,y,z;  
      adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

      // Cast to uint16_t
      uint16_t accel_x = (uint16_t)x;
      uint16_t accel_y = (uint16_t)y;
      uint16_t accel_z = (uint16_t)z;

     Serial.println("Accel: ");
     Serial.println(accel_x);
     Serial.println(accel_y);
     Serial.println(accel_z); 
      
      //gyro ---------------------------------------------------------------------------------------
      byte addr;
      int16_t gyro[4];
      getGyroscopeData(gyro);
      hx = gyro[0]; // 14.375; // this is the scale within the datasheet we will need to go through it again
      hy = gyro[1]; // 14.375;
      hz = gyro[2]; // 14.375;

      // Cast to uint16_t
      uint16_t hx = (uint16_t)gyro[0];
      uint16_t hy = (uint16_t)gyro[1];
      uint16_t hz = (uint16_t)gyro[2];

     Serial.println("Gyro: ");
     Serial.println(hx);
     Serial.println(hy);
     Serial.println(hz); 

      // Magnetometer/compass ------------------------------------------------------------------------
      int16_t mx,my,mz,mt; 
      compass.readRaw(&mx,&my,&mz,&mt); // reads the raw data not sure exactly what the scale is at all, we need comb the data sheet

      // Cast to uint16_t
      uint16_t magnetometer_x = (uint16_t)mx;
      uint16_t magnetometer_y = (uint16_t)my;
      uint16_t magnetometer_z = (uint16_t)mz;
      uint16_t magnetometer_t = (uint16_t)mt;

       Serial.println("mag: ");
       Serial.println(magnetometer_x);
       Serial.println(magnetometer_y);
       Serial.println(magnetometer_z); 
       Serial.println(magnetometer_t);
      
      // convert currentMicro from mirco to millis
      uint8_t syncMillis = currentMicro / 1000; // Convert microseconds to milliseconds and truncate
      
      // Create Buffer
      dataBuffer[BufferIndex++] = 0b01010100; // 01 010100 (ID and length)
      dataBuffer[BufferIndex++] = syncMillis; 
      
      dataBuffer[BufferIndex++] = (uint8_t)((accel_x >> 8) & 0xFF); // Store the high byte of accel_x
      dataBuffer[BufferIndex++] = (uint8_t)(accel_x & 0xFF); // Store the low byte of accel_x
      dataBuffer[BufferIndex++] = (uint8_t)((accel_y >> 8) & 0xFF); // Store the high byte of accel_y
      dataBuffer[BufferIndex++] = (uint8_t)(accel_y & 0xFF); // Store the low byte of accel_y
      dataBuffer[BufferIndex++] = (uint8_t)((accel_z >> 8) & 0xFF); // Store the high byte of accel_z
      dataBuffer[BufferIndex++] = (uint8_t)(accel_z & 0xFF); // Store the low byte of accel_z
      
      dataBuffer[BufferIndex++] = (uint8_t)((hx >> 8) & 0xFF); // Store the high byte of hx
      dataBuffer[BufferIndex++] = (uint8_t)(hx & 0xFF); // Store the low byte of hx
      dataBuffer[BufferIndex++] = (uint8_t)((hy >> 8) & 0xFF); // Store the high byte of hy
      dataBuffer[BufferIndex++] = (uint8_t)(hy & 0xFF); // Store the low byte of hy
      dataBuffer[BufferIndex++] = (uint8_t)((hz >> 8) & 0xFF); // Store the high byte of hz
      dataBuffer[BufferIndex++] = (uint8_t)(hz & 0xFF); // Store the low byte of hz
      
      dataBuffer[BufferIndex++] = (uint8_t)((magnetometer_x >> 8) & 0xFF); // Store the high byte of magnetometer_x
      dataBuffer[BufferIndex++] = (uint8_t)(magnetometer_x & 0xFF); // Store the low byte of magnetometer_x
      dataBuffer[BufferIndex++] = (uint8_t)((magnetometer_y >> 8) & 0xFF); // Store the high byte of magnetometer_y
      dataBuffer[BufferIndex++] = (uint8_t)(magnetometer_y & 0xFF); // Store the low byte of magnetometer_y
      dataBuffer[BufferIndex++] = (uint8_t)((magnetometer_z >> 8) & 0xFF); // Store the high byte of magnetometer_z
      dataBuffer[BufferIndex++] = (uint8_t)(magnetometer_z & 0xFF); // Store the low byte of magnetometer_z
      dataBuffer[BufferIndex++] = (uint8_t)((magnetometer_t >> 8) & 0xFF); // Store the high byte of magnetometer_t
      dataBuffer[BufferIndex++] = (uint8_t)(magnetometer_t & 0xFF); // Store the low byte of magnetometer_t

      // reset high res timer 
      previousHighResolutionMicro = currentMicro;
  }

  // Low resolution loop (50Hz)
  if (currentMicro - previousLowResolutionMicro >= lowResolutionInterval) {
    
    //Barometer ---------------------------------------------------------------------------------------------------

    int32_t praw = get_praw(); // get raw pressure data
    int32_t traw = get_traw(); // get raw temperature data 

    Serial.println("Baro: ");
    Serial.println(praw); 
    Serial.println(traw);
    
    // convert currentMicro from mirco to millis
    uint8_t syncMillis = currentMicro / 1000; // Convert microseconds to milliseconds and truncate
    
    // Shift to 3 bytes and add to Buffer
    dataBuffer[BufferIndex++] = 0b10001000; // 10 001000 (ID and length)
    dataBuffer[BufferIndex++] = syncMillis; 
    
    dataBuffer[BufferIndex++] = (uint8_t)((praw >> 16) & 0xFF); // Store the first byte of praw
    dataBuffer[BufferIndex++] = (uint8_t)((praw >> 8) & 0xFF); // Store the second byte of praw
    dataBuffer[BufferIndex++] = (uint8_t)(praw & 0xFF); // Store the third byte of praw
    
    dataBuffer[BufferIndex++] = (uint8_t)((traw >> 16) & 0xFF); // Store the first byte of traw
    dataBuffer[BufferIndex++] = (uint8_t)((traw >> 8) & 0xFF); // Store the second byte of traw
    dataBuffer[BufferIndex++] = (uint8_t)(traw & 0xFF); // Store the third byte of traw

    // Write to flash
    while (!SerialFlash.ready());
    SerialFlash.write(pageaddr, dataBuffer, sizeof(dataBuffer));

    Serial.println("DataBuffer: ");
    for (int i = 0; i < BUFFER_SIZE; ++i) {
    Serial.print(dataBuffer[i]);
    Serial.print(" ");
  }
      
    // increase addr to change page 
    pageaddr += 0x100; 

    // reset buffer index 
    BufferIndex = 0; 

    // reset low res timer 
    previousLowResolutionMicro = currentMicro;

    
  }

   uint8_t readdata[256]; 
   SerialFlash.read(pageaddr,readdata, sizeof(readdata)); 

  // print to serial
  Serial.println("data read back:");
    for (int i = 0; i < BUFFER_SIZE; ++i) {
    Serial.print(readdata[i]);
    Serial.print(" ");
  }
  
    while(1); 
}



void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}
//reads num bytes starting from address register on ACC in to buff array
 void readFrom(int DEVICE, byte address, int num, byte buff[]) {
 Wire.beginTransmission(DEVICE); //start transmission to ACC 
 Wire.write(address);        //sends address to read from
 Wire.endTransmission(); //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
 
 int i = 0;
 while(Wire.available())    //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read(); // receive a byte
   i++;
 }
 Wire.endTransmission(); //end transmission
}
