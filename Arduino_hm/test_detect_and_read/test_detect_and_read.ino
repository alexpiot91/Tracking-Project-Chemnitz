/** Credits:
 *  - Kris Winer
 *  - https://forum.hobbycomponents.com/viewtopic.php?f=73&t=1956
 *  - http://arduinolearning.com/code/arduino-mpu-9250-example.php
 *  - https://playground.arduino.cc/Main/I2cScanner/
 *  - https://github.com/kriswiner/MPU9250/issues/306 (calibration acc and gyro offsets)
 */


/** ||Hardware setup||
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
**/


/**  ||Get digital low-pass filter configuration||
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
**/


/**  ||Registers management||
 *   Scan only valid addresses (8 to 0x7B)
 *   0x00 - Reserved - General Call Address
 *   0x01 - Reserved for CBUS Compatibility
 *   0x02 - Reserved for I2C-compatible Bus Variants
 *   0x03 - Reserved for Future Use
 *   0x04, 0x05, 0x06, 0x07 - Reserved for Hs-mode Master
 *   0x78 0x79 0x7A 0x7B - Reserved for 10-bit I2C Addressing
 *   x7C 0x7D 0x7E 0x7F - Reserved for Future Purposes
 */


#include <Wire.h>
//#include <TimerOne.h>

 
#define MPU9250_ADDRESS    0x68
#define AK8963_ADDRESS     0x0C

// Magnetometer Registers
#define INFO               0x01
#define AK8963_ST1         0x02  // data ready status bit 0
#define AK8963_XOUT_L      0x03  // data
#define AK8963_XOUT_H      0x04
#define AK8963_YOUT_L      0x05
#define AK8963_YOUT_H      0x06
#define AK8963_ZOUT_L      0x07
#define AK8963_ZOUT_H      0x08
#define AK8963_ST2         0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL        0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC        0x0C  // Self test control
#define AK8963_I2CDIS      0x0F  // I2C disable
#define AK8963_ASAX        0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY        0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ        0x12  // Fuse ROM z-axis sensitivity adjustment value


// Other registers
#define SELF_TEST_X_GYRO   0x00                  
#define SELF_TEST_Y_GYRO   0x01                                                                          
#define SELF_TEST_Z_GYRO   0x02

#define SELF_TEST_X_ACCEL  0x0D
#define SELF_TEST_Y_ACCEL  0x0E    
#define SELF_TEST_Z_ACCEL  0x0F

#define SELF_TEST_A        0x10

#define XG_OFFSET_H        0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L        0x14
#define YG_OFFSET_H        0x15
#define YG_OFFSET_L        0x16
#define ZG_OFFSET_H        0x17
#define ZG_OFFSET_L        0x18
#define SMPLRT_DIV         0x19  // Sample rate divider
#define CONFIG             0x1A  // Gyroscope built-in LPF
#define GYRO_CONFIG        0x1B  // Gyroscope scale configuration
#define ACCEL_CONFIG       0x1C  // Accelerometer scale configuration
#define ACCEL_CONFIG2      0x1D  // Accelerometer built-in LPF
#define LP_ACCEL_ODR       0x1E   
#define WOM_THR            0x1F   

#define MOT_DUR            0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR           0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR          0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24   
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71 
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250   0x75  // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7B
#define YA_OFFSET_L        0x7C
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E 

// Choice of the data range for gyroscope on bits 3 and 4: register 27 (0x1B)
#define GYRO_FULL_SCALE_250_DPS 0x00   // 00
#define GYRO_FULL_SCALE_500_DPS 0x08   // 01
#define GYRO_FULL_SCALE_1000_DPS 0x10  // 10
#define GYRO_FULL_SCALE_2000_DPS 0x18  // 11

// Choice of the data range for accelerometer on bits 3 and 4: register 28 (0x1C) 
#define ACC_FULL_SCALE_2_G 0x00        // 00
#define ACC_FULL_SCALE_4_G 0x08        // 01
#define ACC_FULL_SCALE_8_G 0x10        // 10
#define ACC_FULL_SCALE_16_G 0x18       // 11

// Choice for magnetometer parameters
#define MAG_RES_14 0x00    // 14 bits magnetometer resolution
#define MAG_RES_16 0x01    // 16 bits  magnetometer resolution
#define MAG_FREQ_8 0x02    // 8Hz continuous magnetometer data read
#define MAG_FREQ_100 0x06  // 100Hz continuous magnetometer data read


// Dashboard
uint8_t Gscale = GYRO_FULL_SCALE_500_DPS;  // Choose gyroscope full scale
uint8_t Ascale = ACC_FULL_SCALE_4_G;       // Choose accelerometer full scale
uint8_t Mscale = MAG_RES_16;               // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = MAG_FREQ_100;              // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;                    // scale resolutions per LSB for the sensors
int prec = 2;                              // precision of transmitted data

// Initial parameters
float response[6];                                                                    // holds results of gyro and accelerometer self test
float listCalibMAG[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};   // factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};                              // bias corrections for gyro and accelerometer
int16_t accelData[3], gyroData[3], magData[3];
float ax, ay, az, gx, gy, gz, mx, my, mz;
unsigned long ti;


void scanner()
{
  // Scan only valid addresses (8 to 0x7B)
  // 0x00 - Reserved - General Call Address
  // 0x01 - Reserved for CBUS Compatibility
  // 0x02 - Reserved for I2C-compatible Bus Variants
  // 0x03 - Reserved for Future Use
  // 0x04, 0x05, 0x06, 0x07 - Reserved for Hs-mode Master
  // 0x78 0x79 0x7A 0x7B - Reserved for 10-bit I2C Addressing
  // 0x7C 0x7D 0x7E 0x7F - Reserved for Future Purposes
    
  int result;
  byte devices = 0; // for consistency of Arduino programming style, the byte data type is to be preferred rather than unsigned char.
  byte ad;

  for (ad=0;ad<0x7C;ad++){
    if (ad>7){ 
      // skip address from 0 to 7
      Wire.beginTransmission(ad);           // start transmission
      delay(10);
      result = Wire.endTransmission();      // end transmission and store answer
      // 0:success
      // 1:data too long to fit in transmit buffer
      // 2:received NACK on transmit of address
      // 3:received NACK on transmit of data
      // 4:other error
      if (!result){
        devices++;                          // operator returns either 0 or 1, depending on whether the input is non-zero or 0 respectively. Add a device to the count if ACK
        Serial.print(ad<16?"0x0":"0x");
        Serial.println(ad,HEX);
        }
      }
    }
  Serial.print(devices, DEC);
  Serial.print(" device");
  Serial.print(devices>1?"s":"");
  Serial.println(" found on the bus");
}


void selfTest(float * destination)
// Accelerometer and gyroscope self test; check calibration with regards to factory settings
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;
   
  I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  I2CwriteByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);  // Set full scale range for the gyro to 250 dps
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_2_G); // Set full scale range for the accelerometer to 2 g
  delay(25);  // Delay a while to let the device stabilize

  // Get average current values of gyro and acclerometer
  for( int ii = 0; ii < 200; ii++) 
  {  
    I2CreadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);    // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    
    I2CreadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);     // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++) 
  {  
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
   }
  
// Configure the accelerometer for self-test
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  // Get average self-test values of gyro and acclerometer
  for( int ii = 0; ii < 200; ii++) 
  {  
    I2CreadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    
    I2CreadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++) 
  {  
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
    }   
  
// Configure the gyro and accelerometer for normal operation
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
  delay(25);  // Delay a while to let the device stabilize
   
  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = I2CreadByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) 
  {
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
    }
}


void calibrateMPU(float * dest1, float * dest2)
// Function that accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
{  
  uint8_t data[12]; // data array to hold accelerometer x, y, z and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // Reset device is done in register 107 (0x6B)
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit (1000 0000)
  delay(100);
   
 // Get stable time source is done in register 107 (0x6B)
 // else use the internal oscillator, bits 2:0 = 001
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Write a one to bit 1 (0000 0001) to auto select the best available clock source
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);  // Set on accelerometer x, y, z and gyroscope x, y, z
  delay(200);                                    

// Configure device for bias calculation
  I2CwriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  I2CwriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure gyroscope and accelerometer for bias calculation
  I2CwriteByte(MPU9250_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec at gyro full scale GFS_SEL = 00 (250dps)
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g at acc full scale AFS_SEL = 00 (2g)

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  I2CreadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12; // How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    I2CreadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  I2CreadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  I2CreadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  I2CreadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

//// At end of sample accumulation, turn off FIFO sensor read
//  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
//  I2CreadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);  // Read FIFO sample count
//  fifo_count = ((uint16_t)data[0] << 8) | data[1];
//  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
//  for (ii = 0; ii < packet_count; ii++)
//  {
//    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//    I2CreadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // Read data for averaging
//    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
//    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
//    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
//    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
//    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
//    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
//    
//    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//    accel_bias[1] += (int32_t) accel_temp[1];
//    accel_bias[2] += (int32_t) accel_temp[2];
//    gyro_bias[0]  += (int32_t) gyro_temp[0];
//    gyro_bias[1]  += (int32_t) gyro_temp[1];
//    gyro_bias[2]  += (int32_t) gyro_temp[2];
//    }
//    
//    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
//    accel_bias[1] /= (int32_t) packet_count;
//    accel_bias[2] /= (int32_t) packet_count;
//    gyro_bias[0]  /= (int32_t) packet_count;
//    gyro_bias[1]  /= (int32_t) packet_count;
//    gyro_bias[2]  /= (int32_t) packet_count;
//    
//  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
//  else {accel_bias[2] += (int32_t) accelsensitivity;}
//   
//// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//  data[3] = (-gyro_bias[1]/4)       & 0xFF;
//  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//  data[5] = (-gyro_bias[2]/4)       & 0xFF;
//  
//// Push gyro biases to hardware registers
//  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
//  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
//  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
//  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
//  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
//  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
//  
//// Output scaled gyro biases for display in the main program
//  dest1[0] = (float)gyro_bias[0]/(float)gyrosensitivity;  
//  dest1[1] = (float)gyro_bias[1]/(float)gyrosensitivity;
//  dest1[2] = (float)gyro_bias[2]/(float)gyrosensitivity;
//
//// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//// the accelerometer biases calculated above must be divided by 8.
//  int16_t accel_bias_reg[3] = { 0, 0, 0 };   // A place to hold the factory accelerometer trim biases
//  int16_t mask_bit[3] = { 1, 1, 1 };         // Define array to hold mask bit for each accelerometer bias axis
//  I2CreadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);   // Read factory accelerometer trim values
//  accel_bias_reg[0] = ((int16_t)data[0] << 8) | data[1];
//  I2CreadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[1] = ((int16_t)data[0] << 8) | data[1];
//  I2CreadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[2] = ((int16_t)data[0] << 8) | data[1];
//
//  // Construct total accelerometer bias, including calculated average accelerometer bias from above
//  for (int i = 0; i < 3; i++)
//  {
//    if (accel_bias_reg[i] % 2)
//    {
//      mask_bit[i] = 0;
//      }
//    accel_bias_reg[i] -= accel_bias[i] >> 3; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
//    if (mask_bit[i])
//    {
//      accel_bias_reg[i] = accel_bias_reg[i] & ~mask_bit[i]; // Preserve temperature compensation bit
//      } 
//    else 
//    {
//      accel_bias_reg[i] = accel_bias_reg[i] | 0x0001; // Preserve temperature compensation bit
//     }
//  }
//
//  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//  data[1] = (accel_bias_reg[0]) & 0xFF;
//  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//  data[3] = (accel_bias_reg[1]) & 0xFF;
//  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//  data[5] = (accel_bias_reg[2]) & 0xFF;
//
//// IT DOES NOT WORK
////  // Push accelerometer biases to hardware registers
//  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
//
//// Output scaled accelerometer biases for display in the main program
//   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
//   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity; 
//}


void initMPU()
// Procedure:
// - read previous settings
// - clear everything except reserved bits
// - write new settings
{
  // wake up device
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  // clear sleep mode bit (6), enable all sensors 
  delay(100); // wait for all registers to reset 

  // get stable time source
  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  delay(200); 
   
  // Set accelerometers low pass filter at 5Hz and sample rate at 1kHz. Accelerometer built-in LPF is controlled in register 29 (0x1D)
  uint8_t b = I2CreadByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  // ! bit [4:7] are reserved !
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, b & ~0x0F);    // Clear accel_fchoice_b bit [3] and A_DLPFG bits [2:0]  
  I2CwriteByte(MPU9250_ADDRESS,ACCEL_CONFIG2, 0x06);      // Set accelerometer rate to 1 kHz and bandwidth to 5 Hz
  //I2CwriteByte(MPU9250_ADDRESS,ACCEL_CONFIG2, (0xF0 & b) | 0x0E);      // Set accelerometer to normal mode and rate to 1 kHz and bandwidth to 5 Hz ((11110000 & b)|1110))
  
  // Set gyroscope low pass filter bw = 5Hz and sample rate at 1kHz. Gyroscope built-in LPF is controlled in register 26 (0x1A)
  uint8_t a = I2CreadByte(MPU9250_ADDRESS, CONFIG);
  // ! bit [7] is reserved !
  I2CwriteByte(MPU9250_ADDRESS,CONFIG,0x06); 
  //I2CwriteByte(MPU9250_ADDRESS, CONFIG, (0x40 & a) | 0x06);  //((10000000 & a)|110)
     
  // Configure gyroscope scale. Scale selection is done in register 27 (0x1B)
  uint8_t c = I2CreadByte(MPU9250_ADDRESS, GYRO_CONFIG);    // Set gyroscope rate to 1 kHz and bandwidth to 5 Hz
  // ! bit [2] is reserved !
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0);          // Clear self-test bits [7:5] 
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02);          // Clear Fchoice bits [1:0] 
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18);          // Clear AFS bits [4:3]
  //I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, Gscale);    // Set full scale range for the gyro
  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, (0x04 & c) | Gscale);    // Set full scale range for the gyro ((100 & c)|1000)
  
  // Configure accelerometer scale. Scale selection is done in register 28 (0x1C)
  uint8_t d = I2CreadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  // ! bit [2:0] are reserved !
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, d & ~0xE0);         // Clear self-test bits [7:5] 
  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, d & ~0x18);         // Clear AFS bits [4:3]
  //I2CwriteByte(MPU9250_ADDRESS,ACCEL_CONFIG, Ascale);    // Set full scale range for the accelerometer
  I2CwriteByte(MPU9250_ADDRESS,ACCEL_CONFIG, (0x07 & d) | Ascale);    // Set full scale range for the accelerometer ((111 & d)|1000)

  // Divides the internal sample rate (done in register 25=0x19) to generate the sample rate that controls sensor data output rate, FIFO sample rate.
  I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) = 200Hz
  
  // The accelerometer and gyro are set to 1 kHz sample rates, but all these rates
  // are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  
  // Set by pass mode (done in register 55=0x37).
  I2CwriteByte(MPU9250_ADDRESS,INT_PIN_CFG,0x22);
    // 0x02: When asserted, the i2c_master interface pins (ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c master interface is disabled. 
    // 0x20: INT pin level held until interrupt status is cleared.
  I2CwriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}


void initMAG(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x, y, z gyro calibration data stored here
  I2CwriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(100);
  I2CwriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(100);
  I2CreadBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-axis, y-axis, and z-axis calibration values
  
  // Return x-axis, y-axis, and z-axis sensitivity adjustment values (formula in datasheet)
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  I2CwriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(100);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  I2CwriteByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void calibrateMAG(float * dest1, float * dest2) 
 {
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  long mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF};
  int16_t mag_temp[3] = {0, 0, 0};
  
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  
  sample_count = 200;
  for(ii = 0; ii < sample_count; ii++)
  {
    readMag(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++)
    {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
    delay(50);  // at 8(100) Hz ODR, new mag data is available every 125(10) ms
    }
  
  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
  dest1[0] = (float) mag_bias[0]*mRes*listCalibMAG[0];  // save mag biases in µT for main program
  dest1[1] = (float) mag_bias[1]*mRes*listCalibMAG[1];   
  dest1[2] = (float) mag_bias[2]*mRes*listCalibMAG[2];  
  
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  
  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
  
  Serial.println("Mag Calibration done!");
 }
  

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case ACC_FULL_SCALE_2_G:
          aRes = 2.0/32768.0;
          break;
    case ACC_FULL_SCALE_4_G:
          aRes = 4.0/32768.0;
          break;
    case ACC_FULL_SCALE_8_G:
          aRes = 8.0/32768.0;
          break;
    case ACC_FULL_SCALE_16_G:
          aRes = 16.0/32768.0;
          break;
  }
}


void getGres()
// Possible gyro scales (and their register bit settings) are:
// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
{
  switch (Gscale)
  {
    case GYRO_FULL_SCALE_250_DPS:
          gRes = 250.0/32768.0;
          break;
    case GYRO_FULL_SCALE_500_DPS:
          gRes = 500.0/32768.0;
          break;
    case GYRO_FULL_SCALE_1000_DPS:
          gRes = 1000.0/32768.0;
          break;
    case GYRO_FULL_SCALE_2000_DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}


void getMres()
// Possible magnetometer scales (and their register bit settings) are
// 14 bit resolution (0) and 16 bit resolution (1)
{
  switch (Mscale)
  {
    case MAG_RES_14:
          mRes = 10*4912./8190.; // Proper scale to return mG (1G=100µT)
          break;
    case MAG_RES_16:
          mRes = 10*4912./32760.0; // Proper scale to return mG (1G=100µT)
          break;
  }
}


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2CreadBytes(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
   
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available()){
    Data[index++]=Wire.read();
    }
}


uint8_t I2CreadByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}


void readMag(int16_t * destination)
{ 
  uint8_t rawData[7];  // x, y, z magnetometer data and register ST2
  if(I2CreadByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)  // Wait for magnetometer data ready bit DRDY to be set (=1)
  { 
    I2CreadBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08))  // Check if magnetic sensor overflow set (bit [3] HOFL of ST2 register = 1 if overflow), if not then report data
    { 
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    }
  }
}


void readData(int16_t * destination, int16_t reg)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  I2CreadBytes(MPU9250_ADDRESS, reg, 6, rawData);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
} 

 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
  }

 
void setup()
{
  // Arduino initialization
  Wire.begin();
  Serial.begin(115200);
  delay(250);

  // Scanning procedure
  Serial.println("---Scanning---");
  scanner();
  Serial.println();
  delay(500);

  // Self test procedure
  Serial.println("---Self Test---");
  Serial.println("Put the device on a flat surface and do not move it");
  delay(500);
  selfTest(response); // Start by performing self test and reporting values
  Serial.print("x-axis self test: acceleration response within : "); Serial.print(response[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration response within : "); Serial.print(response[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration response within : "); Serial.print(response[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration response within : "); Serial.print(response[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration response within : "); Serial.print(response[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration response within : "); Serial.print(response[5],1); Serial.println("% of factory value");
  Serial.println();
  delay(500);

  // Calibration
//  Serial.println("---Accelerometer and Gyroscope Calibration---");
//  calibrateMPU(gyroBias, accelBias); // Calibrate gyroscope and accelerometer, load biases in bias registers
//  Serial.print("AccXBias = "); Serial.print((int)(1000*accelBias[0])); Serial.println("mg");
//  Serial.print("AccYBias = "); Serial.print((int)(1000*accelBias[1])); Serial.println("mg");
//  Serial.print("AccZBias = "); Serial.print((int)(1000*accelBias[2])); Serial.println("mg");
//  Serial.print("GyrXBias = "); Serial.print(gyroBias[0], 1); Serial.println("o/s");
//  Serial.print("GyrYBias = "); Serial.print(gyroBias[1], 1); Serial.println("o/s");
//  Serial.print("GyrZBias = "); Serial.print(gyroBias[2], 1); Serial.println("o/s");
//  Serial.println();
//  delay(500);

  // Initialization procedures
  getAres();
  getGres();
  getMres();
  
  Serial.println("---Accelerometer and Gyroscope Initialization ---");
  initMPU();
  Serial.println();
  delay(500);

  Serial.println("---Magnetometer Initialization ---");
  initMAG(listCalibMAG);
  // calibrateMAG(magBias,magScale); 
    
  Serial.println("Magnetometer sensitivity adjustement values (µT):");
  Serial.print("X = "); Serial.println(listCalibMAG[0], 2);
  Serial.print("Y = "); Serial.println(listCalibMAG[1], 2);
  Serial.print("Z = "); Serial.println(listCalibMAG[2], 2);
//  Serial.println("Magnetometer bias:");
//  Serial.print("X = "); Serial.println(magBias[0], 2);
//  Serial.print("Y = "); Serial.println(magBias[1], 2);
//  Serial.print("Z = "); Serial.println(magBias[2], 2);
//  Serial.println("Magnetometer scale:");
//  Serial.print("X = "); Serial.println(magScale[0], 2);
//  Serial.print("Y = "); Serial.println(magScale[1], 2);
//  Serial.print("Z = "); Serial.println(magScale[2], 2);
  Serial.println("GO");
  delay(500);  
}


void loop()
{  
  // Read accelerometer and gyroscope
  if (I2CreadByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
  readData(accelData,ACCEL_XOUT_H);
  ax = (float)accelData[0]*aRes; 
  ay = (float)accelData[1]*aRes;  
  az = (float)accelData[2]*aRes; 
  readData(gyroData,GYRO_XOUT_H);
  gx = (float)gyroData[0]*gRes;
  gy = (float)gyroData[1]*gRes;  
  gz = (float)gyroData[2]*gRes;  
  
  // Read magnetometer (corrected values with sensitivity
  readMag(magData);
  mx = (float)magData[0]*mRes*listCalibMAG[0]; //- magBias[1];  // get actual magnetometer value, this depends on scale being set
  my = (float)magData[1]*mRes*listCalibMAG[1]; //- magBias[0];  
  mz = (float)magData[2]*mRes*listCalibMAG[2]; //- magBias[2];
  }

  // Display values
  // Accelerometer
  Serial.print (ax,prec); 
  Serial.print (",");
  Serial.print (ay,prec);
  Serial.print (",");
  Serial.print (az,prec); 
  Serial.print (";");
   
  // Gyroscope
  Serial.print (gx,prec); 
  Serial.print (",");
  Serial.print (gy,prec);
  Serial.print (",");
  Serial.print (gz,prec); 
  Serial.print (";");

  // Magnetometer
  Serial.print (mx,prec); 
  Serial.print (",");
  Serial.print (my,prec);
  Serial.print (",");
  Serial.print (mz,prec); 
  //Serial.print ("\t");    

  // End of line
  Serial.println("");
  }
