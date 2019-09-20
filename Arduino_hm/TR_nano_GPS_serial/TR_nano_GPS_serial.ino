/**  ========== Light operating file for the transmitter ==========
 * In order to make this operating file lighter on the Arduino Nano, it contains 
 * the operations that do not need any manipulation of the registers and/or the 
 * operations that calculate and send values to the receiver :
 *  - RF initialization
 *  - magnetometer initialization (calculation of its sensitivity adjustement values)
 *  - get the full scale parameters written in the dedicated registers by the init file
 *  - send the mag sensitivity and the full scale parameters to the receiver
 *  - send raw IMU and GPS data
 */
 

/**  ||Credits||
 *  - Kris Winer
 *  - https://forum.hobbycomponents.com/viewtopic.php?f=73&t=1956
 *  - http://arduinolearning.com/code/arduino-mpu-9250-example.php
 *  - https://playground.arduino.cc/Main/I2cScanner/
 *  - https://github.com/kriswiner/MPU9250/issues/306 (calibration acc and gyro offsets)
 */


/**  ||I2C connections||
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */


/**  ||Built-in low-pass filter configuration||
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>10
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
 */


/**  ||IMU registers management||
 *   Scan only valid addresses (8 to 0x7B)
 *   0x00 - Reserved - General Call Address
 *   0x01 - Reserved for CBUS Compatibility
 *   0x02 - Reserved for I2C-compatible Bus Variants
 *   0x03 - Reserved for Future Use
 *   0x04, 0x05, 0x06, 0x07 - Reserved for Hs-mode Master
 *   0x78 0x79 0x7A 0x7B - Reserved for 10-bit I2C Addressing
 *   x7C 0x7D 0x7E 0x7F - Reserved for Future Purposes
 */


/**  ||RF transmission : TRANSMITTER||
 *   3.3V
 *   GND
 *   CE ---------------------- D9
 *   CSN --------------------- D10
 *   MOSI -------------------- D11
 *   MISO -------------------- D12
 *   SCK --------------------- D13
 *   IRQ --------------------- disconnected
 */

/**  ||I2C connections||
 GPS ---------------------- Arduino
 GND ---------------------- GND
 3.3V --------------------- 3.3V
 TX ----------------------- D4
 RX ----------------------- D3
 */
 

//Include Libraries
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

// Registers definition
#define MPU9250_ADDRESS    0x68
#define AK8963_ADDRESS     0x0C

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
#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24   
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

// Initial parameters
float response[6];                                          // holds results of gyro and accelerometer self test
int16_t accelData[3], gyroData[3], magData[3];
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};    // bias corrections for gyro and accelerometer
float listSensMag[3] = {0, 0, 0};                           // factory mag calibration and mag bias

// Dashboard
uint8_t Gscale = GYRO_FULL_SCALE_500_DPS;  // Choose gyroscope full scale
uint8_t Ascale = ACC_FULL_SCALE_4_G;       // Choose accelerometer full scale
uint8_t Mscale = MAG_RES_16;               // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = MAG_FREQ_100;              // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
int aResOut, gResOut, mResOut;             // scale resolutions per LSB for the sensors

// Create an RF24 object and parameters
RF24 radio(9, 10);  // CE, CSN
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };     // radio pipe addresses for the 2 nodes to communicate.

// Create a GPS object and parameters                 
#ifndef NMEAGPS_INTERRUPT_PROCESSING
  #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif
static NMEAGPS   gps;
static gps_fix  fix_data;
float lat, lng;
int lat_int, lat_dec, lng_int, lng_dec, vel_int, h_int;
long lastTime1 = 0;
long lastTime2 = 0;
long startTime = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
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
  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
  I2CreadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);  // Read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    I2CreadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // Read data for averaging
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
  dest1[0] = (float)gyro_bias[0]/(float)gyrosensitivity;  
  dest1[1] = (float)gyro_bias[1]/(float)gyrosensitivity;
  dest1[2] = (float)gyro_bias[2]/(float)gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.
  int16_t accel_bias_reg[3] = { 0, 0, 0 };   // A place to hold the factory accelerometer trim biases
  int16_t mask_bit[3] = { 1, 1, 1 };         // Define array to hold mask bit for each accelerometer bias axis
  I2CreadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);   // Read factory accelerometer trim values
  accel_bias_reg[0] = ((int16_t)data[0] << 8) | data[1];
  I2CreadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = ((int16_t)data[0] << 8) | data[1];
  I2CreadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = ((int16_t)data[0] << 8) | data[1];

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  for (int i = 0; i < 3; i++)
  {
    if (accel_bias_reg[i] % 2)
    {
      mask_bit[i] = 0;
      }
    accel_bias_reg[i] -= accel_bias[i] >> 3; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    if (mask_bit[i])
    {
      accel_bias_reg[i] = accel_bias_reg[i] & ~mask_bit[i]; // Preserve temperature compensation bit
      } 
    else 
    {
      accel_bias_reg[i] = accel_bias_reg[i] | 0x0001; // Preserve temperature compensation bit
     }
  }

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;

// IT DOES NOT WORK
//  // Push accelerometer biases to hardware registers
//  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity; 
}


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
  
  // The accelerometer and gyro are set to 1 kHz sample rates, but these rates
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


void getAres()
// Possible accelerometer scales (and their register bit settings) are:
// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).  
{ 
  byte a = I2CreadByte(MPU9250_ADDRESS,ACCEL_CONFIG);
  switch (a)
  {
    case ACC_FULL_SCALE_2_G:
          aResOut = 2;
          break;
    case ACC_FULL_SCALE_4_G:
          aResOut = 4;
          break;
    case ACC_FULL_SCALE_8_G:
          aResOut = 8;
          break;
    case ACC_FULL_SCALE_16_G:
          aResOut = 16;
          break;
  }
}


void getGres()
// Possible gyro scales (and their register bit settings) are:
// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
{
  byte b = I2CreadByte(MPU9250_ADDRESS, GYRO_CONFIG);
  switch (b)
  {
    case GYRO_FULL_SCALE_250_DPS:
          gResOut = 250;
          break;
    case GYRO_FULL_SCALE_500_DPS:
          gResOut = 500;
          break;
    case GYRO_FULL_SCALE_1000_DPS:
          gResOut = 1000;
          break;
    case GYRO_FULL_SCALE_2000_DPS:
          gResOut = 2000;
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
          mResOut = 14; // Proper scale to return G (1G=100µT)
          break;
    case MAG_RES_16:
          mResOut = 16; // Proper scale to return G (1G=100µT)
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


void coordIntFormat(int &leftInt, int &rightInt, float coord)
{
  float coord_temp = coord/100000.0;
  leftInt = floor(coord_temp);
  rightInt = (coord_temp-leftInt)*10000;
}


int speedIntFormat(float v)
{
  int v_int = floor((v/3.6)*100.0);
  return v_int;
}


int headIntFormat(float h)
{
  int v_int = floor(h*10.0);
  return v_int;
}


void sendRF(int val)
{
  readData(accelData,ACCEL_XOUT_H);
  readData(gyroData,GYRO_XOUT_H);
  readMag(magData);
  int16_t dataOut[15] = {accelData[0], accelData[1], accelData[2], gyroData[0], gyroData[1], gyroData[2], magData[0], magData[1], magData[2], lat_int*val, lat_dec*val, lng_int*val, lng_dec*val, h_int*val, vel_int*val};
  radio.write(&dataOut, sizeof(dataOut));
}


static void GPSisr( uint8_t c )
{
  gps.handle( c );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
void setup()
{
  // Initialization of communication
  NeoSerial.begin(9600);
  while (!NeoSerial)
  NeoSerial.println(F("---Serial,I2C and RF initialization---"));
  Wire.begin();
  NeoSerial.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  trace_header(NeoSerial);
  NeoSerial.flush();
  gpsPort.attachInterrupt( GPSisr );
  gpsPort.begin(9600);

  // RF communication parameters
  radio.begin();
  radio.setAutoAck(1);                    // ensure autoACK is enabled
  radio.setPALevel(RF24_PA_MAX);          // power amplifier level
  radio.enableAckPayload();               // allow optional ack payloads
  radio.setRetries(0,15);                 // smallest time between retries, max no. of retries
  radio.openWritingPipe(pipes[0]);        // both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();                  // stop listening
  NeoSerial.println();

  // Accelerometer-Gyroscope Calibration
  NeoSerial.println("---Accelerometer and Gyroscope Calibration---");
  calibrateMPU(gyroBias, accelBias); // Calibrate gyroscope and accelerometer, load biases in bias registers
  
  // Accelerometer-Gyroscope initialization
  NeoSerial.println(F("---Accelerometer and Gyroscope Initialization ---"));
  initMPU();
  
  // Magnetometer initialization
  initMAG(listSensMag);

  // Transmit calculation parameters (in the transmission file we work with integers,
  // resolutions and other factors are transmitted here and used in the receiver program) 
  NeoSerial.println(F("---Parameters RF transmission---"));
  getAres();
  getGres();
  getMres();
  byte counter = 0;
  int count = 0;
  byte oldByte = 0;
  int array2send[3] = {aResOut, gResOut, mResOut};
  while(count < 3){   
    NeoSerial.print(F("Sending: ")); NeoSerial.print((int)array2send[counter]);
    int data2send = array2send[counter];
    if (!radio.write( &data2send, sizeof(data2send) )){
      NeoSerial.println(F("failed."));      
    }
    else{
      if(!radio.available()){ 
        NeoSerial.println();
      }
      else{
        while(radio.available() ){
          byte ackByte;
          radio.read( &ackByte, 1 );
          NeoSerial.print(F(", Response: ")); NeoSerial.println(ackByte);
          if(ackByte != oldByte){
            oldByte = ackByte;
            count++;
            counter++;
          }
        }
      }
    }
    delay(100);
  }

  delay(500);
  NeoSerial.println(F("GO"));

  startTime = millis();
}


void loop()
{
  //if (I2CreadByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  if (millis() - lastTime2 > 5)  
  {
    
    if (gps.available()) 
    //if (millis() - lastTime1 > 100)
    {
      //lastTime1 = millis();  // Update the timer
      
      // collect GPS data
      fix_data = gps.read();

      // GPS output are long values so we need to cut them into two integers
      // that can be sent in our RF payload.
      coordIntFormat(lat_int, lat_dec, (float)fix_data.latitudeL());
      coordIntFormat(lng_int, lng_dec, (float)fix_data.longitudeL());
      vel_int = speedIntFormat((float)fix_data.speed_kph());
      h_int = headIntFormat((float)fix_data.heading());

      NeoSerial.println(h_int);
      //NeoSerial.print (",");
      //NeoSerial.println(vel_int);

      // send with GPS coordinates
      sendRF(1);
      }
    else
    {
      // send without GPS coordinates
      lastTime2 = millis();  // Update the timer
      sendRF(0);
      }
    }
}
