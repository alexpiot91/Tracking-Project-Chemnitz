/*
  Send command to reset module over I2C
  By: Nathan Seidle
  Date: January 29rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to reset the U-Blox module to factory defaults over I2C.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Connect the U-Blox serial port to Serial1
  If you're using an Uno or don't have a 2nd serial port (Serial1), consider using software serial
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
      Wire.beginTransmission(ad);          // start transmission
      delay(10);
      result = Wire.endTransmission();     // end transmission and store answer
      // 0:success
      // 1:data too long to fit in transmit buffer
      // 2:received NACK on transmit of address
      // 3:received NACK on transmit of data
      // 4:other error
      if (!result){
        devices++;                         // operator returns either 0 or 1, depending on whether the input is non-zero or 0 respectively. Add a device to the count if ACK
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  // Communication initialization
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");
  Wire.begin();
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // I2C scanning procedure
  Serial.println(F("---Scan---"));
  scanner();
  Serial.println();

  // Prepare for reset and reset
  while (Serial.available()) Serial.read(); //Trash any incoming chars
  Serial.println("Press a key to reset module to factory defaults");
  while (Serial.available() == false) ; //Wait for user to send character

  myGPS.factoryReset(); //Reset everything: baud rate, I2C address, update rate, everything.

  if (myGPS.begin() == false) //Attempt to re-connect
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  Serial.println("Unit has now been factory reset. Freezing...");
  while(1);
}


void loop()
{

}
