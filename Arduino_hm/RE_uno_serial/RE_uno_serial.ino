// === RECEIVER ===

// --connection--
//3.3V
//GROUND
//CE = 9
//SS (CSN) = 10
//MOSI = 11
//MISO = 12
//SCK = 13
//IRQ = DISCONNECTED


//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(9, 10);  // CE, CSN

//addresses through which two modules communicate.
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };     // radio pipe addresses for the 2 nodes to communicate.
const byte address[6] = "00001";


// Calculation parameters
float aResIn, gResIn, mResIn;
float aRes, gRes, mRes;
float listSensMAGX = 1.23047, listSensMAGY = 1.22656, listSensMAGZ = 1.18359;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void getAres() {
  switch ((int)aResIn)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case 2:
          aRes = 2.0/32768.0;
          break;
    case 4:
          aRes = 4.0/32768.0;
          break;
    case 8:
          aRes = 8.0/32768.0;
          break;
    case 16:
          aRes = 16.0/32768.0;
          break;
  }
}


void getGres()
// Possible gyro scales (and their register bit settings) are:
// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
{
  switch ((int)gResIn)
  {
    case 250:
          gRes = 250.0/32768.0;
          break;
    case 500:
          gRes = 500.0/32768.0;
          break;
    case 1000:
          gRes = 1000.0/32768.0;
          break;
    case 2000:
          gRes = 2000.0/32768.0;
          break;
  }
}


void getMres()
// Possible magnetometer scales (and their register bit settings) are
// 14 bit resolution (0) and 16 bit resolution (1)
{
  switch ((int)mResIn)
  {
    case 14:
          mRes = 10*4912./8190.; // Proper scale to return mG (1µT=0.01G)
          break;
    case 16:
          mRes = 10*4912./32760.0; // Proper scale to return mG (1µT=0.01G)
          break;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  // Initialization of communication
  Serial.begin(250000);
  radio.begin();

  // Communication parameters
  radio.setAutoAck(1);                    // ensure autoACK is enabled
  radio.setPALevel(RF24_PA_MAX);          // power amplifier level
  radio.enableAckPayload();               // allow optional ack payloads
  radio.setRetries(0,15);                 // smallest time between retries, max no. of retries
  radio.openWritingPipe(pipes[1]);        // both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();                 // start listening
  radio.printDetails();                   // dump the configuration of the rf unit for debugging
  

  // Receive calculation parameters
  int count = 0;
  int oldByte = 0;
  byte ackByte = 1;
  int array2receive[3];
  while(count < 3){
    byte pipeNo;
    int data2receive;                                       // Dump the payloads until we've gotten everything
    while( radio.available(&pipeNo)){
      delay(100);
      radio.read( &data2receive, sizeof(data2receive) );
      radio.writeAckPayload(pipeNo,&ackByte, 1 );  
      if(data2receive != oldByte){
        Serial.println(data2receive);
        oldByte = data2receive;
        ackByte++;
        array2receive[count] = data2receive;
        count++;
      }
    } 
  }

  aResIn = array2receive[0];
  gResIn = array2receive[1];
  mResIn = array2receive[2];
  getAres();
  Serial.println(aRes, 4);
  getGres();
  Serial.println(gRes, 4);
  getMres();
  Serial.println(mRes, 4);

  Serial.println("GO");
}


void loop()
{
  int16_t dataIn[15];
  if (radio.available())
  {
    radio.read( &dataIn, sizeof(dataIn));

    // Accelerometer
    Serial.print((float)dataIn[0]*aRes);  // North -1
    Serial.print (",");
    Serial.print((float)dataIn[1]*aRes);  // East -0
    Serial.print (",");
    Serial.print((float)dataIn[2]*aRes);   // Down -2
    Serial.print (";");
   
    // Gyroscope
    Serial.print((float)dataIn[3]*gRes);  // -4
    Serial.print (",");
    Serial.print((float)dataIn[4]*gRes);   // 3
    Serial.print (",");
    Serial.print((float)dataIn[5]*gRes);  //-5
    Serial.print (";");
    
    // Magnetometer
    Serial.print((float)dataIn[6]*mRes*listSensMAGX);  //6
    Serial.print (",");
    Serial.print((float)dataIn[7]*mRes*listSensMAGY);  //7
    Serial.print (",");
    Serial.print((float)dataIn[8]*mRes*listSensMAGZ);   //-8
    Serial.print (";");

    // GPS
    if (dataIn[9]!= 0)
    {
      Serial.print((float)dataIn[9]/100.0 + (float)dataIn[10]/1000000.0, 6);  // in °
      Serial.print (",");
      Serial.print((float)dataIn[11]/100.0 + (float)dataIn[12]/1000000.0, 6);  // in °
      Serial.print (",");
      Serial.print(0);  
      Serial.print (";");
      Serial.print((float)dataIn[13]/10.0,1);  // in °
      Serial.print (",");
      Serial.print((float)dataIn[14]/100.0,2);  // in m/s
      Serial.print (",");
      Serial.print(0);  
      }
   
    // End of line
    Serial.println("");
  }
  
}
