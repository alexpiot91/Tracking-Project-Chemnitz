#include <SoftwareSerial.h>
SoftwareSerial SerialGPS(4, 3);

void setup()
{
 SerialGPS.begin(9600); // GPS communication at 9600 baud
 Serial.begin(9600); // set serial output to 9600 baud
 while (!Serial); // wait for serial output monitor
 Serial.println("Waiting for GPS data..."); // show something until GPS kicks in
}
void loop()
{
 if (SerialGPS.available()) // a character is ready to read...
 {
 char c = SerialGPS.read(); // so get it
 Serial.print(c); // and display it
 }
}
