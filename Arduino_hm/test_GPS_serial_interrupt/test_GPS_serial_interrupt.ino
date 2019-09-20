#include <NMEAGPS.h>

//======================================================================
//  Program: NMEA_isr.ino
//
//  Prerequisites:
//     1) NMEA.ino works with your device
//
//  Description:  This minimal program parses the GPS data during the 
//     RX character interrupt.  The ISR passes the character to
//     the GPS object for parsing.  The GPS object will add gps_fix 
//     structures to a buffer that can be later read() by loop().
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

#include <GPSport.h>

#include <Streamers.h>

// Check configuration

#ifndef NMEAGPS_INTERRUPT_PROCESSING
  #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

static NMEAGPS   gps;
static gps_fix  fix_data;

//--------------------------

static void GPSisr( uint8_t c )
{
  gps.handle( c );

} // GPSisr

//--------------------------

void setup()
{
  NeoSerial.begin(9600);
  while (!NeoSerial)
    ;

  NeoSerial.print( F("NMEA_isr.INO: started\n") );
  NeoSerial.print( F("fix object size = ") );
  NeoSerial.println( sizeof(gps.fix()) );
  NeoSerial.print( F("NMEAGPS object size = ") );
  NeoSerial.println( sizeof(gps) );
  NeoSerial.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  trace_header( NeoSerial );
  NeoSerial.flush();

  gpsPort.attachInterrupt( GPSisr );
  gpsPort.begin(38400); // BT must run at this rate, too.
}

//--------------------------

void loop()
{
  //NeoSerial.println("hello");
  if (gps.available()) {
    // Print all the things!
    fix_data = gps.read();
    NeoSerial.print(fix_data.latitudeL());
    NeoSerial.print(", ");
    NeoSerial.println(fix_data.latitudeL());
    //trace_all( NeoSerial, gps, gps.read() );
  }

  if (gps.overrun()) {
    gps.overrun( false );
    NeoSerial.println( F("DATA OVERRUN: took too long to print GPS data!") );
  }
}
