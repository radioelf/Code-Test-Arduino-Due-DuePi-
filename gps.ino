// test GPS serial ->DuePi
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and GPS serial.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// libraries/AQ_Gps -> https://github.com/2012SEng466/copter/tree/master/Aero_Quad_X/Libraries

#define GPS_SERIAL Serial2  //pines 17-RX, 16-TX
//#define UseGPSUBLOX
#include "GpsAdapter.h"

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test GPS"
#else
  #error  !!ERROR board Due!! 
#endif

byte espera =1;
String trama = "";

void setup() {
  Serial.begin(115200);                                    // puerto serie 0 a 115200
  initializeGps();
  delay(250);
  Serial.println(duepi);
    Serial.print("Puerto GPS serie 1 a: ");
    Serial.print(gpsBaudRates[gpsData.baudrate]);
    Serial.println(" Bps");

}
void loop() {
  trama = "";
  updateGps();                                              // actualizamos datos del GPS   
  if (haveAGpsLock()){
    Serial.print("Satelites GPS: ");
    Serial.println(gpsData.sats);
    Serial.print("latitud: ");
    Serial.println(currentPosition.latitude);               // grados/10000000
    Serial.print("longitud: ");
    Serial.println(currentPosition.longitude);              // grados/10000000
    Serial.print("Grados: ");
    Serial.println(getCourse()/10);                         // grados
    Serial.print("Velocidad: ");
    Serial.println(getGpsSpeed()*36/1000);                  // km/h
    Serial.print("Altura: ");
    Serial.println((currentPosition.altitude /1000));       // altura metros
    Serial.println();
    }else{
      if (espera ==50){
        Serial.print("Satelites GPS: ");
        Serial.println(gpsData.sats);
        Serial.print("Puerto GPS serie 1 a: ");
        Serial.print(gpsBaudRates[gpsData.baudrate]);
        Serial.println(" Bps") ; 
      }
      delay(100);
      if (--espera ==0){
        do{
          while (GPS_SERIAL.available()>0){                 //Max. 64 bytes
          trama.concat((char) GPS_SERIAL.read());
          }
          if (trama != "") Serial.println(trama);
          delay(100);
          espera ++; 
        }while(espera <50);    
      }
    }
    delay(100);
}
