// test eeprom 24AA32 HAT ->DuePi
// I2C1 SDA1 PA17 pin 70, SCL1 PA18 pin 71
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and eeprom 24AA32.

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


#include <Wire.h>
int addess_HAT = 0x50;                                              // 24AA32->1010000x
String trama = "";
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("Test EEPROM 24AA32");
  Wire1.begin();
}

void loop() {
      //(0-104 info. HATs) HATs-vendor+product 
      //42-69 ->RADIOELF -GNU-DronPi
      // 42                                                          62
      // R  A   D  I  O  E  L  F     -  G  N  U  -  D  r   o   n   P  i       
      Serial.println("Leemos EEPRON:");   
      for (int x = 42; x < 62; x++) {                              // leememos de la posición 42 a la 62
        trama.concat((char) readEEPROM(x));    
      }
      if (trama != ""){
        Serial.println(trama); 
        write_double_eeprom(2000, 2.3415);
        Serial.println();
        Serial.println ("Escribimos en la posicion 2000 -> 2.3415");
        Serial.print("Leemos la posicion 2000: ");
        Serial.println(read_double_eeprom(2000), 4);
        while(1){
          delay(1000);
        }
      }
      else{
        Serial.println("ERROR!! en lectura");
      }
}
/********************************************************************
  EEPROM HATs 24AA32 0x00-0xFFF (4095bytes = 4Kbytes), I2C1
*********************************************************************/
void writeEEPROM(unsigned int eeaddress, byte data ) 
{
  Wire1.beginTransmission(addess_HAT);
  Wire1.write((int)(eeaddress >> 8));                        // MSB
  Wire1.write((int)(eeaddress & 0xFF));                      // LSB
  Wire1.write(data);
  Wire1.endTransmission();
  delay(6);
}
//------------------------------------------------------------------ 
byte readEEPROM(unsigned int eeaddress) 
{
  byte rdata = 0xFF;
  Wire1.beginTransmission(addess_HAT);
  Wire1.write((int)(eeaddress >> 8));                         // MSB
  Wire1.write((int)(eeaddress & 0xFF));                       // LSB
  Wire1.endTransmission();
 
  Wire1.requestFrom(addess_HAT,1);
  if (Wire1.available()) 
    rdata = Wire1.read();
  delay(1);
  return rdata;
} 
//------------------------------------------------------------------ 
double read_double_eeprom(unsigned int direccion)
{
  double dato = 0.0;
  byte* p = (byte*)(void*)&dato;
  for (int i = 0; i < sizeof(dato); i++)
    *p++ = readEEPROM(direccion++);
  return dato;
}
//------------------------------------------------------------------ 
void write_double_eeprom(unsigned int direccion, double dato)
{
  //const byte* p = (const byte*)(const void*)&dato;
  byte* p = (byte*)(void*)&dato;
  for (int i = 0; i < sizeof(dato); i++)
       writeEEPROM(direccion++, *p++);
}
