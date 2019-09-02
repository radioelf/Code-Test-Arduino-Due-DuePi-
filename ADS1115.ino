// Test ADS1115 -> DuePi
// Arduino Due 3V3 SDA D4 pin 7, SCL D5 pin 8
/*            R1                R2
 *   Int    |-----|           |-----|
 * ----->---| 10K |-----*-----| 20K |-------*GND
 *          |-----|     |     |-----|
 *                      |          
 *                      V
 *                 Int ADC ADS1115
 *  Vout =  R2/(R1+R2)/Vint
 *  Vout = (200000/30000)/5 = 3.33V ->1811->5V 65535 
 * ******************************************************************* 
 Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and ADC1115.

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

#include <Wire.h>                                                     // I2C para sensor
#include <Adafruit_ADS1015.h>

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test ADC1115"
#else
  #error  !!ERROR board Due!! 
#endif

Adafruit_ADS1115 ads1115;

#define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA

float multiplo = 1.125F;                                              // ADS1115  +/- 4.096V ganancia 1x (16-bit) 
byte canal =0;

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  // La ganacia del la entrada ADC se pueden cambiar a través de la siguiente
  // funciones, pero se tiene que tener cuidado de no superar nunca VDD + 0,3 V,
  // La configuración de estos valores de forma incorrecta puede destruir el ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads1115.begin();  
  ads1115.setGain(GAIN_ONE);
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                                    // habilitamos placa DronPi
}

void loop() {
 int16_t lectura;
 lectura = ReadADS1115(canal);
 Serial.print("Lectura canal: ");
 Serial.print (canal);
 Serial.print ("-> ");
 Serial.print(lectura);
 Serial.print("(");
 Serial.print(lectura * multiplo);
 Serial.println("mV)");
 if (canal ==3) canal =0; else canal++;
 delay(1000);
}
int16_t ReadADS1115(byte channel){
  return ads1115.readADC_SingleEnded(channel);
}
