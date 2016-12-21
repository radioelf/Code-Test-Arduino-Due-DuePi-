// Test HMC5883 ->DuePi
// Arduino Due 3V3 SDA D4 pin 7, SCL D5 pin 8
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and HMC5883.

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
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>                                       // https://github.com/adafruit/Adafruit_HMC5883_Unified

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test HMC5883"
#else
  #error  !!ERROR board Due!! 
#endif

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(00001);

#define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  initializeHMC5883();
}
void loop() {
  ReadHMC5883();
  delay(1000);
}

void  initializeHMC5883() {
  mag.begin();
}
void ReadHMC5883(){
  // Información básica sobre el sensor
  sensor_t sensor;
  mag.getSensor(&sensor);

    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
    Serial.println("------------------------------------");
    Serial.println("");

  // Obtenemos un nuevo evento del sensor
  sensors_event_t event; 
  mag.getEvent(&event);

    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

}
