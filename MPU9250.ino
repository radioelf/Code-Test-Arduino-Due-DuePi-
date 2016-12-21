// Test MPU9250 (SPI)->DuePi
// MPU9250 Giroscopio/Acelerómetro/Compás interno/9-AXIS (Bus SPI, max. 20Mhz) 
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and  MPU9250.

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
// MPU9250  Magnetic flux density [μT] Min.-4800  Max.4800 (-48,00 a 48,00 Gauss)

#include <SPI.h>                                                      // SPI para sensores
#include <MPU9250.h>                                                  // https://github.com/brianc118/MPU9250

#if defined(ARDUINO_SAM_DUE)
  #define duepi "TEST MPU9250"
#else
  #error  !!ERROR board Due!! 
#endif
 
#define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
#define CS_MPU_PIN 87                                                 // PA29 PIN SAM3X 112, DronPi PIN-> 26, Max. output current 15mA
#define INT_PIN 22                                                    // PB26 PIN SAM3X 1,   DronPi PIN-> 16, Max. output current 3mA!!
#define SPI_CLOCK 1000000                                             // 1MHz clock para bus SPI
MPU9250 mpu(SPI_CLOCK, CS_MPU_PIN);                                   // 1Mhz, pin 87->PA29, low pass filter 184hz, low_pass_filter_acc 184hz
//MPU9250 mpu(SPI_CLOCK, CS_MPU_PIN, BITS_DLPF_CFG_42HZ, BITS_DLPF_CFG_42HZ); // 8Mhz, pin 87->PA29, low pass filter 42hz, low_pass_filter_acc 42hz
 
 //boolean status_MPU9250 = false;

//double gyroDataRateSec = 1.0/760.0;                                //tiempo óptimo entre las muestras-> 0,00131578947368421052631578947368

//double gyro[3] = {0,0,0};   
//double accel[3] = {0,0,0};              
//double compass[3] = {0,0,0};                                        
int8_t wai =0;

void setup() {
    
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                                    // habilitamos placa DronPi
  pinMode(CS_MPU_PIN, OUTPUT);
  digitalWrite(CS_MPU_PIN, HIGH);
  Serial.begin(115200);
  delay(150);
  pinMode(INT_PIN, INPUT);
  SPI.begin();
  delay(100);
  Serial.println(duepi);
  if (initializeMPU9250()){
    delay(2500);
  }else{
    while (true){
      delay(250);
    }
  }
  mpu.read_all();
  mpu.read_all();                                                   // despreciamos primera lectura

while (true){}
}
void loop() {  
  ReadMPU9250(0);
  delay(100);
  ReadMPU9250(1);                                                 // obtenemos las lecturas del acelerómetro  MPU9250
  delay(100);
  ReadMPU9250(2);                                                 // obtenemos las lecturas del compás  MPU9250
  delay(500);
  ReadMPU9250(3);                                                 // obtenemos las lecturas del giroscopio  MPU9250
  delay(500);
}
boolean  initializeMPU9250() {
  mpu.init(true);
  delay(10);
  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    Serial.println("TEST: MPU9250 SPI OK");
    mpu.calib_acc();
    mpu.calib_mag();
    return true;
  }
  else{
    Serial.print("TEST: Error  MPU9250 0x");
    Serial.println(wai, HEX);
    return false;
  }
}
void ReadMPU9250(byte readTo){
  switch (readTo){
    case 1:
      mpu.read_acc();
        Serial.print("RX acelerometro: ");
        Serial.print(mpu.accel_data[0]);  Serial.print('\t');
        Serial.print(mpu.accel_data[1]);  Serial.print('\t');
        Serial.println(mpu.accel_data[2]); 
        Serial.println(); 
    break;
    case 2:
      mpu.read_mag();
      Serial.print("RX magnetometro: ");
        Serial.print(mpu.mag_data[0]);    Serial.print('\t');
        Serial.print(mpu.mag_data[1]);    Serial.print('\t');
        Serial.println(mpu.mag_data[2]);  
        Serial.println(); 
    break;
    case 3:
      mpu.read_gyro();
      Serial.print("RX   giroscopio: ");
       Serial.print(mpu.gyro_data[0]);   Serial.print('\t');
        Serial.print(mpu.gyro_data[1]);   Serial.print('\t');
        Serial.println(mpu.gyro_data[2]);  
        Serial.println();
    break;
    default:
      mpu.read_all();
      Serial.print("RX:              ");
        Serial.print(mpu.gyro_data[0]);   Serial.print('\t');
        Serial.print(mpu.gyro_data[1]);   Serial.print('\t');
        Serial.print(mpu.gyro_data[2]);   Serial.print('\t');
        Serial.print(mpu.accel_data[0]);  Serial.print('\t');
        Serial.print(mpu.accel_data[1]);  Serial.print('\t');
        Serial.print(mpu.accel_data[2]);  Serial.print('\t');
        Serial.print(mpu.mag_data[0]);    Serial.print('\t');
        Serial.print(mpu.mag_data[1]);    Serial.print('\t');
        Serial.print(mpu.mag_data[2]);    Serial.print('\t');
        Serial.println(mpu.temperature);
        Serial.println();
    break;
  }
}
