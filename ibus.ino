// Test ibus ->DuePi
// Arduino Due 3V3 SERIAL 3 -> PD5 
// FlySky FS-IA10B (with 10 channel patch) 

// based on code https://github.com/povlhp/iBus2PPM

/* ******************************************************************* 
  Radioelf - January 2017
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and I-bus FS-IA10B.
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

#define IBUS_MAXCHANNELS 14
#define FAILSAFELIMIT 1020                          // When all the 6 channels below this value assume failsafe
#define IBUS_BUFFSIZE 32                            // Max iBus packet size (2 byte header (0x20, 0x40), 14 channels x 2 bytes, 2 byte checksum)

#define ENABL_PIN 31                                // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test I-BUS"
#else
  #error  !!ERROR board Due!! 
#endif  
                                                                                                                                   
static uint16_t rcValue[IBUS_MAXCHANNELS];
static uint16_t rcBuffer[IBUS_MAXCHANNELS];
static boolean rxFrameDone;
static boolean failsafe = false;
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};

uint8_t i;
uint16_t chksum, rxsum;

void setup() {

  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                    // enabled board DronPi
  Serial.println(duepi);
}

void loop() {
  rxFrameDone = false;
  uint8_t dato =  Serial3.available();
  if (dato){
    uint8_t rx = Serial3.read();
    //  for start 0x20  as 0x40 packet
    if (ibusIndex == 0 && rx != 0x20) {
      return;
    }
    if (ibusIndex == 1 && rx != 0x40) {
      ibusIndex = 0;
      return;
    } 
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = rx;
    ibusIndex++;
    if (ibusIndex == IBUS_BUFFSIZE){
      ibusIndex = 0;
      chksum = 0xFFFF;
      for (i = 0; i < 30; i++){
        chksum -= ibus[i];
      }
      rxsum = ibus[30] + (ibus[31] << 8);
      if (chksum == rxsum){
        // 10 channels 
        rcValue[0] = (ibus[ 3] << 8) + ibus[ 2];
        rcValue[1] = (ibus[ 5] << 8) + ibus[ 4];
        rcValue[2] = (ibus[ 7] << 8) + ibus[ 6];
        rcValue[3] = (ibus[ 9] << 8) + ibus[ 8];
        rcValue[4] = (ibus[11] << 8) + ibus[10];
        rcValue[5] = (ibus[13] << 8) + ibus[12];
        rcValue[6] = (ibus[15] << 8) + ibus[14];
        rcValue[7] = (ibus[17] << 8) + ibus[16];
        rcValue[8] = (ibus[19] << 8) + ibus[18];
        rcValue[9] = (ibus[21] << 8) + ibus[20];
        rxFrameDone = true;
        if (rcValue[0] < FAILSAFELIMIT && rcValue[1] < FAILSAFELIMIT &&
            rcValue[2] < FAILSAFELIMIT && rcValue[3] < FAILSAFELIMIT &&
            rcValue[4] < FAILSAFELIMIT && rcValue[5] < FAILSAFELIMIT ) {
          failsafe = true;
          Serial.println("Error");             //  Error  
        }else{
        // 10 channels
          for (i = 0; i<10; i++) {
            if (rcValue[i] !=  rcBuffer[i]) { 
              Serial.print("Canal ");
              Serial.print(i);
              Serial.print(": ");
              Serial.println(rcValue[i]);
              rcBuffer[i] = rcValue[i];
            }
          }      
        }
      }else{
        Serial.println("Checksum error");     // Checksum error 
      }
    }
  }   
}


