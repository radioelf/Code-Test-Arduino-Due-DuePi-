// Test PPM ->DuePi
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and PPM.

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

#define ENABL_PIN 31                                         // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
#define PPM_PIN 15                                                    // PD5  PIN SAM3X 18,  DronPi PIN-> 7,  Max. output current 15mA
#define MAX_PPM_CHANNELS 8          

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Inicializado RX PPM"
#else
  #error  !!ERROR board Due!! 
#endif

const int FrameSpace = 2500;
unsigned long Last_Time;
unsigned long Current_Time;
unsigned long Delta_Time;
volatile byte Current_Channel = 0;
volatile int PPM[MAX_PPM_CHANNELS];
byte x = 8;

void setup() {
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                             // habilitamos placa DronPi
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  initializeReceiverPPM();
  Serial.println("Seleccionar el canal a monitorizar 0-7");
}

void loop() {
  if(Serial.available() > 0){ // Si existen datos de entrada en la comunicacion serial 
    x = (Serial.read()-48); 
  }
  if (getRawChannelValue(x) >1){
    Serial.print("RX Canal: ");
    Serial.print(x);
    Serial.print("->");
    Serial.println(PPM[x]);
  }
  delay(100);
}

// Recepción de datos del equipo remoto de radio a través de PPM
//IRQ PPM
void CalcPPM()
{
  Current_Time = micros();
  Delta_Time = Current_Time - Last_Time;
  Last_Time = Current_Time;
  if (Delta_Time > FrameSpace) 
    Current_Channel = 0;                       // inicio, canal 0
  else{
    PPM[Current_Channel]=Delta_Time;
    Current_Channel++;
  }   
} 
void  initializeReceiverPPM()
{
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(PPM_PIN, CalcPPM, RISING);// CHANGE
    Last_Time = micros(); 
}

int getRawChannelValue(byte channel) {  
  return PPM[channel];
}
