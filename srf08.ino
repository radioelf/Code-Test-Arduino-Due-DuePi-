// Test SRF08 ->DuePi
/* ******************************************************************* 
  Radioelf - December 2017
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2017 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and SRF08.

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
*****************************************************************************
Registros
RX 0x00 revisión del firmware, 0x01 sensor luz, 0x02 byte alto eco, 0x03 byte bajo del eco, ...0x23 (rx de 16 ecos)
TX 0x00 registro comandos, 0x01 registro ganancia, 0x03 registro de rango

Comandos
 0x50 medición en pulgadas, 0x51 medición en centimetros, 0x52 medición en micro segundos
Modo Artificial Neural Network ANN
 0x53 medición en pulgadas, 0x54 medición en centimetros, 0x55 medición en micro segundos
Cambio dirección I2C
 0xA0 1ª secuencia, 0xAA 2ª secuencia, 0xA5 3ª secuencia
Direcciones I2C
0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4, 0xF6, 0xF8, 0xFA, 0xFC o 0xFE -> 8bits 
0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E o 0x7F -> 7bits + W/R
*/

#include <Wire.h>
#define     Pulgadas        0x50              // cálculo distancia en Pulgadas
#define     Cm              0x51              // cálculo distancia en centímetros
#define     micro_Seg       0x52              // cálculo distancia en microsegundos
#define     secuencia_1     0xA0              // secuencia 1ª cambio dirección I2C
#define     secuencia_2     0xAA              // secuencia 2ª cambio dirección I2C
#define     secuencia_3     0xA5              // secuencia 3ª cambio dirección I2C

uint8_t SRF_ADDRESS[] {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F};// dirección modulo SRF08
uint8_t Address =1;                           // 0x71

uint8_t gain = 0x08;                          // de 0->31 = 94->1025, default 0x1F->31
uint8_t range = 0x8C;                         // 6mts, scope =(rangex43mm) + 43mm), max 11mts 0xFF, 0->43mm, 255->10965mm, default 0xFF->255
uint8_t mesure = Cm;                          // centimetros
int  reading = 0;

#if defined(ARDUINO_SAM_DUE)
  #define duepi "TEST SRF08"
#else
  #error  !!ERROR board Due!! 
#endif

void setup(){
  Serial.begin(115200);
  delay(150);
  Wire.begin();                              
  delay(100);                                     
  Serial.println(duepi);
  uint8_t SRF08status = getSoft();
  if (SRF08status !=0 && SRF08status !=255){
    while (!Conf_Read()){};
    delay(100);
    Conf_Gain(gain);
    Serial.print("Ganancia a: 0x");
    Serial.println(gain, HEX);
    delay(100);
    Conf_Range(range);
    Serial.print("Rango a: 0x");
    Serial.println(range, HEX);
    Serial.print("Distancia maxima de: ");
    Serial.print((range *43) + 43);
    Serial.println("mm");
    delay(100);
    Serial.print("Direccion I2c Actual: 0x");
    Serial.println(SRF_ADDRESS[Address], HEX);
    delay(100); 
    Serial.print("Software revision: ");
    Serial.println(SRF08status, DEC);
    delay(100);
  }else{
    Serial.print("Error en comunicacion SRF08!!");
    while(true){};
  }
}

void loop(){
  RX_SRF08();
  if (mesure == 0x52){
    Serial.print("Eco a : ");
    Serial.print( reading);
    Serial.println(" microsegundos");    
  }else{
    Serial.print("Distancia: ");
    Serial.print( reading);
    if (mesure == 0x50){
      Serial.println(" pulgadas");
    }else{
      Serial.println(" centimetros");
    }
  }
  Serial.print("Int. Luz: ");
  Serial.print(getLight());
  Serial.println("%");
  delay(500);                                      
}
// get command
void SRF08_Command(uint8_t Command){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS[Address]));
  Wire.write(uint8_t(0x00));                                // Dirección interna 0x00 (registro de comandos)
  Wire.write(Command);                                      // Enviar comando a ejecutar
  Wire.endTransmission(); 
  delay(70);                                                //  >65ms                                  
}
// get config. gain
void Conf_Gain(uint8_t gain){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS[Address])); 
  Wire.write(uint8_t(0x01));                         
  Wire.write(uint8_t(gain));                 
  Wire.endTransmission();   
}
// get config. range
void Conf_Range(uint8_t range){
  Wire.beginTransmission(uint8_t (SRF_ADDRESS[Address])); 
  Wire.write(uint8_t(0x02));                         
  Wire.write(uint8_t(range));                 
  Wire.endTransmission();   
}
// change address I2C
void changeAddress(uint8_t NEW_ADDRESS){  
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));  
  Wire.write(uint8_t(0x00));                                 
  Wire.write(uint8_t(secuencia_1));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));  
  Wire.write(uint8_t(0x00));  
  Wire.write(uint8_t(secuencia_2));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));  
  Wire.write(uint8_t(0x00));  
  Wire.write(uint8_t(secuencia_3));  
  Wire.endTransmission();  
 
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));  
  Wire.write(uint8_t(0x00));  
  Wire.write(SRF_ADDRESS[NEW_ADDRESS]*2);                      // pasamos de 7bits a 8bits
  Wire.endTransmission();  
  Address = NEW_ADDRESS;
  delay(70);                                        
}
// read mesure
void RX_SRF08(){                                  
  SRF08_Command(mesure);                                                      
  
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));             
  Wire.write(uint8_t(0x02));                           
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t(SRF_ADDRESS[Address]),uint8_t (2)); // peticion 2 bytes 
  while(Wire.available() < 2);                     
   reading = Wire.read();  
   reading =  reading << 8;    
   reading |= Wire.read();                                 
}
// read LDR, 0x00 min., 0xF8 max.
uint8_t getLight(){   
  SRF08_Command(mesure);                                      // actualizamos la lectura del LDR   
                                   
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));
  Wire.write(uint8_t(0x01));                           
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t (SRF_ADDRESS[Address]),uint8_t (1));// peticion 1 byte
  while(Wire.available() < 0);  
  byte LDR = Wire.read();                  
  return(round (LDR/2.55));                                    // Returns % LDR
}
// read software version
uint8_t getSoft(){                                     
  Wire.beginTransmission(uint8_t(SRF_ADDRESS[Address]));             
  Wire.write(uint8_t(0x00));                                 
  Wire.endTransmission();
  
  Wire.requestFrom(uint8_t(SRF_ADDRESS[Address]),uint8_t (1)); // peticion 1 byte
  while(Wire.available() < 0);                                         
  return(Wire.read());                               
}
// config. data read
bool Conf_Read(){
  Serial.println("Configurar lectura a: ");
  Serial.println("p-> pulgadas, c-> centimetros, m-> microsegundos");
  while (!Serial.available()){};
     char RX = Serial.read();
     switch (RX) {
      case 'p':
        mesure = Pulgadas;     
        Serial.println("Lectura en pulgadas");
        return true;
      case 'c':
        mesure = Cm;
        Serial.println("Lectura en centimetros");
        return true;
      case 'm':
        mesure = micro_Seg;
        Serial.println("Lectura de microsegundos");
        return true;
      default:
        Serial.println("Error Conf. lectura!!");
        delay(1000);
        while (Serial.available() !=0){Serial.read();};
        return false;
    }
}
