// Test barometro MS5611-01 -> DronPi
// Arduino Due spi 3v3
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and  MS5611-01.

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
#include <SPI.h>   
  
#define CS_BARO_PIN 77                                            // PA28 PIN SAM3X 111, DronPi PIN-> 24, Max. output current 15mA
#define ENABL_PIN 31                                              // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48                               // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58                               // Maximun resolution

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test barometro MS561101"
#else
  #error  !!ERROR board Due!! 
#endif

uint16_t C1,C2,C3,C4,C5,C6;
uint32_t D1,D2;
int32_t _raw_press;
int32_t _raw_temp;
long MS5611_timer;
uint8_t MS5611_State;
float Factor     = 1/5.255;
byte elevacion =27;                                              // elevación actual sobe el mar nivel del mar 1013.25 hPa/mb, 15ºC de temperatura, 0% de humedad
float calculo_altura=NULL;
float temperatura =NULL;
float presion = NULL;
bool ok = true;
float tmp_float;
void setup() {
  pinMode(CS_BARO_PIN, OUTPUT);
  digitalWrite(CS_BARO_PIN, HIGH);                              // Pin CS a 1->dehabilitado 
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                                // habilitamos placa DronPi
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  SPI.begin ();
  SPI.setClockDivider(84);                                       // 84Mhz /84->1Mhz
  calculo_altura = (float)101325 * (pow(((288-0.0065 *elevacion)/288),5.256));
  Serial.println("Espera...");
  delay(2500);
  InitMS5611();
  delay(1000);
}

void loop() {
  ok =ReadMS5611();                                              // actualizamos temperatura y presion
  if (MS5611_State ==4){                                         // solo mostralos la cuarta lectura
    Serial.print("Respuesta: "); 
    if (ok){                                                
      Serial.println("RX ->OK");  
      Serial.print("presion: ");
      Serial.println(presion/100);
      Serial.print("Temperatura: ");
      Serial.println(temperatura/100);
      Serial.print("Altura: ");
      Serial.print(44330 * (1.0 - pow((float)(presion)/((101325+presion)-calculo_altura), Factor)), 6);
      //tmp_float = (Press / 101325.0);
      //tmp_float = pow(tmp_float, 0.190295);
      //Serial.println(44330.0 * (1.0 - tmp_float));
      Serial.println("mt");
      
    }
    else{
      Serial.println("ERROR!");
    }
  }
  delay(500);
}
//***************************************************************************************************
// SPI Debe ser inicializado
void InitMS5611()
{
  pinMode(CS_BARO_PIN, OUTPUT);  // Chip select Pin
  MS5611_SPI_write(CMD_MS5611_RESET);
  delay(4);
  // Leemos la calibración de fábrica
  C1 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C1);
  C2 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C2);
  C3 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C3);
  C4 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C4);
  C5 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C5);
  C6 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C6);
  //Enviar un comando para leer primero la temperatura
  MS5611_SPI_write(CMD_CONVERT_D2_OSR4096);
  MS5611_timer = millis();
  MS5611_State = 1;
  temperatura = 0;
  presion = 0;
}
// maquina de 4 estados
uint8_t ReadMS5611()
{
  uint8_t result = 0;
  if (MS5611_State == 1){
    if (MS5611_Ready()){
      D2 =MS5611_SPI_read_ADC();          
      //_raw_temp = D2;
      MS5611_State++;
      MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Comando para leer la presión
      MS5611_timer = millis();   
    }
  }else{
    if (MS5611_State == 5){
      if (MS5611_Ready()){
        D1 = MS5611_SPI_read_ADC();
        //_raw_press = D1;
        calcular();
        MS5611_State = 1;                         // Comienzar de nuevo desde el estado = 1
        MS5611_SPI_write(CMD_CONVERT_D2_OSR4096); // Comando para leer la temperatura
        MS5611_timer = millis();
        result = 1;                               // Nueva lectura de presión
      }
    }else{
      if (MS5611_Ready()){
        D1 = MS5611_SPI_read_ADC();
        //_raw_press = D1;
        calcular();
        MS5611_State++;
        MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Comando para leer la presión
        MS5611_timer = millis();
        result = 1;                                // Nueva lectura de presión
      }
    }
  }
  return(result);
}
// El proceso de conversión 8.2ms Desde el comando
uint8_t MS5611_Ready()
{
  if ((millis()-MS5611_timer)>10)                       // Esperar más de 10ms
    return(1);
  else
    return(0);
}
// Calcular la temperatura y la presión compensada en unidades reales (grados centigrados*100, mbar*100)
void calcular()
{
  int32_t dT;
  int64_t TEMP;  // 64 bits
  int64_t OFF;
  int64_t SENS;
  int64_t P;

  // Formulas from manufacturer datasheet
  // as per data sheet some intermediate results require over 32 bits, therefore
  // we define parameters as 64 bits to prevent overflow on operations
  // sub -20c temperature compensation is not included
  dT = D2-((long)C5*256);
  TEMP = 2000 + ((int64_t)dT * C6)/8388608;
  OFF = (int64_t)C2 * 65536 + ((int64_t)C4 * dT ) / 128;
  SENS = (int64_t)C1 * 32768 + ((int64_t)C3 * dT) / 256;

  if (TEMP < 2000){   // second order temperature compensation
    int64_t T2 = (((int64_t)dT)*dT) >> 31;
    int64_t Aux_64 = (TEMP-2000)*(TEMP-2000);
    int64_t OFF2 = (5*Aux_64)>>1;
    int64_t SENS2 = (5*Aux_64)>>2;
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  P = (D1*SENS/2097152 - OFF)/32768;
  temperatura = TEMP;
  presion = P;
}

uint8_t MS5611_SPI_read(byte reg)
{
  byte dump;
  uint8_t return_value;
  byte addr = reg; // | 0x80;                                   // Establesemos el bit más significativo
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return(return_value);
}
uint16_t MS5611_SPI_read_16bits(byte reg)
{
  byte dump,byteH,byteL;
  uint16_t return_value;
  byte addr = reg; // | 0x80;                                    // Establecemos el bit más significativot
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return(return_value);
}

uint32_t MS5611_SPI_read_ADC()
{
  byte dump,byteH,byteM,byteL;
  uint32_t return_value;
  byte addr = 0x00;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(CS_BARO_PIN, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return(return_value);
}
void MS5611_SPI_write(byte reg)
{
  byte dump;
  digitalWrite(CS_BARO_PIN, LOW);
  dump = SPI.transfer(reg);
  digitalWrite(CS_BARO_PIN, HIGH);
}
