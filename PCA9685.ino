// Test PCA9685 -> DuePi
// PCA9685 Generador de PWM (Bus I2c) dirección 0x40 y 0x70((ALLCALL) 
// Arduino Due 3V3 SDA D4 pin 7, SCL D5 pin 8
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and PCA9685.

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

#define PCA9685_RA_MODE1           0x00
#define PCA9685_RA_MODE2           0x01
#define PCA9685_RA_LED0_ON_L       0x06
#define PCA9685_RA_LED0_ON_H       0x07
#define PCA9685_RA_LED0_OFF_L      0x08
#define PCA9685_RA_LED0_OFF_H      0x09
#define PCA9685_RA_ALL_LED_ON_L    0xFA
#define PCA9685_RA_ALL_LED_ON_H    0xFB
#define PCA9685_RA_ALL_LED_OFF_L   0xFC
#define PCA9685_RA_ALL_LED_OFF_H   0xFD
#define PCA9685_RA_PRE_SCALE       0xFE

#define PCA9685_MODE1_RESTART_BIT  (1 << 7)
#define PCA9685_MODE1_EXTCLK_BIT   (1 << 6)
#define PCA9685_MODE1_AI_BIT       (1 << 5)
#define PCA9685_MODE1_SLEEP_BIT    (1 << 4)
#define PCA9685_MODE1_SUB1_BIT     (1 << 3)
#define PCA9685_MODE1_SUB2_BIT     (1 << 2)
#define PCA9685_MODE1_SUB3_BIT     (1 << 1)
#define PCA9685_MODE1_ALLCALL_BIT  (1 << 0)

#define PCA9685_MODE2_INVRT_BIT    (1 << 4)
#define PCA9685_MODE2_OCH_BIT      (1 << 3)
#define PCA9685_MODE2_OUTDRV_BIT   (1 << 2)
#define PCA9685_MODE2_OUTNE1_BIT   (1 << 1)
#define PCA9685_MODE2_OUTNE0_BIT   (1 << 0)
#define PCA9685_ALL_LED_OFF_H_SHUT (1 << 4)
#define PCA9685_INTERNAL_CLOCK (1.04f * 25000000.f)
#define PCA9685_EXTERNAL_CLOCK 24576000.f

#include <Wire.h>

#if defined(ARDUINO_SAM_DUE)
 #define duepi "Test PWM PCA9685"
#else
#error  !!ERROR!! 
#endif

#define PWM_EN_PIN 18                                        // PA11 PIN SAM3X 4,   DronPi PIN-> 13, Max. output current 3mA!!
#define ENABL_PIN 31                                         // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA

#define ESC_Frec 400                                         // blheli ESC->20-500hz, 400hz->2.5ms

#define direccion 0x40

#define numberOfMotors  4
uint16_t MinimPulse = round(4096 / (1000.f / ESC_Frec)) - 1;
int motorCommand[numberOfMotors] = {0,0,0,0};  
uint16_t frecuencia =50;
float frequency;
bool x = false;

void setup() {
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                             // habilitamos placa DronPi
  pinMode(PWM_EN_PIN, OUTPUT);
  digitalWrite(PWM_EN_PIN, HIGH);                            // pin PWM_EN_PIN a 1 (deshabilitado)
  
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  Wire.setClock(400000);                                     // 400kHz I2C clock. 
  Wire.begin();
  delay(250);
  Init_PCA9685();
  if (testPCA9685){
    Serial.println("PCA9685 OK");
  }
  sleep();
  setFrequency(ESC_Frec);                                    // configuramos frecuencia 
  delay(5);
  Info_MODE1();
  delay(500);
  
}

void loop() {

  inverter(x);
  if (x){
    Serial.println("Salida invertida");
  }else{
    Serial.println("Salida normal");
  }
  setAllPWM(0);
  setPWM(0, 2000,1000);                                       // Channel(0-15), Offset (0-4095), Length (0-4095)
  Serial.println("Salida Port 0->2000-1000");
  delay(5000);
  setPWM_(0, 1000);                                           // Channel (0-15), Length (0-4095)
  Serial.println("Salida Port 0->1000");
  delay(5000);
  setPWMmS(0, 1);                                             // Channel (0-15). Length en milisegundos
  Serial.println("Salida Port 0->1mS");
  delay(5000);
  setPWMuS(0, 1500);                                          // Channel (0-15). Length en microsegundos
  Serial.println("Salida Port 0->1500uS");
  delay(5000);
  setAllPWM(200,100);                                         // Offset (0-4095), Length (0-4095)
  Serial.println("Salida todos->200-100");
  delay(5000);
  setAllPWM(500);                                             // Length (0-4095)
  Serial.println("Salida todos->500");
  delay(5000);
  setAllPWMmS(2);                                             // Length en milisegundos
  Serial.println("Salida todos->2mS");
  delay(5000);
  setAllPWMuS(800);                                           // Length en milisegundos
  Serial.println("Salida todos->800uS");
  delay(5000);
  x = !x;
}
//*************************************************************************
void Init_PCA9685() {
  write_register(PCA9685_RA_MODE2, B00000100);               // defecto PCA9685_RA_MODE2
  frequency = getFrequency();                                // lee el valor prescale almacenado en PCA9685 y calcula la frecuencia basada en él
  write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_AI_BIT));
  digitalWrite(PWM_EN_PIN, LOW);                             // pin PWM_EN_PIN a 1 (habilitado)
  restart();                                                 // el reinicio se realiza para habilitar el reloj
}
// Desactivar el modo de reposo e iniciar las salidas
void restart() {
    write_register ( PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);
    write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_SLEEP_BIT  |  PCA9685_MODE1_EXTCLK_BIT));
    write_register (PCA9685_RA_MODE1, (PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_EXTCLK_BIT| PCA9685_MODE1_AI_BIT));
}  
bool testPCA9685() {
    if (read_register(PCA9685_RA_PRE_SCALE))
        return true;
    else
return false;
}
// La salida se desactivan si estamos en sleep (sleep OFF)
void sleep() {
  byte MODE1 =read_register(PCA9685_RA_MODE1);
  write_register(PCA9685_RA_MODE1, (MODE1 | PCA9685_MODE1_SLEEP_BIT));
  delay(1);
}
//lee el valor prescale actual y calcula la fecuencia 
float getFrequency() {
    uint8_t data =read_register(PCA9685_RA_PRE_SCALE);
    return 24576000.f / 4096.f / (data + 1);
}
//Calcula el valor prescale basado en la frecuencia especificada y lo escribimos en el dispositivo.
void setFrequency(float frecuencia) {
    sleep();
    delay(1);
    uint8_t prescale = roundf(24576000.f / 4096.f / frecuencia)  - 1;
    write_register( PCA9685_RA_PRE_SCALE, prescale);
    frequency = getFrequency();
    Serial.print("prescale:");Serial.println(prescale);//14
    Serial.print("frecuencia:");Serial.println(frequency);//400
    restart();
}
// Establece el offset de inicio del pulso y su longitud del canal indicado. Channel(0-15), Offset (0-4095), Length (0-4095)
void setPWM(uint8_t channel, uint16_t offset, uint16_t length) {
    uint8_t data[4] = {0, 0, 0, 0};
    if(length == 0) {
        data[3] = 0x10;
    } else if(length >= 4096) {
        data[1] = 0x10;
    } else {
        data[0] = offset & 0xFF;
        data[1] = offset >> 8;
        data[2] = length & 0xFF;
        data[3] = length >> 8;
    }
    write_register_bytes ((PCA9685_RA_LED0_ON_L + 4 * channel), data[0], data[1], data[2], data[3]);
    //write_register((PCA9685_RA_LED0_ON_L + 4 * channel), data[0]);
    //write_register((PCA9685_RA_LED0_ON_H + 4 * channel), data[1]);
    //write_register((PCA9685_RA_LED0_OFF_L + 4 * channel), data[2]);
    //write_register((PCA9685_RA_LED0_OFF_H + 4 * channel), data[3]);
}
void inverter(bool ON_OFF){
  byte data =read_register(PCA9685_RA_MODE2);
  if (ON_OFF){
    bitSet (data, 4);                         // invertir salida
  }else{
    bitClear(data ,4);                        // NO invertir salida
  }
  write_register(PCA9685_RA_MODE2, data);
}
// Establece la longitud del impulso del canal indicado, Channel (0-15), Length (0-4095)
void setPWM_(uint8_t channel, uint16_t length) {
    setPWM(channel, 0, length);
}

// Establece la longitud en milisegundos del impulso del canal indicado.Channel (0-15). Length en milisegundos
void setPWMmS(uint8_t channel, float length_mS) {
    setPWM_(channel, round((length_mS * 4096.f) / (1000.f / frequency)));
}

// Establece la duración del impulso del canal en microsegundos. Channel number (0-15). Length en microsegundos
void setPWMuS(uint8_t channel, float length_uS) {
    setPWM_(channel, round((length_uS * 4096.f) / (1000000.f / frequency)));
}

// Establece el offset de inicio del pulso y su longitud detodos los canales. Offset (0-4095), Length (0-4095)
void setAllPWM(uint16_t offset, uint16_t length) {
    uint8_t data[4] = {offset & 0xFF, offset >> 8, length & 0xFF, length >> 8};
    write_register_bytes((PCA9685_RA_ALL_LED_ON_L), data[0], data[1], data[2], data[3]);
    //write_register((PCA9685_RA_ALL_LED_ON_L), data[0]);
    //write_register((PCA9685_RA_ALL_LED_ON_H), data[1]);
    //write_register((PCA9685_RA_ALL_LED_OFF_L), data[2]);
    //write_register((PCA9685_RA_ALL_LED_OFF_H), data[3]);
}

// Establece la longitud del pulso para todos los canales. Length (0-4095)
void setAllPWM(uint16_t length) {
    setAllPWM(0, length);
}

// Establece la longitud del pulso en milisegundos para todos los canales. Length en milisegundos
void setAllPWMmS(float length_mS) {
    setAllPWM(round((length_mS * 4096.f) / (1000.f / frequency)));
}

// Establece la longitud del pulso en milisegundos para todos los canales. Length en microsegundos
void setAllPWMuS(float length_uS) {
    setAllPWM(round((length_uS * 4096.f) / (1000000.f / frequency)));
}
//************************************************************************
void write_register_bytes(int regAddress, byte data0, byte data1, byte data2, byte data3){
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.write(data0);
  Wire.write(data1);
  Wire.write(data2);
  Wire.write(data3);
  Wire.endTransmission();
}
void write_register(int regAddress, byte data){
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
word read_register(int regAddress) {
  word returnword = 0x00;
  Wire.beginTransmission(direccion);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)direccion, 1);
  // esperamos e bytes
  while (Wire.available()) {
    returnword |= Wire.read(); 
  }
  return returnword;
}
void Info_MODE1(){
  byte info_MODE1 = read_register(PCA9685_RA_MODE1);
  Serial.println("******-Info registro MODE1-*********");
  Serial.print("MODE1 ->");
  Serial.println(info_MODE1, BIN);
  if (bitRead (info_MODE1, 0)){
    Serial.println("ON direccion LED_ALL");
  }else{
    Serial.println("OFF direccion LED_ALL");
  }  
  if (bitRead (info_MODE1, 1)){
    Serial.println("ON subdireccion 3 I2C-bus");
  }else{
    Serial.println("OFF subdireccion 3 I2C-bus");
  }
  if (bitRead (info_MODE1, 2)){
    Serial.println("ON subdireccion 2 I2C-bus");
  }else{
    Serial.println("OFF subdireccion 2 I2C-bus");
  }
  if (bitRead (info_MODE1, 3)){
    Serial.println("ON subdireccion 1 I2C-bus");
  }else{
    Serial.println("OFF subdireccion 1 I2C-bus");
  }
  if (bitRead (info_MODE1, 4)){
    Serial.println("Modo reposo"); 
  }else{
    Serial.println("Modo mormal");
  }
  if (bitRead (info_MODE1, 5)){
    Serial.println("Incremento habilitado");
  }else{
    Serial.println("Incremento deshabilitado");
  }
  if (bitRead (info_MODE1, 6)){
    Serial.println("Oscilador externo");
  }else{
    Serial.println("Oscilador interno");
  }
  if (bitRead (info_MODE1, 7)){
    Serial.println("Reiniciar habilitado");
  }else{
    Serial.println("Reiniciar deshabilitado");
  }
  Serial.println("************************************");
}


