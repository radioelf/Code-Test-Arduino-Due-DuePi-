// Test MCP2515 -> DuePi
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and MCP2515.

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

#include <mcp_can.h>                                                  // https://github.com/Seeed-Studio/CAN_BUS_Shield
#include <SPI.h>


#define CS_CAN_PIN 52                                                 // PB21 PIN SAM3X 92,  DronPi PIN-> 38, Max. output current 3mA!!
#define ENABL_PIN 31                                                  // PA7  PIN SAM3X 26,  DronPi PIN-> 1,  Max. output current 15mA
#define INT_CAN_PIN 19                                                // PA10 PIN SAM3X 3,   DronPi PIN-> 15, Max. output current 3mA!!

#define Led_TX 1
#define Led_RX 2

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test MCP2515"
#else
  #error  !!ERROR board Due!! 
#endif

bool flagRecv = false;
bool TX_data = true;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

MCP_CAN CAN(CS_CAN_PIN);                                    

void setup()
{
  Serial.begin(115200);
  pinMode(ENABL_PIN, OUTPUT);
  digitalWrite(ENABL_PIN, HIGH);                                    // habilitamos placa DronPi
  delay(250);
  Serial.println(duepi);
  while (CAN_OK != CAN.begin(CAN_500KBPS))                          // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS fallo al inicializar");
    Serial.println(" Init CAN BUS de nuevo!");
    delay(1000);
  }
  Serial.println("CAN BUS incializado ok!");
  attachInterrupt(INT_CAN_PIN, MCP2515_ISR, FALLING);               // Dispara en el flanco de bajada
}

void loop()
{
  if (TX_data){
    CAN.sendMsgBuf(0x00, 0, 8, stmp);
    TX_data = false;
    ON_led(Led_TX);
    Serial.println("CAN Bus TX..");
  }   
  delay(100);
  if(flagRecv) {                                   
    flagRecv = 0;                   
    TX_data = true;
    OFF_led(Led_TX);
    ON_led(Led_RX);
    Serial.println("CAN Bus IRQ!");
    while (CAN_MSGAVAIL == CAN.checkReceive()){                     // read data,  len: data length, buf: data buf          
      CAN.readMsgBuf(&len, buf);
      for(int i = 0; i<len; i++){
        Serial.print(buf[i]);Serial.print("\t");
      }
      Serial.println();           
    }
    OFF_led(Led_RX);
  }
}
void MCP2515_ISR(){
  flagRecv = true;
}

void ON_led(byte led_ON){
  byte estado_leds =CAN.mcp2515_readRegister(MCP_BFPCTRL);
  if (led_ON ==1){
    bitSet (estado_leds, B0BFE);
    CAN.mcp2515_setRegister(MCP_BFPCTRL, estado_leds);
  }else{
    bitSet (estado_leds, B1BFE);
    CAN.mcp2515_setRegister(MCP_BFPCTRL, estado_leds);
  }
}
void OFF_led(byte led_OFF){
  byte estado_leds =CAN.mcp2515_readRegister(MCP_BFPCTRL);
  if (led_OFF ==1){
    bitClear (estado_leds, B0BFE); 
    CAN.mcp2515_setRegister(MCP_BFPCTRL, estado_leds);
  }else{
    bitClear (estado_leds, B1BFE);
    CAN.mcp2515_setRegister(MCP_BFPCTRL, estado_leds);
  }
}
