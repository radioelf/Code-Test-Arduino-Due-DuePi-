// Test leds, zumbador y pulsador ON/OFF -> DuePi
// Arduino Due 3V3 
/* ******************************************************************* 
  Radioelf - December 2016
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2016 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and test I/O.

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
#define LED_Verde_PIN 7                                               // PB23 PIN SAM3X 134, DronPi PIN-> 18, Max. output current 15mA
#define LED_Azul_PIN 8                                                // PC22 PIN SAM3X 133, DronPi PIN-> 22, Max. output current 15mA
#define LED_L_PIN  13                                                 // PB27 PIN SAM3X 68,  DronPi PIN-> N/C,Max. output current 3mA!!
#define BUZZER_PIN 66                                                 // PB15 PIN SAM3X 76,  DronPi PIN-> 29, Max. output current 3mA!!
#define ON_OFF_PIN 48                                                 // PC15 PIN SAM3X 97,  DronPi PIN-> 40, Max. output current 15mA

#define ON_LED LOW
#define OFF_LED HIGH

#if defined(ARDUINO_SAM_DUE)
  #define duepi "Test LEDs, zumbador y pulsador"
#else
  #error  !!ERROR board Due!! 
#endif

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println(duepi);
  pinMode(LED_Verde_PIN, OUTPUT);
  digitalWrite(LED_Verde_PIN, OFF_LED);
  pinMode(LED_Azul_PIN, OUTPUT);
  digitalWrite(LED_Azul_PIN, OFF_LED);
  pinMode(LED_L_PIN, OUTPUT);
  digitalWrite(LED_L_PIN, OFF_LED);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(ON_OFF_PIN, INPUT);                                       // pin ON/OFF como entrada
  attachInterrupt(digitalPinToInterrupt(ON_OFF_PIN), isr, CHANGE);
}

void loop() {
 Serial.println("LED verde");
 digitalWrite(LED_Verde_PIN, ON_LED); 
 delay(1000);
 Serial.println("LED Azul");
 digitalWrite(LED_Azul_PIN, ON_LED);
 digitalWrite(LED_Verde_PIN, OFF_LED);
 delay(1000);
 Serial.println("LED L");
 digitalWrite(LED_L_PIN, HIGH);
 digitalWrite(LED_Azul_PIN, OFF_LED);
 delay(1000);
 digitalWrite(LED_L_PIN, LOW);
 Serial.println("Zumbador");
 digitalWrite(BUZZER_PIN, HIGH);
 delay(50);
 digitalWrite(BUZZER_PIN, LOW);
 delay(1000);
}
void isr() {
  if (digitalRead(ON_OFF_PIN) == LOW)
    Serial.println("Pulsando");
  else
    Serial.println("Reposo");
}
