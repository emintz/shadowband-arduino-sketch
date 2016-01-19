/*
    Test sketch that validates the overlapped analog to digital conversion
    Note that this test requires an ATMega 328 or equivalent with signals
    applied to all 8 analog pins. Use an Arduino Nano or some other circuit
    built around the flat pack version of the IC because the DIP version
    exposes only 6 analog pins.

    Copyright (C) 2016 The Winer Observatory, http://www.winer.org

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "OverlappedAnalogRead.h"


#define ANALOG_PIN_COUNT 8

#define GREEN_LIGHT 2
#define RED_LIGHT 3

#define BAUD 9600

OverlappedAnalogRead analogReader;

uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
uint8_t current_pin = 7;

void setup() {
  Serial.begin(BAUD);
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, LOW);
  digitalWrite(RED_LIGHT, LOW);
  while(!Serial) {}
}

void loop() {
  switch(analogReader.get_state()) {
    case OverlappedAnalogRead::IDLE:
      digitalWrite(GREEN_LIGHT, LOW);
      digitalWrite(RED_LIGHT, LOW);
      current_pin = (current_pin + 1) % 8;
      analogReader.startConvertingPin(current_pin);
      break;

    case OverlappedAnalogRead::CONVERTING:
      digitalWrite(GREEN_LIGHT, LOW);
      digitalWrite(RED_LIGHT, HIGH);
      analogReader.checkConversion();
      break;

    case OverlappedAnalogRead::DONE:
      digitalWrite(GREEN_LIGHT, HIGH);
      digitalWrite(RED_LIGHT, HIGH);
      uint16_t analog_value = analogReader.readAnalogValue();
      Serial.print("Pin ");
      Serial.print(current_pin);
      Serial.print(" has value ");
      Serial.print(analog_value);
      Serial.println('.');
      break;
  }
}
