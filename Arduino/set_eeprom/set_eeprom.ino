/**
 * EEPROM initializer that reads instructions from the USB Port and sets
 * EEPROM memory as directed.
 * 
 * Records have the following fixed length format:
 * 
 * Bytes  Contents:
 * -----  ---------------------------------------------------
 *   0-3  EEPROM address in HEX. Note that there is NO prefix
 *     4  One space
 *   5-6  Value in HEX. Note that there is NO orefix
 *     7  Newline character
 */

#include <EEPROM.h>
 
#define BAUD 115200

static uint8_t index = 0;
static uint8_t bytes[16];
static char ch;

void setup() {
  Serial.begin(BAUD);
  while(!Serial) {}
  Serial.println("Serial I/O Ready!");
}

void loop() {
  if (Serial.available() > 0) {
    ch = Serial.read();
    Serial.print("At index ");
    Serial.print(index);
    Serial.print(" read: ");
    Serial.println(ch, HEX);
    bytes[index++] = ch;
    if (index >= 7) {
      uint16_t address = 0;
      uint8_t value = 0;
      uint8_t i = 0;
      while (i < 4) {
        address = (address <<4) | bytes[i] & 0x0F;
        ++i;
      }

      ++i;

      while (i < 7) {
        value = (value << 4) | bytes[i] & 0x0F;
        ++i;
      }

      index = 0;

      EEPROM.update(address, value);
      Serial.print("EEPROM address ");
      Serial.print(address);
      Serial.print("Has been set to: ");
      Serial.println(EEPROM.read(address));
    }
    Serial.print("At end, index is ");
    Serial.println(index);
  }
}
