
#define BAUD 115200

#define GREEN_LIGHT 2
#define RED_LIGHT 3

#define ANALOG_PIN_COUNT 8
#define DATA_SIZE 2 * ANALOG_PIN_COUNT
#define BUFFER_SIZE DATA_SIZE + 2

#define SEND (unsigned char) 'S'

void setup() {
  Serial.begin(BAUD);
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, HIGH);
  digitalWrite(RED_LIGHT, HIGH);

  analogReference(DEFAULT);

  delay(500);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(GREEN_LIGHT, HIGH);
    digitalWrite(RED_LIGHT, LOW);
    delay(250);
    digitalWrite(GREEN_LIGHT, LOW);
    digitalWrite(RED_LIGHT, HIGH);
    delay(100);
  }
  digitalWrite(RED_LIGHT, LOW);
  while(!Serial) {}
  digitalWrite(RED_LIGHT, HIGH);
}

void loop() {
  if (acquire_data()) {
      digitalWrite(RED_LIGHT, LOW);
      digitalWrite(GREEN_LIGHT, HIGH);

      uint8_t checksum = 0;
      Serial.write((uint8_t) BUFFER_SIZE);

      for (int i = 0; i < ANALOG_PIN_COUNT; ++i) {
        int val = analogRead(i);
        uint8_t lsb = lowByte(val);
        uint8_t msb = highByte(val);
        checksum ^= lsb;
        Serial.write(lsb);
        checksum ^= msb;
        Serial.write(msb);
      }
      Serial.write(checksum);
      digitalWrite(RED_LIGHT, HIGH);
      digitalWrite(GREEN_LIGHT, LOW);
  }

}


boolean acquire_data() {
  boolean result = false;
  if (0 < Serial.available()) {
    result = Serial.read() == SEND;
  }

  return result;
}

