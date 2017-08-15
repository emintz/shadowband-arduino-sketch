#define IN_PANIC_BUTTON_NOT 9
#define BAUD 115200

void setup() {
  Serial.begin(BAUD);
  while(!Serial) {}
  Serial.println("Starting.");

  pinMode(IN_PANIC_BUTTON_NOT, INPUT_PULLUP);

  Serial.println("Initialization complete.");
  Serial.flush();
}

void loop() {
  if (digitalRead(IN_PANIC_BUTTON_NOT) == LOW) {
    Serial.println("Panic button is LOW");
  } else {
    Serial.println("Panic button is HIGH!");
  }
  Serial.flush();
  delay(100);

}
