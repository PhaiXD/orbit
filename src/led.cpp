#include <Arduino.h>

// Blink LED using a variable for the pin
constexpr uint32_t ledPin = PB14;   // LED connected to PB15

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);   // Configure pin as output
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn LED ON
  Serial.println(HIGH);
  delay(500);                  // Wait 500 ms
  digitalWrite(ledPin, LOW);   // Turn LED OFF
  Serial.println(LOW);
  delay(500);                  // Wait 500 ms
}
