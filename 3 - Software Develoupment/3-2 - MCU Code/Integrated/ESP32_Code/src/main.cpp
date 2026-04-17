#include <Arduino.h>

// Use IO23 (Pin 37 on the ESP32 chip / Pin 1 on Header J8)
#define LED_PIN 23 

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(100); // Fast blink so it's obvious
  
  digitalWrite(LED_PIN, LOW);
  delay(100);
}