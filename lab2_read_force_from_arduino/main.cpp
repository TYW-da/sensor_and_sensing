#include <Arduino.h>

float k = 0.2;
float value1;

void setup() {
 Serial.begin(9600);
}

void loop() {
  value1 = (float)analogRead(0) * k + value1 * (1.-k); 
  Serial.println(value1);
  delay(100);
}