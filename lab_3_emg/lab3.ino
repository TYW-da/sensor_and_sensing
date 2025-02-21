const int analogPin = A1;
void setup() {
  Serial.begin(115200);
  pinMode(analogPin, INPUT);
}

void loop() {
  int sensorValue = analogRead(analogPin);
  Serial.println(sensorValue);
  delay(1);
}