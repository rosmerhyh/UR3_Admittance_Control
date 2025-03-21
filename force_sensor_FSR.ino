int fsrPin = 0;     // the FSR and 1K pulldown are connected to A0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage

void setup(void) {
  Serial.begin(115200);   // We'll send debugging information via the Serial monitor
}

void loop(void) {
  fsrReading = analogRead(fsrPin);
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);  // Convert analog reading to voltage

  // Send fsrVoltage over serial
  Serial.println(fsrVoltage);
  delay(480);  // Delay for stability
  //Serial.flush();
}