#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(); // This will initialize I2C on default pins
}

void loop() {
  while(!Serial);
  Serial.println("I2C Scanner");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int result = Wire.endTransmission();
    Serial.print("Address: 0x");
    if (address < 16) Serial.print("0");
    Serial.print(address, HEX);
    Serial.print(" Result: ");
    Serial.println(result);
    if (result == 0) {
      Serial.print("I2C device found at address: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scan complete");
}