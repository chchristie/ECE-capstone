#include <Wire.h>

byte val = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Initializing MAX32664...");
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  
  digitalWrite(8, HIGH); // Set mfio HIGH
  digitalWrite(7, LOW);  // Set ~rstn LOW
  delay(10);     // Wait for 10 milliseconds
  digitalWrite(7, HIGH); // Set ~rstn HIGH permanently

  delay(1500);
  Serial.println("MAX32664 initialized.");
  Wire.begin(); // Join I2C bus
  digitalWrite(8, LOW); // Set mfio LOW permanently
}

void loop() {
    Wire.beginTransmission(0x55);  // Transmit MAX32664 address

    //Wire.write(val);             // Sends value byte
    Wire.endTransmission();      // Stop transmitting

    val++;                       // Increment value

    // if reached 64th position (max)
    if(val == 64) {
        val = 0;                   // Start over from lowest value
        Serial.println("Done");
    }

    delay(500);
}
