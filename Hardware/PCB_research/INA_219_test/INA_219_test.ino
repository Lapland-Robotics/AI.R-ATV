#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  Serial.println("I2C Scanner starting...");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } 
  }
  if (nDevices == 0) Serial.println("No I2C devices found.\n");
  else Serial.println("Done.\n");

  delay(3000); // Wait 3 seconds, then repeat
}
