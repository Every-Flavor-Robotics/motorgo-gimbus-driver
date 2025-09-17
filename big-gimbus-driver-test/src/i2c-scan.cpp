#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "define_pins.h"

// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);

CRGB ind[1];

bool setup_indicator(){
  pinMode(IND_LIGHT, OUTPUT);
  FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
  ind[0] = CRGB::DeepPink1;
  FastLED.show();
  return true;
}

bool setup_i2c(){
  Serial.println("Trying to start I2C");
  Wire.begin(i2c_sda, i2c_scl);
  delay(300);
  Serial.println("I2C init success");
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("Gimbus test");
  setup_indicator();
  setup_i2c();
}

void loop() {
  byte error, address;
  int nDevices;

  nDevices = 0;
  // The I2C address space is 7-bits, so devices can have addresses
  // from 1 to 127. We will scan this range.
  for(address = 1; address < 127; address++ ) {
    // The Wire library uses a buffer and writes to it with write().
    // We begin a transmission to the I2C slave device with the given address.
    Wire.beginTransmission(address);
    // endTransmission() will transmit the bytes that were queued by write().
    // It returns 0 if an ACK is received from the device, meaning a device was found.
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("Scan complete.\n");
  }

  delay(1000);
  // Serial.println("Motor Driver SPI Check:")
}
