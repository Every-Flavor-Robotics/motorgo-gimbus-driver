#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>

// Include your custom pin definitions file.
#include "define_pins.h"

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include <Adafruit_HUSB238.h>

const uint8_t HUSB_X42_ADDR = 0x42;
const uint8_t HUSB_X62_ADDR = 0x62;

Adafruit_HUSB238 husb_x42;
Adafruit_HUSB238 husb_x62;


// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);

// ... (rest of your motor and driver setup code remains the same) ...
BLDCMotor motor = BLDCMotor(11, 6.900, 24);
BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
LowsideCurrentSense i_sense_motor = LowsideCurrentSense(0.15, 1.0f, isen_u, isen_v, isen_w);
Commander command = Commander(Serial);
void doMotor(char* cmd) {command.motor(&motor, cmd);}
bool setup_motor(){ /* ... your existing code ... */ return true; }
CRGB ind[1];
bool setup_indicator(){ /* ... your existing code ... */ return true; }
// --- End of unchanged section ---

/**
 * @brief DEBUGGING: Performs a raw read of the PD_STATUS1 register and prints it.
 * @param device_name A character string for the device's name in serial output.
 * @param device_addr The I2C address of the chip for raw register reading.
 */
void print_raw_device_status(const char* device_name, uint8_t device_addr) {
  uint8_t raw_value = 0;
  
  // Perform a direct I2C read of the status register
  Wire.beginTransmission(device_addr);
  Wire.write(HUSB238_PD_STATUS1); // Target register 0x01
  byte error = Wire.endTransmission(false); // Use false to keep connection active for read

  if (error == 0) {
    if (Wire.requestFrom((int)device_addr, 1) == 1) {
      raw_value = Wire.read();
    } else {
      Serial.print(device_name);
      Serial.println(" Status: Read Fail (No data returned)");
      return;
    }
  } else {
    Serial.print(device_name);
    Serial.print(" Status: Write Fail (Error: ");
    Serial.print(error);
    Serial.println(")");
    return;
  }

  // Print the raw value we read for diagnosis
  Serial.print(device_name);
  Serial.print(" Status: [Raw Val: 0x");
  if (raw_value < 0x10) Serial.print("0");
  Serial.print(raw_value, HEX);
  Serial.println("]");
}

void setup() {
  Serial.begin(115200);
  
  while (!Serial);
  delay(1000);
  
  Serial.println("Gimbus test - HUSB238 Status Check (RAW READ TEST)");
  
  Serial.println("Starting I2C bus...");
  Wire.begin(i2c_sda, i2c_scl);
  delay(100);

  setup_motor();
  setup_indicator();

  Serial.println("Initializing HUSB238 devices...");
  if (!husb_x42.begin(HUSB_X42_ADDR)) {
    Serial.println(" - Failed to find HUSB238 chip at 0x42");
  } else {
    Serial.println(" - HUSB238 at 0x42 found! Resetting...");
    husb_x42.reset();
  }

  if (!husb_x62.begin(HUSB_X62_ADDR)) {
    Serial.println(" - Failed to find HUSB238 chip at 0x62");
  } else {
    Serial.println(" - HUSB238 at 0x62 found! Resetting...");
    husb_x62.reset();
  }
  delay(200);

  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
}

void loop() {
  // motor.move();
  // motor.loopFOC();
  // motor.monitor();
  // command.run();

  // Call the simplified RAW read function
  Serial.println(husb_x42.isAttached()? "attached" : "unattached");
  Serial.println(husb_x42.getCCdirection()? "CC1 Connected" : "CC2 connected");

   // What voltages and currents are available from this adapter?
  Serial.println("Available PD Voltages and Current Detection Test:");
  for (int i = PD_SRC_5V; i <= PD_SRC_20V; i++) {
    bool voltageDetected = husb_x42.isVoltageDetected((HUSB238_PDSelection)i);
    switch ((HUSB238_PDSelection)i) {
      case PD_SRC_5V:
        Serial.print("5V");
        break;
      case PD_SRC_9V:
        Serial.print("9V");
        break;
      case PD_SRC_12V:
        Serial.print("12V");
        break;
      case PD_SRC_15V:
        Serial.print("15V");
        break;
      case PD_SRC_18V:
        Serial.print("18V");
        break;
      case PD_SRC_20V:
        Serial.print("20V");
        break;
      default:
        continue;
    }
    Serial.print(voltageDetected ? " Available " : " Unavailable ");
  }

  Serial.println(husb_x62.isAttached()? "attached" : "unattached");
  Serial.println(husb_x62.getCCdirection()? "CC1 Connected" : "CC2 connected");
   // What voltages and currents are available from this adapter?
  Serial.println("Available PD Voltages and Current Detection Test:");
  for (int i = PD_SRC_5V; i <= PD_SRC_20V; i++) {
    bool voltageDetected_2 = husb_x62.isVoltageDetected((HUSB238_PDSelection)i);

    switch ((HUSB238_PDSelection)i) {
      case PD_SRC_5V:
        Serial.print("5V");
        break;
      case PD_SRC_9V:
        Serial.print("9V");
        break;
      case PD_SRC_12V:
        Serial.print("12V");
        break;
      case PD_SRC_15V:
        Serial.print("15V");
        break;
      case PD_SRC_18V:
        Serial.print("18V");
        break;
      case PD_SRC_20V:
        Serial.print("20V");
        break;
      default:
        continue;
    }
    Serial.print(voltageDetected_2 ? " Available" : " Unavailable");
  }
  
  Serial.println("--------------------------");

  delay(2000);
}