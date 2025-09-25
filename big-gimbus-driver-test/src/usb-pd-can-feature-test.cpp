#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>

// Include your custom pin definitions file.
#include "define_pins.h"

// #include <SimpleFOC.h>
// #include <SimpleFOCDrivers.h>
// #include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include <Adafruit_HUSB238.h>

const uint8_t HUSB_X42_ADDR = 0x42;
const uint8_t HUSB_X62_ADDR = 0x62;

Adafruit_HUSB238 husb_x42;
Adafruit_HUSB238 husb_x62;


// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
// MagneticSensorMT6701SSI sensor(enc_cs);

// ... (rest of your motor and driver setup code remains the same) ...
// BLDCMotor motor = BLDCMotor(11, 6.900, 24);
// BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
// LowsideCurrentSense i_sense_motor = LowsideCurrentSense(0.15, 1.0f, isen_u, isen_v, isen_w);
// Commander command = Commander(Serial);
// void doMotor(char* cmd) {command.motor(&motor, cmd);}
// bool setup_motor(){ /* ... your existing code ... */ return true; }
CRGB ind[1];
bool setup_indicator(){ /* ... your existing code ... */ return true; }
// --- End of unchanged section ---

/**
 * @brief Performs a raw I2C read of a range of registers and prints their values.
 * This helps diagnose issues by bypassing the library's logic.
 * @param device_addr The I2C address of the chip.
 * @param device_name A string to identify the device in the serial output.
 */
void dump_husb_registers(uint8_t device_addr, const char* device_name) {
  Serial.print("Raw Register Dump for ");
  Serial.println(device_name);
  Serial.println(" REG | VAL");
  Serial.println("-----|-----");
  for (uint8_t reg = 0x00; reg <= 0x0A; reg++) {
    Wire.beginTransmission(device_addr);
    Wire.write(reg);
    byte error = Wire.endTransmission(false); // Use false to keep connection active

    if (error != 0) {
      Serial.print("ERR  | Failed to write to register 0x");
      Serial.println(reg, HEX);
      continue;
    }

    if (Wire.requestFrom((int)device_addr, 1) == 1) {
      uint8_t val = Wire.read();
      Serial.print(" 0x");
      if (reg < 0x10) Serial.print("0");
      Serial.print(reg, HEX);
      Serial.print(" | 0x");
      if (val < 0x10) Serial.print("0");
      Serial.println(val, HEX);
    } else {
      Serial.print(" 0x");
      if (reg < 0x10) Serial.print("0");
      Serial.print(reg, HEX);
      Serial.println(" | READ FAIL");
    }
  }
}


/**
 * @brief Prints a human-readable string for a given current setting.
 * @param srcCurrent The HUSB238_CurrentSetting enum value to print.
 */
void printCurrentSetting(HUSB238_CurrentSetting srcCurrent) {
  switch (srcCurrent) {
    case CURRENT_0_5_A:   Serial.print("0.5A"); break;
    case CURRENT_0_7_A:   Serial.print("0.7A"); break;
    case CURRENT_1_0_A:   Serial.print("1.0A"); break;
    case CURRENT_1_25_A:  Serial.print("1.25A"); break;
    case CURRENT_1_5_A:   Serial.print("1.5A"); break;
    case CURRENT_1_75_A:  Serial.print("1.75A"); break;
    case CURRENT_2_0_A:   Serial.print("2.0A"); break;
    case CURRENT_2_25_A:  Serial.print("2.25A"); break;
    case CURRENT_2_50_A:  Serial.print("2.50A"); break;
    case CURRENT_2_75_A:  Serial.print("2.75A"); break;
    case CURRENT_3_0_A:   Serial.print("3.0A"); break;
    case CURRENT_3_25_A:  Serial.print("3.25A"); break;
    case CURRENT_3_5_A:   Serial.print("3.5A"); break;
    case CURRENT_4_0_A:   Serial.print("4.0A"); break;
    case CURRENT_4_5_A:   Serial.print("4.5A"); break;
    case CURRENT_5_0_A:   Serial.print("5.0A"); break;
    default:              Serial.print("Unknown"); break;
  }
}

/**
 * @brief Performs a comprehensive status check on a HUSB238 device and prints the results.
 * @param husb A reference to the Adafruit_HUSB238 object to check.
 * @param device_name A string to identify the device in the serial output.
 */
void print_husb_status(Adafruit_HUSB238 &husb, const char* device_name) {
  Serial.print(F("--- Status for "));
  Serial.print(device_name);
  Serial.println(F(" ---"));

  // Determine whether attached or unattached
  bool attached = husb.isAttached();
  Serial.print("Attachment Status: ");
  Serial.println(attached ? "Attached" : "Unattached");

  if (!attached) {
    Serial.println(); // Add a newline for spacing and exit
    return;
  }
  
  // Test getCCStatus function
  bool ccStatus = husb.getCCdirection();
  Serial.print("CC Direction: ");
  Serial.println(ccStatus ? "CC1 connected" : "CC2 Connected");

  // Check if we can get responses to our PD queries!
  HUSB238_ResponseCodes pdResponse = husb.getPDResponse();
  Serial.print("USB PD query response: ");
  switch (pdResponse) {
    case NO_RESPONSE:                   Serial.println("No response"); break;
    case SUCCESS:                       Serial.println("Success"); break;
    case INVALID_CMD_OR_ARG:            Serial.println("Invalid command or argument"); break;
    case CMD_NOT_SUPPORTED:             Serial.println("Command not supported"); break;
    case TRANSACTION_FAIL_NO_GOOD_CRC:  Serial.println("Transaction fail"); break;
    default:                            Serial.println("Unknown response code"); break;
  }

  if (pdResponse != SUCCESS) {
    Serial.println(); // Add a newline for spacing and exit
    return;
  }

  // Is there a default 5V 'contract' voltage available
  bool contractV = husb.get5VContractV();
  Serial.print("5V Contract Voltage: ");
  Serial.print(contractV ? "5V" : "Other");
  
  // How much current can we get?
  HUSB238_5VCurrentContract contractA = husb.get5VContractA();
  Serial.print(" & Current: ");
  switch (contractA) {
    case CURRENT5V_DEFAULT: Serial.println("Default current"); break;
    case CURRENT5V_1_5_A:   Serial.println("1.5A"); break;
    case CURRENT5V_2_4_A:   Serial.println("2.4A"); break;
    case CURRENT5V_3_A:     Serial.println("3A"); break;
    default:                Serial.println("Unknown current"); break;
  }

  // What is the actual voltage being output right now?
  HUSB238_VoltageSetting srcVoltage = husb.getPDSrcVoltage();
  Serial.print("Current Source Voltage: ");
  switch (srcVoltage) {
    case UNATTACHED:  Serial.println("Unattached"); break;
    case PD_5V:       Serial.println("5V"); break;
    case PD_9V:       Serial.println("9V"); break;
    case PD_12V:      Serial.println("12V"); break;
    case PD_15V:      Serial.println("15V"); break;
    case PD_18V:      Serial.println("18V"); break;
    case PD_20V:      Serial.println("20V"); break;
    default:          Serial.println("Unknown voltage setting"); break;
  }

  // What is the max current available right now?
  HUSB238_CurrentSetting srcCurrent = husb.getPDSrcCurrent();
  Serial.print("Current Source Max Current: ");
  printCurrentSetting(srcCurrent);
  Serial.println();

  // What voltages and currents are available from this adapter?
  Serial.println("Available PD Voltages & Currents:");
  for (int i = PD_SRC_5V; i <= PD_SRC_20V; i++) {
    HUSB238_PDSelection pdo = (HUSB238_PDSelection)i;
    bool voltageDetected = husb.isVoltageDetected(pdo);

    switch (pdo) {
      case PD_SRC_5V:  Serial.print("  - 5V"); break;
      case PD_SRC_9V:  Serial.print("  - 9V"); break;
      case PD_SRC_12V: Serial.print("  - 12V"); break;
      case PD_SRC_15V: Serial.print("  - 15V"); break;
      case PD_SRC_18V: Serial.print("  - 18V"); break;
      case PD_SRC_20V: Serial.print("  - 20V"); break;
      default: continue;
    }
    Serial.print(voltageDetected ? ": Available" : ": Unavailable");

    // Loop over currents if voltage is detected
    if (voltageDetected) {
      HUSB238_CurrentSetting currentDetected = husb.currentDetected(pdo);
      Serial.print(" up to ");
      printCurrentSetting(currentDetected);
    }
    Serial.println();
  }
  Serial.println(); // Final newline for spacing
}


void setup() {
  Serial.begin(115200);
  
  while (!Serial);
  delay(1000);
  
  Serial.println("Gimbus test - HUSB238 Full Status Check");
  
  Serial.println("Starting I2C bus...");
  Wire.begin(i2c_sda, i2c_scl);
  delay(100);

  // setup_motor();
  setup_indicator();

  Serial.println("Initializing HUSB238 devices...");
  if (!husb_x42.begin(HUSB_X42_ADDR, &Wire)) {
    Serial.println(" - Failed to find HUSB238 chip at 0x42");
  } else {
    Serial.println(" - HUSB238 at 0x42 found! Resetting...");
    husb_x42.reset();
  }

  if (!husb_x62.begin(HUSB_X62_ADDR, &Wire)) {
    Serial.println(" - Failed to find HUSB238 chip at 0x62");
  } else {
    Serial.println(" - HUSB238 at 0x62 found! Resetting...");
    husb_x62.reset();
  }
  delay(500); // Give chips time to reset and negotiate

  // Here you can uncomment to select a default voltage on startup if desired.
  // husb_x42.selectPD(PD_SRC_20V);
  // husb_x42.requestPD();
  // husb_x62.selectPD(PD_SRC_20V);
  // husb_x62.requestPD();

  // motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
}

void loop() {
  // motor.move();
  // motor.loopFOC();
  // motor.monitor();
  // command.run();

  // Call the detailed status functions for each chip
  print_husb_status(husb_x42, "HUSB @ 0x42");
  // Get the raw register values to see what the hardware is actually reporting
  dump_husb_registers(HUSB_X42_ADDR, "HUSB @ 0x42");
  
  Serial.println(); // Spacer

  print_husb_status(husb_x62, "HUSB @ 0x62");
  dump_husb_registers(HUSB_X62_ADDR, "HUSB @ 0x62");
  
  Serial.println("----------------------------------------------");

  delay(5000); // Check status every 5 seconds
}

