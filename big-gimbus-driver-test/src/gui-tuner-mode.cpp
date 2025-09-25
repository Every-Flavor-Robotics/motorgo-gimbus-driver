#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <drivers/drv8316/drv8316.h>
#include "define_pins.h"

// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);

// Calibrated sensor wrapper - this will handle eccentricity
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

const int SPI_CLK = 21;
const int SPI_CIPO = 48;
const int SPI_COPI = 47;
const int DRV_CS = 14;

float rds_on_shunt = 1.0f;
float current_gain = 0.15f;

// BLDC motor instance
int pole_pairs = 11;      // number of pole pairs
int kv = 24;              // rpm/V
float resistance = 6.900; // phase resistance
BLDCMotor motor = BLDCMotor(pole_pairs, resistance, kv);

DRV8316Driver6PWM driver = DRV8316Driver6PWM(UH, UL, VH, VL, WH, WL, DRV_CS, false, NOT_SET, NOT_SET);
LowsideCurrentSense i_sense_motor = LowsideCurrentSense(150, isen_u, isen_v, isen_w);

// create commander for serial interface to simplefocStudio
Commander command = Commander(Serial);
void doMotor(char* cmd) {command.motor(&motor, cmd);}

// motor setup
bool setup_motor(){
  SimpleFOCDebug::enable();
  motor.useMonitoring(Serial);
  hspi.begin(enc_scl, enc_sda, enc_cs);
  SPI.begin(SPI_CLK, SPI_CIPO, SPI_COPI, DRV_CS);
  delay(300);
  
  // Initialize raw sensor only
  sensor.init(&hspi);
  
  // Link the raw sensor to the motor first (for calibration)
  motor.linkSensor(&sensor);
  
  driver.voltage_power_supply = 15; // voltage of power supply [V]
  driver.pwm_frequency = 20000; 
  driver.init(&SPI);
  
  // set motor driver settings
  driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V15);
  driver.setSlew(DRV8316_Slew::Slew_200Vus);
  driver.setPWMMode(DRV8316_PWMMode::PWM6_Mode);
  driver.setPWM100Frequency(DRV8316_PWM100DUTY::FREQ_20KHz);
  
  motor.linkDriver(&driver);
  i_sense_motor.linkDriver(&driver);
  
// Motor configuration
  motor.voltage_sensor_align = 5;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;  // Using FOC current as tuned
  motor.controller = MotionControlType::torque;  // Start in torque mode
  
  // Voltage and current limits (from tuning)
  motor.voltage_limit = 4.0;  // From tuning parameters
  motor.current_limit = 1.2;  // Adjusted for operation
  
  // Current control PID parameters (from tuning)
  // Current D gains: 0, 0, 0
  motor.PID_current_d.P = 0;
  motor.PID_current_d.I = 0;
  motor.PID_current_d.D = 0;
  motor.PID_current_d.output_ramp = 0;
  motor.PID_current_d.limit = 12;
  motor.LPF_current_d.Tf = 0.01;
  
  // Current Q gains: 1, 1, 0
  motor.PID_current_q.P = 1;
  motor.PID_current_q.I = 1;
  motor.PID_current_q.D = 0;
  motor.PID_current_q.output_ramp = 100;
  motor.PID_current_q.limit = 12;
  motor.LPF_current_q.Tf = 0.01;
  
  // Velocity PID parameters (from tuning)
  motor.PID_velocity.P = 4;
  motor.PID_velocity.I = 3;
  motor.PID_velocity.D = 0.015;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.05;
  
  // Position PID parameters (from tuning)
  motor.P_angle.P = 20;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;
  
  // Velocity limit
  motor.velocity_limit = 100.0;
  
  Serial.println("Initializing motor:");
  delay(2000);
  motor.init();
  Serial.println("Motor initialized");
  
  Serial.println("Initializing current sense");
  i_sense_motor.init();
  Serial.println("Linking current sense");
  motor.linkCurrentSense(&i_sense_motor);
  
  // Initialize FOC with raw sensor first
  Serial.println("Initial FOC calibration with raw sensor...");
  motor.initFOC();
  Serial.println("Initial FOC complete");
  
  // Set calibration voltage
  sensor_calibrated.voltage_calibration = 6;
  
  Serial.println("Calibrating sensor for eccentricity compensation...");
  Serial.println("Motor will rotate slowly for calibration.");
  
  // Perform calibration - motor will slowly rotate
  sensor_calibrated.calibrate(motor, 300);
  
  Serial.println("Calibration complete!");
  
  // NOW link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);
  
  // Re-initialize FOC with calibrated sensor
  Serial.println("Re-initializing FOC with calibrated sensor...");
  motor.initFOC();
  Serial.println("FOC with calibrated sensor complete");
  
  // setup commander id and logging
  char motor_id = 'a';
  command.add(motor_id, doMotor, "motor");
  
  // tell motor to use monitoring
  motor.monitor_start_char = motor_id;
  motor.monitor_end_char = motor_id;
  command.verbose = VerboseMode::machine_readable;
  
  return true;
}

CRGB ind[1];

bool setup_indicator(){
  pinMode(IND_LIGHT, OUTPUT);
  FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
  
  // Seed the random number generator
  randomSeed(esp_random()); 
  // Set the LED to a random, bright, "fun" color
  ind[0] = CHSV(random(0, 256), 255, 255);
  
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
  // while (!Serial);
  delay(5000);
  
  Serial.println("Setting up indicator");
  setup_indicator();
  Serial.println("Gimbus test");
  setup_motor();
  Serial.println("Setting up i2c");
  setup_i2c();
  Serial.println("Waiting and beginning loop");
  
  // Configure which monitoring variables to log
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
  delay(5000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  motor.monitor();
  command.run();
}