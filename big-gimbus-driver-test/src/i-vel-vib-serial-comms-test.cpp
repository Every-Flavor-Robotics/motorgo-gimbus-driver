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
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

const int SPI_CLK = 21;
const int SPI_CIPO = 48;
const int SPI_COPI = 47;
const int DRV_CS = 14;
float rds_on_shunt = 1.0f;
float current_gain = 0.15f;

// BLDC motor instance
int pole_pairs = 11;
int kv = 24;
float resistance = 6.900;
BLDCMotor motor = BLDCMotor(pole_pairs, resistance, kv);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(UH, UL, VH, VL, WH, WL, DRV_CS, false, NOT_SET, NOT_SET);
LowsideCurrentSense i_sense_motor = LowsideCurrentSense(150, isen_u, isen_v, isen_w);

// Control mode parameters
const float voltage_limit = 12.0;
float speed_voltage_limit = 2;
float speed_current_limit = 0.2;
float torque_voltage_limit = 12;
float torque_current_limit = 1.2;
float maxCurr = 1.3f;
float speed = 0.0;
float target = 10.0;

// Reeling parameters
bool noslack = false;
bool reeling = false;
bool reelPause = false;
bool umove = false;
float speedThreshold = 5.0;
float reelDelay = 200;
float pauseDelay = 300;
unsigned long pauseTime;
unsigned long lastMoveTime;

// Vibration parameters
bool vibration = false;
float curr1;
float curr2;
bool curr1on = true;
int frequency;
unsigned long switchperiod = 0;
unsigned long vib_prevMillis = 0;
unsigned long vib_currMillis;

// Command handling
char command[64];
char type;
float value;

// LED indicator
CRGB ind[1];

bool setup_indicator() {
  pinMode(IND_LIGHT, OUTPUT);
  FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
  randomSeed(esp_random());
  ind[0] = CHSV(random(0, 256), 255, 255);
  FastLED.show();
  return true;
}

bool setup_motor() {
  SimpleFOCDebug::enable();
  motor.useMonitoring(Serial);
  
  hspi.begin(enc_scl, enc_sda, enc_cs);
  SPI.begin(SPI_CLK, SPI_CIPO, SPI_COPI, DRV_CS);
  delay(300);
  
  // Initialize raw sensor
  sensor.init(&hspi);
  
  // Link the raw sensor to the motor first (for calibration)
  motor.linkSensor(&sensor);
  
  driver.voltage_power_supply = 15;
  driver.pwm_frequency = 20000;
  driver.init(&SPI);
  
  // Set motor driver settings
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
  
  // Calibration
  sensor_calibrated.voltage_calibration = 6;
  Serial.println("Calibrating sensor for eccentricity compensation...");
  Serial.println("Motor will rotate slowly for calibration.");
  sensor_calibrated.calibrate(motor, 300);
  Serial.println("Calibration complete!");
  
  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);
  
  // Re-initialize FOC with calibrated sensor
  Serial.println("Re-initializing FOC with calibrated sensor...");
  motor.initFOC();
  Serial.println("FOC with calibrated sensor complete");
  
  // Configure monitoring
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
  
  return true;
}

bool setup_i2c() {
  Serial.println("Trying to start I2C");
  Wire.begin(i2c_sda, i2c_scl);
  delay(300);
  Serial.println("I2C init success");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  
  Serial.println("Setting up indicator");
  setup_indicator();
  Serial.println("Motor control with tuned parameters");
  setup_motor();
  Serial.println("Setting up i2c");
  setup_i2c();
  
  // Initial state - start with motor disabled
  motor.disable();
  noslack = false;
  vibration = false;
  
  Serial.println("Ready for commands");
  Serial.println("Commands: S (speed/reel), A (always reel), T (torque), B (vibration), O (off), V (voltage limit), C (current limit)");
}

void processCommand() {
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', command, 63);
    command[len] = '\0';
    
    char *token = strtok(command, " ");
    char* tokens[4];
    int tokenCount = 0;
    
    while (token != NULL && tokenCount < 4) {
      tokens[tokenCount++] = token;
      token = strtok(NULL, " ");
    }
    
    type = tokens[0][0];
    
    if (tokenCount == 4) {
      // Vibration mode with 4 parameters
      curr1 = atof(tokens[1]);
      curr2 = atof(tokens[2]);
      if (curr1 >= 0) { curr1 = min(curr1, maxCurr); }
      else { curr1 = max(curr1, -maxCurr); }
      if (curr2 >= 0) { curr2 = min(curr2, maxCurr); }
      else { curr2 = max(curr2, -maxCurr); }
      switchperiod = 1000 / (atoi(tokens[3]));
    } else if (tokenCount == 2) {
      value = atof(tokens[1]);
    }
    
    switch(type) {
      case 'S':  // HIGH SPEED MODE, REELING
        target = value;
        motor.enable();
        motor.controller = MotionControlType::velocity;
        motor.voltage_limit = speed_voltage_limit;
        motor.current_limit = speed_current_limit;
        vibration = false;
        noslack = true;
        reeling = true;
        lastMoveTime = millis();
        Serial.println("Speed/Reel mode activated");
        break;
        
      case 'A':  // ALWAYS REEL
        if (value >= 0) { target = min(value, maxCurr); }
        else { target = max(value, -maxCurr); }
        motor.enable();
        motor.controller = MotionControlType::torque;
        motor.voltage_limit = 1;
        motor.current_limit = 0.1;
        noslack = false;
        vibration = false;
        Serial.println("Always reel mode activated");
        break;
        
      case 'T':  // TORQUE CONTROL MODE
        if (value >= 0) { target = min(value, maxCurr); }
        else { target = max(value, -maxCurr); }
        motor.enable();
        motor.controller = MotionControlType::torque;
        motor.voltage_limit = torque_voltage_limit;
        motor.current_limit = torque_current_limit;
        noslack = false;
        vibration = false;
        Serial.println("Torque mode activated");
        break;
        
      case 'B':  // VIBRATION MODE
        motor.enable();
        motor.controller = MotionControlType::torque;
        motor.voltage_limit = torque_voltage_limit;
        motor.current_limit = torque_current_limit;
        noslack = false;
        vibration = true;
        vib_prevMillis = millis();
        Serial.println("Vibration mode activated");
        break;
        
      case 'O':  // MOTORS OFF
        motor.disable();
        noslack = false;
        vibration = false;
        Serial.println("Motor disabled");
        break;
        
      case 'V':  // SET VOLTAGE LIMIT
        motor.voltage_limit = value;
        Serial.print("Voltage limit set to: ");
        Serial.println(value);
        break;
        
      case 'C':  // SET CURRENT LIMIT
        motor.current_limit = value;
        Serial.print("Current limit set to: ");
        Serial.println(value);
        break;
        
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

void handleReeling() {
  speed = motor.shaftVelocity();
  
  if (reeling) {
    if (speed > speedThreshold) { 
      lastMoveTime = millis(); 
    }
    if (millis() - lastMoveTime > reelDelay) {
      motor.disable();
      reeling = false;
      reelPause = true;
      pauseTime = millis();
    }
  } else if (reelPause) {
    if (millis() - pauseTime > pauseDelay) { 
      reelPause = false; 
    }
  } else {
    if (speed < -speedThreshold) { 
      umove = true; 
      lastMoveTime = millis(); 
    }
    if (umove && millis() - lastMoveTime > reelDelay) {
      motor.enable();
      umove = false;
      reeling = true;
      lastMoveTime = millis();
    }
  }
}

void handleVibration() {
  vib_currMillis = millis();
  if (vib_currMillis - vib_prevMillis >= switchperiod / 2) {
    vib_prevMillis = vib_currMillis;
    if (curr1on) { 
      target = curr1; 
    } else { 
      target = curr2; 
    }
    curr1on = !curr1on;
  }
}

void loop() {
  // Core motor control
  motor.loopFOC();
  
  // Process serial commands
  processCommand();
  
  // Handle special modes
  if (noslack) {
    handleReeling();
  } else if (vibration) {
    handleVibration();
  }
  
  // Move motor with target
  motor.move(target);
  
  // Optional monitoring (comment out for production)
  // motor.monitor();
}