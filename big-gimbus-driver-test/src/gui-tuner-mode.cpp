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

const int SPI_CLK = 21;
const int SPI_CIPO = 48;
const int SPI_COPI = 47;
const int DRV_CS = 14;
float rds_on_shunt = 0.1;
float current_gain = .15;

// BLDC motor instance
int pole_pairs = 11;      // number of pole pairs
int kv = 24;              // rpm/V
float resistance = 6.900; // phase resistance
BLDCMotor motor = BLDCMotor(pole_pairs, resistance, kv);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(UH, UL, VH, VL, WH, WL, DRV_CS, false, NOT_SET, NOT_SET);
LowsideCurrentSense i_sense_motor = LowsideCurrentSense(rds_on_shunt, current_gain, isen_u, isen_v, isen_w);

// create commander for serial interface to simplefocStudio
Commander command = Commander(Serial);
void doMotor(char* cmd) {command.motor(&motor, cmd);}

// motor setup
bool setup_motor(){
  SimpleFOCDebug::enable();
  hspi.begin(enc_scl, enc_sda, enc_cs);
  SPI.begin(SPI_CLK, SPI_CIPO, SPI_COPI, DRV_CS);
  delay(300);
  sensor.init(&hspi);
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

  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // Limits 
  motor.velocity_limit = 100.0; // 100 rad/s velocity limit
  motor.voltage_limit = 2.0;    // 2 Volt limit 
  motor.current_limit = 1.0;    // 2 Amp current limit

  // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P  = 5;
  motor.PID_current_q.I  = 300;
  motor.PID_current_d.P  = 5;
  motor.PID_current_d.I  = 300;
  motor.LPF_current_q.Tf = 0.01;
  motor.LPF_current_d.Tf = 0.01;

  Serial.println("Intiializing motor:");
  delay(2000);
  motor.init();
  Serial.println("motor initialized");

  Serial.println("initializing current sense");
  i_sense_motor.init();

  Serial.println("linking current sense");
  motor.linkCurrentSense(&i_sense_motor);
  Serial.println("initFOC");
  motor.initFOC();

  // setup commander id and logginge
  char motor_id = 'a';
  command.add(motor_id, doMotor, "motor");
  // tell motor to use monitoring
  motor.useMonitoring(Serial);
  motor.monitor_start_char = motor_id;
  motor.monitor_end_char = motor_id;
  command.verbose = VerboseMode::machine_readable;

  return true;
}

CRGB ind[1];

bool setup_indicator(){
  pinMode(IND_LIGHT, OUTPUT);
  FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
  
  // Seed the random number generator using an unconnected analog pin.
  // This ensures you get a different "random" color every time you boot.
  // IMPORTANT: Change A0 to any analog-capable pin you are not using.
  randomSeed(esp_random()); 

  // Set the LED to a random, bright, "fun" color
  // We use the HSV color space:
  // CHSV(HUE, SATURATION, VALUE)
  // Hue (0-255): The color. We pick this randomly.
  // Saturation (0-255): The "purity" of the color. 255 is max fun.
  // Value (0-255): The brightness. 255 is max bright.
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
  while (!Serial);
  delay(1000);
  
  Serial.println("setting up indicator");
  setup_indicator();
  Serial.println("Gimbus test");
  setup_motor();
  Serial.println("setting up i2c");
  setup_i2c();
  Serial.println("waiting and beginning loop");

  // Configure which monitoring variables to log
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
  delay(5000);
}

void loop() {

  motor.move();
  motor.loopFOC();
  motor.monitor();
  command.run();

  // Serial.println("Motor Driver SPI Check:")
}
