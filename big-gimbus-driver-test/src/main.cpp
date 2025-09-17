#include <Arduino.h>
#include <Fastled.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>

#include "define_pins.h"

// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);

// BLDC motor instance
int pole_pairs = 11; // number of pole pairs
int kv = 24; // rpm/V
float resistance = 6.900; // phase resistance
BLDCMotor motor = BLDCMotor(pole_pairs, resistance, kv);
BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
float csa_gain = 0.15; // Volts per Amp
LowsideCurrentSense i_sense_motor = LowsideCurrentSense(csa_gain, 1.0f, isen_u, isen_v, isen_w); 

// motor setup
bool setup_motor(){
  hspi.begin(enc_scl, enc_sda, enc_cs);
  delay(300);
  sensor.init(&hspi);
  motor.linkSensor(&sensor);
  
  driver.voltage_power_supply = 8; // voltage of power supply [V]
  driver.pwm_frequency = 30000; // 30Khz, above hearing
  driver.init();
  motor.linkDriver(&driver);
  i_sense_motor.linkDriver(&driver);

  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
    // velocity loop PID
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0.01;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 1.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motor.PID_current_q.P = 1.0;
  motor.PID_current_q.I = 10.0;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.02;
  // current d loop PID
  motor.PID_current_d.P = 1.0;
  motor.PID_current_d.I = 10.0;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.02;

  // Limits 
  motor.velocity_limit = 100.0; // 100 rad/s velocity limit
  motor.voltage_limit = 12.0;   // 12 Volt limit 
  motor.current_limit = 1.0;    // 2 Amp current limit

  motor.init();
  i_sense_motor.init();
  motor.linkCurrentSense(&i_sense_motor);
  motor.LPF_current_q.Tf = 0.05;
  motor.initFOC();

  return true;
}

CRGB ind[1];

bool setup_indicator(){
  pinMode(IND_LIGHT, OUTPUT);
  FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
  ind[0] = CRGB::Blue;
  // FastLED.setBrightness(75);
  FastLED.show();
  return true;
}

PhaseCurrent_s i_phase;
void setup() {
  Serial.begin(5000000);
  while (!Serial);
  delay(1000);
  Serial.println("Gimbus test");
  setup_motor();
  setup_indicator();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("motor shaft angle: ");
  Serial.print(motor.shaftAngle());
  Serial.print("  - phase A Motor current: ");
  Serial.println(i_sense_motor.getDCCurrent());
  motor.move(1.0);
  motor.loopFOC();
}
