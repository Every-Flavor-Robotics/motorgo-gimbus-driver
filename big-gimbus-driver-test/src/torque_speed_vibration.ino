#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/calibrated/CalibratedSensor.h"

// this script automatically does reeling, and switches between velocity and torque control
// it has reeling implemented to automatically stop when it is fully reeled in
// ONLY FOR 6823F MOTORS

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 7);
BLDCMotor motor = BLDCMotor(11, 5.8, 24);  //6823
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5);
// BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 6); //left back
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

const float voltage_limit = 12.0;
float speed_voltage_limit = 2;
float speed_current_limit = 0.2;
float torque_voltage_limit = 12;
float torque_current_limit = 1.2;
float maxCurr = 1.3f;
float speed = 0.0;
float target = 10.0;

//reeling parameters
bool noslack = false;
bool reeling = false;
bool reelPause = false;
bool umove = false;
float speedThreshold = 5.0;
float reelDelay = 200; 
float pauseDelay = 300; 
unsigned long pauseTime;
unsigned long lastMoveTime;

//vibration parameters
bool vibration = false;
float curr1;
float curr2;
bool curr1on = true;
int frequency;
unsigned long switchperiod = 0;
unsigned long vib_prevMillis = 0;
unsigned long vib_currMillis;

char command[64];
char type;
float value;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  SimpleFOCDebug::enable(&Serial);
  sensor.init();

  // Initialize driver
  driver.voltage_power_supply = voltage_limit;
  driver.init();
  motor.linkDriver(&driver);
  
  // aligning voltage 
  motor.voltage_sensor_align = 8;

  // PID tuning
  motor.PID_velocity.P = 0.2;  
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;

  motor.init();
  // Run calibration
  // sensor_calibrated.voltage_calibration = 5;
  // sensor_calibrated.calibrate(motor); 
  // motor.linkSensor(&sensor_calibrated);
  // After calibration
  motor.zero_electric_angle = 3.57;
  motor.sensor_direction = Direction::CCW;
  motor.linkSensor(&sensor);
  // left front: 5 - 0.41 CW / 1.78 CCW
  // left hip: 3 - 3.86 - CW / 4.53 CCW
  // left back: 8 - 2.11 - CCW  / CHANGE PWM TO 6
  // back: 9 - 0.61 - CCW  
  // right back: 11 - 4.74 CCW
  // right hip: 13 - 3.57 - CCW 
  // right front: 35 - 1.2 - CCW 
  // test: 0.48 CCW

  motor.initFOC();
  // Serial.println(F("Motor ready."));
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = speed_voltage_limit;
  motor.current_limit = speed_current_limit;
  noslack = true;
  reeling = true;
  lastMoveTime = millis(); 
}

void loop() {
  motor.loopFOC();
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', command, 63);
    command[len] = '\0'; 
    char *token = strtok(command, " ");
    char* tokens[4]; // Array to hold up to 4 tokens
    int tokenCount = 0;
    while (token != NULL && tokenCount < 4) {
      tokens[tokenCount++] = token;
      token = strtok(NULL, " ");
    }
    type = tokens[0][0];
    if (tokenCount == 4) {
      curr1 = atof(tokens[1]);
      curr2 = atof(tokens[2]);
      if (curr1 >=0){ curr1 = min(curr1, maxCurr); }
      else{ curr1 = max(curr1, -maxCurr); }
      if (curr2 >=0){ curr2 = min(curr2, maxCurr); }
      else{ curr2 = max(curr2, -maxCurr); }
      switchperiod = 1000/(atoi(tokens[3]));
    }else if (tokenCount == 2) {
      value = atof(tokens[1]);
    }

    if (type == 'S') {    // HIGH SPEED MODE, REELING
      target = value;
      motor.enable();
      motor.controller = MotionControlType::velocity;
      motor.voltage_limit = speed_voltage_limit;
      motor.current_limit = speed_current_limit;
      vibration = false;
      noslack = true;
      reeling = true;
      lastMoveTime = millis(); 
    } else if (type == 'A') {    // ALWAYS REEL
      if (value >=0){ target = min(value, maxCurr); }
      else{ target = max(value, -maxCurr); }
      motor.enable();
      motor.controller = MotionControlType::torque;
      motor.voltage_limit = 1;
      motor.current_limit = 0.1;
      noslack = false;
      vibration = false;
    } else if (type == 'T') {    // TORQUE VOLTAGE CONTROL MODE
      if (value >=0){ target = min(value, maxCurr); }
      else{ target = max(value, -maxCurr); }
      motor.enable();
      motor.controller = MotionControlType::torque;
      motor.voltage_limit = torque_voltage_limit;
      motor.current_limit = torque_current_limit;
      noslack = false;
      vibration = false;
    } else if ( type == 'B'){   // VIBRATION MODE
      motor.enable();
      motor.controller = MotionControlType::torque;
      motor.voltage_limit = torque_voltage_limit;
      motor.current_limit = torque_current_limit;
      noslack = false;
      vibration = true;
      vib_prevMillis = millis();
    } else if ( type == 'O'){    // MOTORS OFF
      motor.disable();
      noslack = false;
      vibration = false;
    } else if ( type == 'V'){    
      motor.voltage_limit = value;
    } else if ( type == 'C'){    
      motor.current_limit = value;
    } 
  }

  if (noslack){ // REELING WITH PAUSE
    speed = motor.shaftVelocity(); 
    if (reeling){
      if (speed > speedThreshold){ lastMoveTime = millis(); }
      if (millis() - lastMoveTime > reelDelay){ 
        motor.disable();
        reeling = false;
        reelPause = true;
        pauseTime = millis();
      } 
    } else if (reelPause){ 
      if (millis() - pauseTime > pauseDelay) { reelPause = false; }
    } else{
      if (speed < -speedThreshold){ umove = true; lastMoveTime = millis(); }
      if (umove && millis() - lastMoveTime > reelDelay){ 
        motor.enable();
        umove = false;
        reeling = true;
        lastMoveTime = millis();
      } 
    }
  } else if (vibration){
    vib_currMillis = millis();
    if (vib_currMillis - vib_prevMillis >= switchperiod/2) {
      vib_prevMillis = vib_currMillis;
      if (curr1on) { target = curr1; } 
      else { target = curr2; }
      curr1on = !curr1on;
    }
  }
  motor.move(target); 

  // Serial.print(reeling);
  // Serial.print(reelPause);
  // Serial.print(type);
  // Serial.print("\t");
  // Serial.print(curr1);
  // Serial.print("\t");
  // Serial.print(curr2);
  // Serial.print("\t");
  // Serial.print(switchperiod);
  // Serial.print("\t");
  // Serial.println(vib_currMillis);
  // Serial.print(motor.voltage_limit);
  // Serial.print("\t");
  // Serial.print(millis() - lastMoveTime);
  // Serial.print("\t");
  // Serial.println(motor.shaftVelocity());

}
