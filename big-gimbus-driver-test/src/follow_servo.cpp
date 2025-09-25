#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <drivers/drv8316/drv8316.h>
#include <driver/twai.h>
#include "define_pins.h"
#include "can_protocol.h"

// !! SET THIS FOR EACH MOTOR BEFORE UPLOADING !!
// Choose from: MOTOR_LEFT_HAND_UP (1), MOTOR_LEFT_HAND_DOWN (2), 
// MOTOR_RIGHT_HAND_UP (3), MOTOR_RIGHT_HAND_DOWN (4), etc.
#define MY_MOTOR_ID MOTOR_LEFT_HAND_UP  // Change this!

// Firmware version for identification
#define FW_VERSION 0x0101  // Version 1.1

// Magnetic encoder MT6701 SPI
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

const int SPI_CLK = 21;
const int SPI_CIPO = 48;
const int SPI_COPI = 47;
const int DRV_CS = 14;

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
float target = 0.0;

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
unsigned long switchperiod = 0;
unsigned long vib_prevMillis = 0;
unsigned long vib_currMillis;

// LED indicator
CRGB ind[1];

// Status tracking
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 5000;  // Report status every 5 seconds
uint8_t motor_health_status = 0;  // 0 = OK, 1 = Error, 2 = Warning

// Ping statistics
uint32_t ping_count = 0;
uint32_t pong_count = 0;
unsigned long last_ping_time = 0;

bool setup_can() {
    // Use can_tx and can_rx from define_pins.h
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx, (gpio_num_t)can_rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // Hardware filter to accept messages for this motor AND broadcast messages
    // We need to accept both our specific ID and broadcast ID
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all for now to handle both

    Serial.println("Uninstalling USB Serial/JTAG driver...");
    esp_err_t jtag_uninstall_result = usb_serial_jtag_driver_uninstall();
    if (jtag_uninstall_result != ESP_OK) {
        Serial.printf("Failed to uninstall JTAG driver. Error: %s\n", esp_err_to_name(jtag_uninstall_result));
        // Note: Depending on the board, this might not be a fatal error.
        // The driver might already be uninstalled.
    } else {
        Serial.println("JTAG driver uninstalled successfully.");
    }
    
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("Failed to install TWAI driver");
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        Serial.println("Failed to start TWAI driver");
        return false;
    }
    
    Serial.printf("CAN Bus initialized for %s (ID: %d)\n", getMotorName(MY_MOTOR_ID), MY_MOTOR_ID);
    return true;
}

void sendPongResponse(uint32_t timestamp, uint8_t sequence) {
    twai_message_t message;
    message.identifier = create_can_id(MOTOR_CONTROLLER, CMD_PONG);  // Send pong back to controller
    message.flags = 0;
    
    CanPayloadPong pong;
    pong.timestamp = timestamp;
    pong.sequence = sequence;
    pong.motor_status = motor_health_status;
    pong.fw_version = FW_VERSION;
    
    message.data_length_code = sizeof(CanPayloadPong);
    memcpy(message.data, &pong, sizeof(CanPayloadPong));
    
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result == ESP_OK) {
        pong_count++;
        Serial.printf("PONG sent (seq: %d, timestamp: %lu)\n", sequence, timestamp);
    } else {
        Serial.println("Failed to send PONG");
    }
}

void processCANMessages() {
    twai_message_t message;
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(10));
    
    if (result != ESP_OK) {
        return;  // No message or error
    }
    
    uint8_t motor_id;
    CAN_COMMAND cmd;
    parse_can_id(message.identifier, motor_id, cmd);
    
    // Check if message is for us or broadcast
    if (motor_id != MY_MOTOR_ID && motor_id != MOTOR_BROADCAST) return;
    
    Serial.printf("Received command %d for %s\n", cmd, getMotorName(motor_id));
    
    switch(cmd) {
        case CMD_PING: {
            ping_count++;
            last_ping_time = millis();
            CanPayloadPing* ping = (CanPayloadPing*)message.data;
            Serial.printf("PING received (seq: %d, timestamp: %lu)\n", ping->sequence, ping->timestamp);
            
            // Send PONG response
            sendPongResponse(ping->timestamp, ping->sequence);
            
            // Flash LED to indicate ping received
            ind[0] = CRGB::White;
            FastLED.show();
            delay(50);
            uint8_t hue = (MY_MOTOR_ID * 23) % 256;
            ind[0] = CHSV(hue, 255, 255);
            FastLED.show();
            break;
        }
        
        case CMD_SET_VELOCITY: {
            CanPayloadFloat* payload = (CanPayloadFloat*)message.data;
            target = payload->value;
            motor.controller = MotionControlType::velocity;
            motor.voltage_limit = speed_voltage_limit;
            motor.current_limit = speed_current_limit;
            motor.enable();
            vibration = false;
            noslack = true;
            reeling = true;
            lastMoveTime = millis();
            Serial.printf("Velocity mode: %.2f\n", target);
            break;
        }
        
        case CMD_SET_TORQUE: {
            CanPayloadFloat* payload = (CanPayloadFloat*)message.data;
            float value = payload->value;
            if (value >= 0) { target = min(value, maxCurr); }
            else { target = max(value, -maxCurr); }
            motor.controller = MotionControlType::torque;
            motor.voltage_limit = torque_voltage_limit;
            motor.current_limit = torque_current_limit;
            motor.enable();
            noslack = false;
            vibration = false;
            Serial.printf("Torque mode: %.2f\n", target);
            break;
        }
        
        case CMD_SET_VIBRATION: {
            CanPayloadVibration* payload = (CanPayloadVibration*)message.data;
            vibration = true;
            noslack = false;
            // Unscale the currents
            curr1 = (float)payload->current1_scaled / 1000.0f;
            curr2 = (float)payload->current2_scaled / 1000.0f;
            switchperiod = 1000 / payload->frequency;
            motor.controller = MotionControlType::torque;
            motor.voltage_limit = torque_voltage_limit;
            motor.current_limit = torque_current_limit;
            motor.enable();
            vib_prevMillis = millis();
            Serial.printf("Vibration mode: %.3f/%.3f @ %dHz\n", curr1, curr2, payload->frequency);
            break;
        }
        
        case CMD_MOTOR_OFF: {
            motor.disable();
            vibration = false;
            noslack = false;
            Serial.println("Motor disabled");
            break;
        }
        
        case CMD_ALWAYS_REEL: {
            CanPayloadFloat* payload = (CanPayloadFloat*)message.data;
            float value = payload->value;
            if (value >= 0) { target = min(value, maxCurr); }
            else { target = max(value, -maxCurr); }
            motor.controller = MotionControlType::torque;
            motor.voltage_limit = 1;
            motor.current_limit = 0.1;
            motor.enable();
            noslack = false;
            vibration = false;
            Serial.printf("Always reel mode: %.2f\n", target);
            break;
        }
        
        case CMD_SET_VOLTAGE_LIM: {
            CanPayloadLimits* payload = (CanPayloadLimits*)message.data;
            motor.voltage_limit = payload->value;
            Serial.printf("Voltage limit: %.2f\n", payload->value);
            break;
        }
        
        case CMD_SET_CURRENT_LIM: {
            CanPayloadLimits* payload = (CanPayloadLimits*)message.data;
            motor.current_limit = payload->value;
            Serial.printf("Current limit: %.2f\n", payload->value);
            break;
        }
        
        case CMD_EMERGENCY_STOP: {
            motor.disable();
            vibration = false;
            noslack = false;
            target = 0;
            Serial.println("EMERGENCY STOP!");
            break;
        }
        
        case CMD_STATUS_REQUEST: {
            // Future implementation: Send status back via CAN
            Serial.println("Status requested");
            break;
        }
        
        case CMD_CALIBRATE: {
            // Trigger recalibration
            Serial.println("Calibration requested - not implemented yet");
            break;
        }
        
        default:
            Serial.printf("Unknown command: %d\n", cmd);
            break;
    }
}

bool setup_indicator() {
    pinMode(IND_LIGHT, OUTPUT);
    FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
    
    // Use different colors for different motor positions for easy identification
    uint8_t hue = (MY_MOTOR_ID * 23) % 256;  // Spread colors across spectrum
    ind[0] = CHSV(hue, 255, 255);
    FastLED.show();
    return true;
}

bool setup_motor() {
    SimpleFOCDebug::enable();
    motor.useMonitoring(Serial);
     
    hspi.begin(enc_scl, enc_sda, enc_cs);
    SPI.begin(SPI_CLK, SPI_CIPO, SPI_COPI, DRV_CS);
    delay(300);
    
    sensor.init(&hspi);
    motor.linkSensor(&sensor);
    
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 20000;
    driver.init(&SPI);
    
    driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V15);
    driver.setSlew(DRV8316_Slew::Slew_200Vus);
    driver.setPWMMode(DRV8316_PWMMode::PWM6_Mode);
    driver.setPWM100Frequency(DRV8316_PWM100DUTY::FREQ_20KHz);
    
    motor.linkDriver(&driver);
    i_sense_motor.linkDriver(&driver);
    
    motor.voltage_sensor_align = 6;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    
    motor.voltage_limit = 4.0;
    motor.current_limit = 1.2;
    
    // Current control PID parameters (from tuning)
    motor.PID_current_d.P = .25;
    motor.PID_current_d.I = 0;
    motor.PID_current_d.D = 0;
    motor.PID_current_d.output_ramp = 0;
    motor.PID_current_d.limit = 12;
    motor.LPF_current_d.Tf = 0.01;
    
    motor.PID_current_q.P = 1;
    motor.PID_current_q.I = 0;
    motor.PID_current_q.D = 0;
    motor.PID_current_q.output_ramp = 100;
    motor.PID_current_q.limit = 12;
    motor.LPF_current_q.Tf = 0.01;
    
    motor.PID_velocity.P = .75;
    motor.PID_velocity.I = .075;
    motor.PID_velocity.D = 0.001;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.05;
    
    motor.P_angle.P = 20;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    
    motor.velocity_limit = 100.0;
    
    Serial.println("Initializing motor...");
    delay(2000);
    motor.init();
    
    i_sense_motor.init();
    motor.linkCurrentSense(&i_sense_motor);
    
    motor.initFOC();
    
    Serial.println("Calibrating sensor for eccentricity compensation...");
    Serial.println("Motor will rotate slowly for calibration.");
  
    sensor_calibrated.voltage_calibration = 3;

    // Perform calibration - motor will slowly rotate
    if (!sensor_calibrated.loadCalibration(motor)){
        sensor_calibrated.calibrate(motor, 500);    
        sensor_calibrated.saveCalibration(motor);
    }
    Serial.println("Calibration complete!");
  
     // NOW link the calibrated sensor to the motor
    motor.linkSensor(&sensor_calibrated);
  
    // Re-initialize FOC with calibrated sensor
    Serial.println("Re-initializing FOC with calibrated sensor...");
    motor.initFOC();
    
    motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_Q | _MON_CURR_Q;
    
    Serial.println("Motor initialized");
    return true;
}

bool setup_i2c() {
    Wire.begin(i2c_sda, i2c_scl);
    delay(300);
    Serial.println("I2C initialized");
    return true;
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

void printStatus() {
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        lastStatusTime = millis();
        Serial.printf("\n=== %s Status ===\n", getMotorName(MY_MOTOR_ID));
        Serial.printf("Mode: ");
        if (motor.enabled) {
            if (vibration) Serial.print("Vibration");
            else if (noslack) Serial.print("Reeling");
            else if (motor.controller == MotionControlType::torque) Serial.print("Torque");
            else if (motor.controller == MotionControlType::velocity) Serial.print("Velocity");
            else Serial.print("Unknown");
        } else {
            Serial.print("Disabled");
        }
        Serial.printf("\nTarget: %.2f, Velocity: %.2f rad/s\n", target, motor.shaftVelocity());
        Serial.printf("Voltage limit: %.2f, Current limit: %.2f\n", motor.voltage_limit, motor.current_limit);
        Serial.printf("Ping stats: Received: %lu, Responded: %lu\n", ping_count, pong_count);
        if (last_ping_time > 0) {
            Serial.printf("Last ping: %lu ms ago\n", millis() - last_ping_time);
        }
        Serial.println("==================\n");
    }
}

void setup() {
    Serial.begin(115200);
    delay(5000);
    
    Serial.println("=================================");
    Serial.printf("=== %s NODE STARTING ===\n", getMotorName(MY_MOTOR_ID));
    Serial.printf("=== Motor ID: %d ===\n", MY_MOTOR_ID);
    Serial.printf("=== Firmware: v%d.%d ===\n", (FW_VERSION >> 8) & 0xFF, FW_VERSION & 0xFF);
    Serial.println("=================================\n");
    
    setup_indicator();
    setup_motor();
    setup_i2c();
    
    if (!setup_can()) {
        Serial.println("WARNING: CAN setup failed!");
        motor_health_status = 1;  // Set error status
    }
    
    motor.disable();
    
    Serial.println("\n=== Follower Node Ready ===");
    Serial.println("Waiting for CAN commands...\n");
}


// Test message tracking  
unsigned long last_test_send = 0;
uint32_t test_seq = 0;
uint32_t test_received = 0;

void loop() {

    // Check for new CAN commands
    processCANMessages();
    
    // Core motor control
    motor.loopFOC();
    
    // Handle special modes
    if (noslack) {
        handleReeling();
    } else if (vibration) {
        handleVibration();
    }
    
    // Move motor with target
    motor.move(target);
    
    // Print periodic status updates
    printStatus();
}