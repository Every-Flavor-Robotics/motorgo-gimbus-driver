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
#include "SPIFFS.h"

// !! CONTROLLER NODE - ID 0 !!
#define MY_MOTOR_ID MOTOR_CONTROLLER

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
float maxCurr = 1.0f;
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
unsigned long switchperiod = 0;
unsigned long vib_prevMillis = 0;
unsigned long vib_currMillis;

// Command handling
char command[64];
char type;
float value;

// LED indicator
CRGB ind[1];

// Ping/Pong tracking
struct PingStatus {
    uint8_t motor_id;
    uint8_t sequence;
    uint32_t sent_time;
    uint32_t response_time;
    bool responded;
    uint8_t motor_status;
    uint16_t fw_version;
};

PingStatus ping_status[12];  // Track ping status for each motor (1-11)
uint8_t ping_sequence = 0;
const unsigned long PING_TIMEOUT = 100;  // 100ms timeout for ping responses
bool ping_test_active = false;

// CAN command forwarding
struct MotorCommand {
    uint8_t motor_id;
    CAN_COMMAND cmd;
    union {
        float value;
        struct {
            float curr1;
            float curr2;
            uint16_t freq;
        } vibration;
    } data;
};

bool setup_can() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx, (gpio_num_t)can_rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
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
    
    Serial.println("CAN Bus initialized as Controller");
    return true;
}

void sendCANCommand(uint8_t motor_id, CAN_COMMAND cmd, void* payload, size_t payload_size) {
    twai_message_t message;
    message.identifier = create_can_id(motor_id, cmd);
    message.flags = 0;
    message.data_length_code = payload_size;
    
    if (payload && payload_size > 0) {
        memcpy(message.data, payload, min(payload_size, (size_t)8));
    }
    
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result != ESP_OK) {
        Serial.printf("Failed to send CAN message to motor %d (cmd: %d)\n", motor_id, cmd);
    } else {
        Serial.printf("Sent command %d to %s\n", cmd, getMotorName(motor_id));
    }
}

void broadcastCommand(CAN_COMMAND cmd, void* payload, size_t payload_size) {
    for (uint8_t id = 1; id <= 11; id++) {
        sendCANCommand(id, cmd, payload, payload_size);
        delayMicroseconds(100);
    }
}

void sendPing(uint8_t motor_id) {
    twai_message_t message;
    message.identifier = create_can_id(motor_id, CMD_PING);
    message.flags = 0;
    
    CanPayloadPing ping;
    ping.timestamp = millis();
    ping.sequence = ping_sequence++;
    
    message.data_length_code = sizeof(CanPayloadPing);
    memcpy(message.data, &ping, sizeof(CanPayloadPing));
    
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result == ESP_OK) {
        if (motor_id <= 11) {
            ping_status[motor_id].motor_id = motor_id;
            ping_status[motor_id].sequence = ping.sequence;
            ping_status[motor_id].sent_time = ping.timestamp;
            ping_status[motor_id].responded = false;
        }
        Serial.printf("PING sent to %s (seq: %d)\n", getMotorName(motor_id), ping.sequence);
    } else {
        Serial.printf("Failed to send PING to motor %d\n", motor_id);
    }
}

void processPongResponse() {
    twai_message_t message;
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(0));  // Non-blocking
    
    if (result != ESP_OK) return;
    
    uint8_t motor_id;
    CAN_COMMAND cmd;
    parse_can_id(message.identifier, motor_id, cmd);
    
    if (cmd == CMD_PONG) {
        CanPayloadPong* pong = (CanPayloadPong*)message.data;
        uint32_t latency = millis() - pong->timestamp;
        
        // Find which motor this is from based on sequence
        for (int i = 1; i <= 11; i++) {
            if (ping_status[i].sequence == pong->sequence && !ping_status[i].responded) {
                ping_status[i].responded = true;
                ping_status[i].response_time = millis();
                ping_status[i].motor_status = pong->motor_status;
                ping_status[i].fw_version = pong->fw_version;
                
                Serial.printf("PONG from %s (seq: %d, latency: %lums, status: %d, fw: v%d.%d)\n", 
                    getMotorName(i), 
                    pong->sequence, 
                    latency,
                    pong->motor_status,
                    (pong->fw_version >> 8) & 0xFF,
                    pong->fw_version & 0xFF);
                break;
            }
        }
    }
}

void runPingTest() {
    Serial.println("\n=== Starting CAN Bus Ping Test ===");
    ping_test_active = true;
    
    // Clear previous results
    for (int i = 0; i <= 11; i++) {
        ping_status[i].responded = false;
        ping_status[i].motor_status = 0;
        ping_status[i].fw_version = 0;
    }
    
    // Send ping to each motor
    for (uint8_t id = 1; id <= 11; id++) {
        sendPing(id);
        delay(10);
        
        // Process any immediate responses
        for (int j = 0; j < 5; j++) {
            processPongResponse();
            delay(2);
        }
    }
    
    // Wait for remaining responses
    unsigned long start_wait = millis();
    while (millis() - start_wait < PING_TIMEOUT) {
        processPongResponse();
        delay(1);
    }
    
    // Print results
    Serial.println("\n=== Ping Test Results ===");
    Serial.println("Motor            | Status  | Latency | FW Ver  | Health");
    Serial.println("-----------------|---------|---------|---------|--------");
    
    int responded_count = 0;
    for (uint8_t id = 1; id <= 11; id++) {
        if (ping_status[id].responded) {
            responded_count++;
            uint32_t latency = ping_status[id].response_time - ping_status[id].sent_time;
            const char* health = ping_status[id].motor_status == 0 ? "OK" : 
                                 ping_status[id].motor_status == 1 ? "ERROR" : "WARNING";
            Serial.printf("%-16s | OK      | %3lums   | v%d.%-4d | %s\n", 
                getMotorName(id), 
                latency,
                (ping_status[id].fw_version >> 8) & 0xFF,
                ping_status[id].fw_version & 0xFF,
                health);
        } else {
            Serial.printf("%-16s | NO RESP | ---     | ---     | ---\n", getMotorName(id));
        }
    }
    
    Serial.printf("\nSummary: %d/11 motors responded\n", responded_count);
    Serial.println("=========================\n");
    
    ping_test_active = false;
}

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
    
    motor.voltage_sensor_align = 1;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    
    motor.voltage_limit = 4.0;
    motor.current_limit = 1.2;
    
    // Current control PID parameters
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

void processLocalCommand(char cmd_type, char* tokens[], int tokenCount) {
    switch(cmd_type) {
        case 'S':  // Speed/Reel
            if (tokenCount >= 1) {
                target = atof(tokens[0]);
                motor.enable();
                motor.controller = MotionControlType::velocity;
                motor.voltage_limit = speed_voltage_limit;
                motor.current_limit = speed_current_limit;
                vibration = false;
                noslack = true;
                reeling = true;
                lastMoveTime = millis();
                Serial.print("Controller: Speed/Reel mode, target: ");
                Serial.println(target);
            }
            break;
            
        case 'T':  // Torque
            if (tokenCount >= 1) {
                value = atof(tokens[0]);
                if (value >= 0) { target = min(value, maxCurr); }
                else { target = max(value, -maxCurr); }
                motor.enable();
                motor.controller = MotionControlType::torque;
                motor.voltage_limit = torque_voltage_limit;
                motor.current_limit = torque_current_limit;
                noslack = false;
                vibration = false;
                Serial.println("Controller: Torque mode");
            }
            break;
            
        case 'O':  // Off
            target = 0;
            motor.disable();
            noslack = false;
            vibration = false;
            Serial.println("Controller: Motor off");
            break;
    }
}

void processSerialCommand() {
    if (!Serial.available()) return;
    
    int len = Serial.readBytesUntil('\n', command, 63);
    command[len] = '\0';
    
    // Check for ping commands
    if (strcmp(command, "PING") == 0 || strcmp(command, "ping") == 0) {
        runPingTest();
        return;
    }
    
    char* token = strtok(command, " ");
    if (!token) return;
    
    // Check for "PING <motor_id>" format
    if (strcmp(token, "PING") == 0 || strcmp(token, "ping") == 0) {
        token = strtok(NULL, " ");
        if (token) {
            if (strcmp(token, "ALL") == 0) {
                runPingTest();
            } else {
                uint8_t motor_id = atoi(token);
                if (motor_id >= 1 && motor_id <= 11) {
                    sendPing(motor_id);
                    // Wait for response
                    unsigned long start = millis();
                    while (millis() - start < PING_TIMEOUT) {
                        processPongResponse();
                        delay(1);
                    }
                }
            }
        } else {
            runPingTest();
        }
        return;
    }
    
    // Normal command parsing
    uint8_t target_motor;
    bool broadcast = false;
    
    if (strcmp(token, "ALL") == 0) {
        broadcast = true;
    } else if (strcmp(token, "ME") == 0) {
        target_motor = MY_MOTOR_ID;
    }
      else if(strcmp(token, "CAL") == 0){
        Serial.println("-> CAL command received. Disabling motor and deleting calibration LUT...");
        
        // 1. Disable the motor to be safe
        motor.disable();
        
        // 2. Initialize the SPIFFS filesystem
        if (!SPIFFS.begin(true)) {
            Serial.println("-> Error: Failed to mount SPIFFS. Cannot delete file.");
        } else {
            // 3. Delete the specific calibration file
            if (SPIFFS.exists("/calibration.bin")) {
                if (SPIFFS.remove("/calibration.bin")) {
                    Serial.println("-> File '/calibration.bin' deleted successfully.");
                } else {
                    Serial.println("-> Error: Failed to delete '/calibration.bin'.");
                }
            } else {
                Serial.println("-> Info: File '/calibration.bin' not found. Nothing to delete.");
            }
        }
        
        Serial.println("Restarting board in 2 seconds...");
        delay(2000); // Short delay to ensure the message is sent
        
        // 3. Restart the ESP32
        ESP.restart();
        return; // Stop further processing of the command
    }
    else {
        target_motor = atoi(token);
        if (target_motor > 11) {
            Serial.println("Invalid motor ID");
            return;
        }
    }
    
    // Second token is command
    token = strtok(NULL, " ");
    if (!token) return;
    type = token[0];
    
    // Parse remaining values
    char* tokens[3];
    int tokenCount = 0;
    while ((token = strtok(NULL, " ")) != NULL && tokenCount < 3) {
        tokens[tokenCount++] = token;
    }
    
    // Process command for local motor if it's for us
    if (!broadcast && target_motor == MY_MOTOR_ID) {
        processLocalCommand(type, tokens, tokenCount);
        return;
    }
    
    // Otherwise, send via CAN
    switch(type) {
        case 'S': {  // Speed/Reel mode
            if (tokenCount >= 1) {
                CanPayloadFloat payload = {.value = atof(tokens[0])};
                if (broadcast) {
                    broadcastCommand(CMD_SET_VELOCITY, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_SET_VELOCITY, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        case 'A': {  // Always reel
            if (tokenCount >= 1) {
                CanPayloadFloat payload = {.value = atof(tokens[0])};
                if (broadcast) {
                    broadcastCommand(CMD_ALWAYS_REEL, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_ALWAYS_REEL, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        case 'T': {  // Torque mode
            if (tokenCount >= 1) {
                CanPayloadFloat payload = {.value = atof(tokens[0])};
                if (broadcast) {
                    broadcastCommand(CMD_SET_TORQUE, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_SET_TORQUE, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        case 'B': {  // Vibration mode
            if (tokenCount >= 3) {
                CanPayloadVibration payload;
                payload.current1_scaled = (int16_t)(atof(tokens[0]) * 1000);
                payload.current2_scaled = (int16_t)(atof(tokens[1]) * 1000);
                payload.frequency = atoi(tokens[2]);
                if (broadcast) {
                    broadcastCommand(CMD_SET_VIBRATION, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_SET_VIBRATION, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        case 'O': {  // Off
            if (broadcast) {
                broadcastCommand(CMD_MOTOR_OFF, nullptr, 0);
            } else {
                sendCANCommand(target_motor, CMD_MOTOR_OFF, nullptr, 0);
            }
            break;
        }
        
        case 'V': {  // Voltage limit
            if (tokenCount >= 1) {
                CanPayloadLimits payload = {.value = atof(tokens[0])};
                if (broadcast) {
                    broadcastCommand(CMD_SET_VOLTAGE_LIM, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_SET_VOLTAGE_LIM, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        case 'C': {  // Current limit
            if (tokenCount >= 1) {
                CanPayloadLimits payload = {.value = atof(tokens[0])};
                if (broadcast) {
                    broadcastCommand(CMD_SET_CURRENT_LIM, &payload, sizeof(payload));
                } else {
                    sendCANCommand(target_motor, CMD_SET_CURRENT_LIM, &payload, sizeof(payload));
                }
            }
            break;
        }
        
        default:
            Serial.println("Unknown command");
            break;
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

void setup() {
    Serial.begin(115200);
    delay(5000);
    
    Serial.println("=== CONTROLLER NODE STARTING ===");
    setup_indicator();
    setup_motor();
    setup_i2c();
    setup_can();
    
    motor.disable();
    
    Serial.println("\n=== Controller Ready ===");
    Serial.println("Commands: [MOTOR_ID/ALL/ME] [S/A/T/B/O/V/C] [values...]");
    Serial.println("Examples:");
    Serial.println("  1 S 10        - Left hand up at speed 50");
    Serial.println("  ALL O         - All motors off");
    Serial.println("  ME T 0.1      - Controller torque 0.5");
    Serial.println("  3 B 0.3 -0.3 10 - Right hand up vibrate");
    Serial.println("\nPing Commands:");
    Serial.println("  PING          - Test all motors connectivity");
    Serial.println("  PING 3        - Test specific motor");
    Serial.println("  PING ALL      - Test all motors");
}


unsigned long last_test_send = 0;
uint32_t test_seq = 0;
uint32_t test_received = 0;

void loop() {
    // Process serial commands and forward to CAN
    processSerialCommand();
    
    // Process any incoming CAN messages (pong responses)
    if (!ping_test_active) {
        processPongResponse();
    }
    
    // Core motor contr ol for local motor
    motor.loopFOC();
    
    // Handle special modes for local motor
    if (noslack) {
        handleReeling();
    } else if (vibration) {
        handleVibration();
    }
    
    // Move local motor
    motor.move(target);
    motor.disable();


    // // 1. Broadcast a test message periodically
    // if (millis() - last_test_send > 2000) {
    //     last_test_send = millis();

    //     CanPayloadPing ping_payload;
    //     ping_payload.timestamp = millis();
    //     ping_payload.sequence = test_seq;

    //     twai_message_t tx_message;
    //     tx_message.identifier = create_can_id(MOTOR_BROADCAST, CMD_PING); // Sends to broadcast ID
    //     tx_message.flags = 0;
    //     tx_message.data_length_code = sizeof(ping_payload);
    //     memcpy(tx_message.data, &ping_payload, sizeof(ping_payload));

    //     if (twai_transmit(&tx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    //         Serial.printf("[TX] Broadcast PING sent (seq: %u)\n", test_seq);
    //         test_seq++; 
    //     } else {
    //         Serial.println("[TX] Failed to send broadcast message.");
    //         twai_clear_transmit_queue();
    //         test_seq = 0;
    //     }
    // }

    // // 2. Listen for all incoming messages and print them to Serial
    // twai_message_t rx_message;
    // if (twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK) { // Non-blocking check
    //     uint8_t source_id;
    //     CAN_COMMAND cmd;
    //     parse_can_id(rx_message.identifier, source_id, cmd);

    //     Serial.println("-------------------------");
    //     Serial.printf("[RX] Message Received!\n");
    //     Serial.printf("  > Raw ID: 0x%lX\n", rx_message.identifier);
    //     Serial.printf("  > Parsed Source ID: %u, Command: %d\n", source_id, cmd);
    //     Serial.printf("  > Data Length: %d bytes\n", rx_message.data_length_code);
    //     Serial.print("  > Data Hex: ");
    //     for (int i = 0; i < rx_message.data_length_code; i++) {
    //         Serial.printf("0x%02X ", rx_message.data[i]);
    //     }
    //     Serial.println("\n-------------------------");
    // }
}