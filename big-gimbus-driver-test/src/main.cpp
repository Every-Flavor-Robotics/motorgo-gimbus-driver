#include <Arduino.h>
#include <FastLED.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <drivers/drv8316/drv8316.h>
#include <SPIFFS.h>
#include "define_pins.h"

// ============================================================
//  Hardware constants
// ============================================================
#define SPI_CLK   21
#define SPI_CIPO  48
#define SPI_COPI  47
#define DRV_CS    14

// Motor parameters
#define POLE_PAIRS  11
#define MOTOR_KV    24
#define MOTOR_R     6.9f

// Limits
#define VOLTAGE_POWER_SUPPLY  12.0f
#define VOLTAGE_LIMIT         4.0f
#define CURRENT_LIMIT         1.0f
#define VELOCITY_LIMIT        100.0f   // rad/s
#define MAX_TORQUE_VOLTAGE    12.0f
#define MAX_CURRENT           1.2f

// Velocity mode limits
#define SPEED_VOLTAGE_LIMIT   2.0f
#define SPEED_CURRENT_LIMIT   0.2f

// Heartbeat / telemetry timing
#define HEARTBEAT_TIMEOUT_MS  1000
#define TELEM_INTERVAL_MS     100    // 10 Hz

// Serial command buffer
#define CMD_BUF_LEN  64

// ============================================================
//  Motor objects
// ============================================================
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI sensor(enc_cs);
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

BLDCMotor motor = BLDCMotor(POLE_PAIRS, MOTOR_R, MOTOR_KV);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(UH, UL, VH, VL, WH, WL, DRV_CS, false, NOT_SET, NOT_SET);

// ============================================================
//  LED
// ============================================================
CRGB ind[1];

// ============================================================
//  State
// ============================================================
enum MotorMode { MODE_DISABLED, MODE_VELOCITY, MODE_POSITION, MODE_TORQUE, MODE_CALIBRATING, MODE_ERROR };
MotorMode currentMode = MODE_DISABLED;
float target = 0.0f;

unsigned long lastCommandTime = 0;
unsigned long lastTelemTime   = 0;

char cmdBuf[CMD_BUF_LEN];

// ============================================================
//  LED helpers
// ============================================================
void updateLED() {
    switch (currentMode) {
        case MODE_DISABLED:  ind[0] = CRGB(0, 255, 255);   break;  // Cyan
        case MODE_VELOCITY:  ind[0] = CRGB(0, 255, 0);     break;  // Green
        case MODE_POSITION:  ind[0] = CRGB(255, 255, 0);   break;  // Yellow
        case MODE_TORQUE:    ind[0] = CRGB(255, 0, 255);   break;  // Magenta
        case MODE_CALIBRATING: ind[0] = CRGB(255, 165, 0);  break;  // Orange
        case MODE_ERROR:     ind[0] = CRGB(255, 0, 0);     break;  // Red
    }
    FastLED.show();
}

// ============================================================
//  Telemetry
// ============================================================
void sendTelemetry() {
    if (millis() - lastTelemTime < TELEM_INTERVAL_MS) return;
    lastTelemTime = millis();

    char modeChar;
    switch (currentMode) {
        case MODE_VELOCITY: modeChar = 'V'; break;
        case MODE_POSITION: modeChar = 'P'; break;
        case MODE_TORQUE:   modeChar = 'T'; break;
        case MODE_CALIBRATING: modeChar = 'C'; break;
        case MODE_ERROR:    modeChar = 'E'; break;
        default:            modeChar = 'D'; break;
    }

    Serial.printf("TELEM:%c:%.4f:%.4f:%.4f:%lu\n",
        modeChar,
        target,
        motor.shaftAngle(),
        motor.shaftVelocity(),
        millis()
    );
}

// ============================================================
//  Heartbeat watchdog
// ============================================================
void checkHeartbeat() {
    // Do not fire during calibration, idle, or error — motor is either off or intentionally busy
    if (currentMode == MODE_DISABLED || currentMode == MODE_CALIBRATING || currentMode == MODE_ERROR) return;
    if (millis() - lastCommandTime > HEARTBEAT_TIMEOUT_MS) {
        target = 0.0f;
        motor.disable();
        currentMode = MODE_DISABLED;
        updateLED();
        Serial.println("DBG:Heartbeat timeout - motor disabled");
    }
}

// ============================================================
//  Forced recalibration
//  Deletes the saved SPIFFS file, runs a fresh calibration,
//  saves the result, and re-inits FOC. The motor must be
//  free to rotate during this call (~10 s). Heartbeat is
//  suppressed for the duration via MODE_CALIBRATING.
// ============================================================
void runCalibration() {
    Serial.println("DBG:Recalibration requested - disabling motor");

    target = 0.0f;
    motor.disable();
    currentMode = MODE_CALIBRATING;
    updateLED();

    // Flush any pending telemetry so the Jetson sees the C state immediately
    lastTelemTime = 0;
    sendTelemetry();

    // Delete existing calibration file
    if (SPIFFS.exists("/calibration.bin")) {
        SPIFFS.remove("/calibration.bin");
        Serial.println("DBG:Old calibration deleted");
    }

    // Re-link raw sensor for calibration pass
    motor.linkSensor(&sensor);
    motor.initFOC();

    Serial.println("DBG:Calibrating - motor will rotate slowly...");
    sensor_calibrated.voltage_calibration = 4;
    sensor_calibrated.calibrate(motor, 10);
    sensor_calibrated.saveCalibration(motor);
    Serial.println("DBG:Calibration complete and saved");

    // Switch back to calibrated sensor
    motor.linkSensor(&sensor_calibrated);
    motor.initFOC();

    motor.disable();
    currentMode = MODE_DISABLED;
    updateLED();

    // Reset heartbeat timer so the watchdog doesn't fire immediately
    lastCommandTime = millis();

    Serial.println("DBG:Ready after recalibration");
}

// ============================================================
//  Serial command parser
//  Protocol: CMD:<type>:<value>\n
//    CMD:V:<float>  - velocity mode (rad/s)
//    CMD:P:<float>  - position mode (rad)
//    CMD:T:<float>  - torque mode (voltage)
//    CMD:O          - disable motor
//    CMD:R          - force sensor recalibration
// ============================================================
void processSerialCommand() {
    if (!Serial.available()) return;

    int len = Serial.readBytesUntil('\n', cmdBuf, CMD_BUF_LEN - 1);
    cmdBuf[len] = '\0';

    // Must start with "CMD:"
    if (strncmp(cmdBuf, "CMD:", 4) != 0) return;

    char type = cmdBuf[4];
    float value = 0.0f;

    // Parse value after second colon if present
    if (len > 6 && cmdBuf[5] == ':') {
        value = atof(&cmdBuf[6]);
    }

    switch (type) {
        case 'V': {
            // Clamp to velocity limit
            value = constrain(value, -VELOCITY_LIMIT, VELOCITY_LIMIT);
            target = value;
            motor.controller = MotionControlType::velocity;
            motor.voltage_limit = SPEED_VOLTAGE_LIMIT;
            motor.current_limit = SPEED_CURRENT_LIMIT;
            if (currentMode != MODE_VELOCITY) {
                // Zero target before mode switch to avoid jerk
                motor.move(0.0f);
            }
            motor.enable();
            currentMode = MODE_VELOCITY;
            updateLED();
            lastCommandTime = millis();
            break;
        }

        case 'P': {
            target = value;  // rad — no hard clamp, mechanical limits are application-specific
            motor.controller = MotionControlType::angle;
            motor.voltage_limit = VOLTAGE_LIMIT;
            motor.current_limit = CURRENT_LIMIT;
            if (currentMode != MODE_POSITION) {
                motor.move(motor.shaftAngle());  // hold current position before switching
            }
            motor.enable();
            currentMode = MODE_POSITION;
            updateLED();
            lastCommandTime = millis();
            break;
        }

        case 'T': {
            // Clamp to max torque voltage
            value = constrain(value, -MAX_CURRENT, MAX_CURRENT);
            target = value;
            motor.controller = MotionControlType::torque;
            motor.voltage_limit = MAX_TORQUE_VOLTAGE;
            motor.current_limit = MAX_CURRENT;
            if (currentMode != MODE_TORQUE) {
                motor.move(0.0f);
            }
            motor.enable();
            currentMode = MODE_TORQUE;
            updateLED();
            lastCommandTime = millis();
            break;
        }

        case 'O': {
            target = 0.0f;
            motor.disable();
            currentMode = MODE_DISABLED;
            updateLED();
            break;
        }

        case 'R': {
            runCalibration();
            break;
        }

        default:
            Serial.printf("DBG:Unknown command type '%c'\n", type);
            break;
    }
}

// ============================================================
//  Setup helpers
// ============================================================
void setup_indicator() {
    pinMode(IND_LIGHT, OUTPUT);
    FastLED.addLeds<SK6812, IND_LIGHT, GRB>(ind, 1);
    ind[0] = CRGB(0, 0, 255);  // Blue during boot
    FastLED.show();
}

bool setup_motor() {
    // SPI buses
    hspi.begin(enc_scl, enc_sda, enc_cs);
    SPI.begin(SPI_CLK, SPI_CIPO, SPI_COPI, DRV_CS);
    delay(300);

    // Encoder
    sensor.init(&hspi);
    motor.linkSensor(&sensor);

    // Driver
    driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver.pwm_frequency = 20000;

    driver.init(&SPI);
    if (!driver.initialized) {
        Serial.println("DBG:Driver init failed - check SPI wiring and DRV_CS pin");
        currentMode = MODE_ERROR;
        updateLED();
        return false;
    }

    driver.setCurrentSenseGain(DRV8316_CSAGain::Gain_0V15);
    driver.setSlew(DRV8316_Slew::Slew_200Vus);
    driver.setPWMMode(DRV8316_PWMMode::PWM6_Mode);
    driver.setPWM100Frequency(DRV8316_PWM100DUTY::FREQ_20KHz);

    motor.linkDriver(&driver);

    // FOC configuration
    motor.voltage_sensor_align  = 4;
    motor.foc_modulation        = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller     = TorqueControlType::voltage;
    motor.controller            = MotionControlType::torque;

    // Limits
    motor.voltage_limit  = VOLTAGE_LIMIT;
    motor.current_limit  = CURRENT_LIMIT;
    motor.velocity_limit = VELOCITY_LIMIT;

    // Velocity PID
    motor.PID_velocity.P           = 0.75f;
    motor.PID_velocity.I           = 0.075f;
    motor.PID_velocity.D           = 0.001f;
    motor.PID_velocity.output_ramp = 1000.0f;
    motor.LPF_velocity.Tf          = 0.05f;

    // Angle PID
    motor.P_angle.P = 20.0f;
    motor.P_angle.I = 0.0f;
    motor.P_angle.D = 0.0f;

    // Current D PID
    motor.PID_current_d.P           = 0.25f;
    motor.PID_current_d.I           = 0.0f;
    motor.PID_current_d.D           = 0.0f;
    motor.PID_current_d.output_ramp = 0.0f;
    motor.PID_current_d.limit       = 12.0f;
    motor.LPF_current_d.Tf          = 0.01f;

    // Current Q PID
    motor.PID_current_q.P           = 1.0f;
    motor.PID_current_q.I           = 0.0f;
    motor.PID_current_q.D           = 0.0f;
    motor.PID_current_q.output_ramp = 100.0f;
    motor.PID_current_q.limit       = 12.0f;
    motor.LPF_current_q.Tf          = 0.01f;

    Serial.println("DBG:Initializing motor...");
    delay(2000);
    motor.init();
    motor.initFOC();

    // Calibrated sensor — load from SPIFFS or run calibration
    Serial.println("DBG:Loading sensor calibration...");
    sensor_calibrated.voltage_calibration = 4;

    if (!SPIFFS.begin(true)) {
        Serial.println("DBG:SPIFFS mount failed - calibration unavailable");
        currentMode = MODE_ERROR;
        updateLED();
        return false;
    }

    if (!sensor_calibrated.loadCalibration(motor)) {
        Serial.println("DBG:No calibration found - calibrating now (motor will rotate)...");
        sensor_calibrated.calibrate(motor, 10);
        sensor_calibrated.saveCalibration(motor);
        Serial.println("DBG:Calibration saved");
    } else {
        Serial.println("DBG:Calibration loaded from SPIFFS");
    }

    // Switch to calibrated sensor and re-init FOC
    motor.linkSensor(&sensor_calibrated);
    motor.initFOC();

    Serial.println("DBG:Motor ready");
    return true;
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(5000);

    Serial.println("DBG:=== Gimbus Motor Driver Starting ===");

    setup_indicator();

    if (!setup_motor()) {
        // Error state — LED already set red in setup_motor()
        Serial.println("TELEM:E:0.0000:0.0000:0.0000:0");
        // Halt — require power cycle to retry
        while (true) {
            ind[0] = (millis() / 250) % 2 ? CRGB(255, 0, 0) : CRGB(0, 0, 0);
            FastLED.show();
            delay(10);
        }
    }

    motor.disable();
    currentMode = MODE_DISABLED;
    updateLED();

    Serial.println("DBG:=== Ready ===");
    Serial.println("DBG:Commands: CMD:V:<rad/s>  CMD:P:<rad>  CMD:T:<voltage>  CMD:O  CMD:R");
}

void loop() {
    processSerialCommand();
    checkHeartbeat();

    motor.loopFOC();
    motor.move(target);

    sendTelemetry();
}
