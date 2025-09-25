#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <Arduino.h>

// Motor ID Mapping - Assign meaningful names to motor IDs
// Controller is ID 0, follower motors are 1-11
#define MOTOR_CONTROLLER      0
#define MOTOR_LEFT_HAND_UP    1
#define MOTOR_LEFT_HAND_DOWN  2
#define MOTOR_RIGHT_HAND_UP   3
#define MOTOR_RIGHT_HAND_DOWN 4
#define MOTOR_LEFT_FOOT_UP    5
#define MOTOR_RIGHT_FOOT_UP   6
#define MOTOR_BACK_LEFT       7
#define MOTOR_BACK_RIGHT      8
#define MOTOR_HEAD_LEFT       9
#define MOTOR_HEAD_RIGHT      10
#define MOTOR_CHEST_DOWN      11
#define MOTOR_EXTRA           12  // Reserved for future use

// Broadcast ID for sending commands to all motors
#define MOTOR_BROADCAST       0x7F

// Define the CAN command types (using the lower 4 bits of the ID)
enum CAN_COMMAND : uint8_t {
    CMD_SET_TORQUE      = 0x01,
    CMD_SET_VELOCITY    = 0x02,
    CMD_SET_VIBRATION   = 0x03,
    CMD_MOTOR_OFF       = 0x04,
    CMD_ALWAYS_REEL     = 0x05,
    CMD_SET_VOLTAGE_LIM = 0x06,
    CMD_SET_CURRENT_LIM = 0x07,
    CMD_EMERGENCY_STOP  = 0x08,  // Added for safety
    CMD_STATUS_REQUEST  = 0x09,  // Request motor status
    CMD_CALIBRATE       = 0x0A,  // Trigger calibration
    CMD_PING            = 0x0B,  // Send PING 
    CMD_PONG            = 0x0C   // Reply to PING with PONG
};

// Helper function to create the CAN Arbitration ID
static inline uint32_t create_can_id(uint8_t motor_id, CAN_COMMAND cmd) {
    return (motor_id << 4) | cmd;
}

// Helper function to parse the CAN Arbitration ID
static inline void parse_can_id(uint32_t can_id, uint8_t &motor_id, CAN_COMMAND &cmd) {
    motor_id = (can_id >> 4) & 0x7F; // 7 bits for motor ID
    cmd = (CAN_COMMAND)(can_id & 0x0F); // 4 bits for command
}

// === Payload Structs for Data Packing ===

// For single float commands like Torque and Velocity
struct CanPayloadFloat {
    float value;
} __attribute__((packed));

// For the Vibration command
struct CanPayloadVibration {
    int16_t current1_scaled; // Scaled by 1000 to fit in int16
    int16_t current2_scaled; // Scaled by 1000 to fit in int16
    uint16_t frequency;      // Frequency in Hz
} __attribute__((packed));

// For limit commands
struct CanPayloadLimits {
    float value;
} __attribute__((packed));

// For status reporting (future use)
struct CanPayloadStatus {
    int16_t velocity_scaled;  // Scaled by 100
    int16_t current_scaled;   // Scaled by 1000
    uint16_t position;        // Encoder position
    uint8_t status_flags;     // Bit flags for various states
} __attribute__((packed));

// For ping/pong commands
struct CanPayloadPing {
    uint32_t timestamp;       // Timestamp for latency measurement
    uint8_t sequence;         // Sequence number for tracking
} __attribute__((packed));

struct CanPayloadPong {
    uint32_t timestamp;       // Echo back the original timestamp
    uint8_t sequence;         // Echo back the sequence number
    uint8_t motor_status;     // 0 = OK, 1 = Error, 2 = Warning
    uint16_t fw_version;      // Firmware version (optional)
} __attribute__((packed));

// Helper function to get motor name string
static inline const char* getMotorName(uint8_t motor_id) {
    switch(motor_id) {
        case MOTOR_CONTROLLER:      return "Controller";
        case MOTOR_LEFT_HAND_UP:    return "Left Hand Up";
        case MOTOR_LEFT_HAND_DOWN:  return "Left Hand Down";
        case MOTOR_RIGHT_HAND_UP:   return "Right Hand Up";
        case MOTOR_RIGHT_HAND_DOWN: return "Right Hand Down";
        case MOTOR_LEFT_FOOT_UP:    return "Left Foot Up";
        case MOTOR_RIGHT_FOOT_UP:   return "Right Foot Up";
        case MOTOR_BACK_LEFT:       return "Back Left";
        case MOTOR_BACK_RIGHT:      return "Back Right";
        case MOTOR_HEAD_LEFT:       return "Head Left";
        case MOTOR_HEAD_RIGHT:      return "Head Right";
        case MOTOR_CHEST_DOWN:      return "Chest Down";
        case MOTOR_EXTRA:           return "Extra";
        case MOTOR_BROADCAST:       return "Broadcast";
        default:                    return "Unknown";
    }
}

// Helper macros for motor groups (useful for coordinated movements)
#define IS_HAND_MOTOR(id) ((id) >= MOTOR_LEFT_HAND_UP && (id) <= MOTOR_RIGHT_HAND_DOWN)
#define IS_FOOT_MOTOR(id) ((id) == MOTOR_LEFT_FOOT_UP || (id) == MOTOR_RIGHT_FOOT_UP)
#define IS_BACK_MOTOR(id) ((id) == MOTOR_BACK_LEFT || (id) == MOTOR_BACK_RIGHT)
#define IS_HEAD_MOTOR(id) ((id) == MOTOR_HEAD_LEFT || (id) == MOTOR_HEAD_RIGHT)

#endif // CAN_PROTOCOL_H