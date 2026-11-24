#pragma once
#include "SwarmProtocol.h"

// ============================================================
// CHANGE ONLY THESE LINES WHEN UPLOADING TO EACH OF THE 8 CARS
// ROOT/SECONDARY/SLAVE are Swarm:: values.
// M1 = ROOT, group 1, ID 0
// M2 = SECONDARY, group 2, ID 0
// S1-S3 = SLAVE, group 1, IDs 1,2,3
// S4-S6 = SLAVE, group 2, IDs 1,2,3
// ============================================================
#ifndef NODE_ROLE
#define NODE_ROLE Swarm::ROOT
#endif
#ifndef NODE_GROUP
#define NODE_GROUP 1
#endif
#ifndef NODE_ID
#define NODE_ID 0
#endif

// Wi-Fi network is hosted ONLY by Master 1 (ROOT).
#define PHONE_AP_NAME "SwarmLab-Control"
#define PHONE_AP_PASSWORD "CHANGE-ME-1234" // minimum 8 characters; CHANGE this

static constexpr uint8_t ESPNOW_CHANNEL = 6;    // Same for all eight ESP32s
static constexpr uint16_t PWM_HZ = 2000;
static constexpr uint8_t PWM_RESOLUTION = 8;

// CLASSIC ESP32 DevKit GPIOs; not a pin map for ESP32-C3/S3/etc.
// One motor per L298N channel; TWO L298N modules per 4-wheel car.
// L298N A = front-left + front-right; L298N B = rear-left + rear-right.
struct MotorPins { uint8_t en, in1, in2; bool invert; };
static constexpr MotorPins FL_PINS = {25, 26, 27, false};
static constexpr MotorPins FR_PINS = {33, 32, 23, false};
static constexpr MotorPins RL_PINS = {18, 19, 13, false};
static constexpr MotorPins RR_PINS = {17, 16, 14, false};

// Keep modest speeds until motor rotation and current draw are verified.
static constexpr uint32_t PHONE_DEADMAN_MS = 550;
static constexpr uint32_t MASTER_LINK_DEADMAN_MS = 550;
static constexpr uint32_t SLAVE_LINK_DEADMAN_MS = 550;
static constexpr uint32_t PEER_ONLINE_MS = 1300;
static constexpr uint32_t PEER_REPLACE_MS = 5000;
static constexpr uint32_t MOTOR_UPDATE_MS = 110;
static constexpr uint32_t TELEMETRY_MS = 280;
static constexpr uint32_t DISCOVERY_MS = 850;
static constexpr uint32_t TX_GAP_MS = 7;

// ================= TWO-BUILD CONFIGURATION =================
// This value is set for each complete sketch folder. DO NOT change it to
// mix code generations. All eight ESP32s MUST use this V4 wire protocol.
#ifndef SWARM_REAR_ENCODERS
#define SWARM_REAR_ENCODERS 1
#endif
// Only rear-left and rear-right wheel encoder A signals are sampled.
// GPIO34/35 are input-only and have NO internal pull-ups. For open-collector
// outputs, 4.7k-10k pull-ups to 3.3 V may be required. NEVER feed 5 V to ESP32.
static constexpr uint8_t REAR_ENCODER_PINS[2] = {34, 35}; // RL, RR
// CALIBRATION PLACEHOLDERS, replace on EACH encoder-equipped robot:
static constexpr float ENCODER_TICKS_PER_WHEEL_REV = 330.0f;
static constexpr float WHEEL_DIAMETER_MM = 65.0f;
static constexpr float EFFECTIVE_TRACK_WIDTH_MM = 165.0f;
static constexpr float MAX_WHEEL_RPM = 85.0f;
static constexpr float MIN_GOAL_RPM = 16.0f;
static constexpr float PID_KP = 1.2f;
static constexpr float PID_KI = 0.60f;
static constexpr float PWM_STATIC = 76.0f;
static constexpr float PWM_PER_RPM = 1.15f;
static constexpr uint32_t CONTROL_INTERVAL_MS = 60;
static constexpr uint32_t ENCODER_STALL_MS = 1900;
static constexpr uint32_t MAX_GOAL_RUNTIME_MS = 120000;
static constexpr uint32_t PHONE_MISSION_HEARTBEAT_MS = 2300;
static constexpr uint32_t GOAL_GROUP_GRACE_MS = 2300;
static constexpr int32_t GOAL_TOLERANCE_TICKS = 3;
// Non-encoder vehicles use open-loop PWM. Not a speed/position guarantee.
static constexpr int OPEN_LOOP_PWM_MIN = 65;
static constexpr int OPEN_LOOP_PWM_MAX = 210;
