#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

// Version 4 wire format. Flash ALL eight robots with matching files.
namespace Swarm {
static constexpr uint32_t MAGIC = 0x53324D34u; // S2M4
static constexpr uint8_t VERSION = 4;
static constexpr uint8_t GROUP_ONE = 1;
static constexpr uint8_t GROUP_TWO = 2;
static constexpr uint8_t SLAVES_PER_GROUP = 3;

enum Role : uint8_t { ROOT = 1, SECONDARY = 2, SLAVE = 3 };
enum Type : uint8_t { HELLO = 1, GROUP_COMMAND = 2, WHEEL_COMMAND = 3, STATUS = 4, RELAY_STATUS = 5 };
enum Pattern : uint8_t { FOLLOW = 0, MIRROR = 1, SPIN = 2, WAVE = 3 };
enum MotionMode : uint8_t { STOP = 0, MANUAL = 1, DRIVE_MM = 2, TURN_DEG = 3 };
enum MotionState : uint8_t { IDLE = 0, RUNNING = 1, DONE = 2, ERROR = 3 };
enum MotionFault : uint8_t { NO_FAULT = 0, ENCODER_STALL = 1, MOTION_TIMEOUT = 2, LINK_TIMEOUT = 3, ENCODER_REQUIRED = 4 };
static constexpr uint8_t ONLINE_FLAG = 1;
static constexpr uint8_t REAR_ENCODER_FLAG = 2; // capability bit on telemetry

// Fixed-size, ESP-NOW v1 compatible packet. No pointers or platform-dependent floats.
// For commands: originBoot and goalId identify an autonomous mission; repeated
// transmissions of the SAME mission MUST NOT restart its local encoder targets.
// For telemetry: pulses, measured rpm, odomMm, progress, state and fault are measured locally.
struct __attribute__((packed)) Packet {
  uint32_t magic, boot, sequence, ack, frame, uptime;
  uint16_t ageMs;
  uint8_t version, type, role, group, id, target, pattern, flags;
  int8_t throttle, turn, left, right;
  uint32_t originBoot, goalId;
  int32_t distanceMm;
  int16_t angleDeg;
  uint16_t speedRpm;
  uint32_t pulses[4];          // FL/FR always 0; RL/RR measured on encoder variant
  int16_t rpmX10[4];          // FL/FR always 0; RL/RR measured on encoder variant
  int32_t odomMm;             // estimated mean signed wheel travel, millimetres
  uint8_t mode, state, progress, fault;
};
static_assert(sizeof(Packet) < 130, "Unexpected ESP-NOW packet padding");
static_assert(sizeof(Packet) <= 250, "ESP-NOW v1 payload size exceeded");

struct Differential { int8_t left, right; };
inline int limit(int v) { return v < -100 ? -100 : (v > 100 ? 100 : v); }
inline bool valid(const Packet &p) {
  return p.magic == MAGIC && p.version == VERSION &&
    p.type >= HELLO && p.type <= RELAY_STATUS &&
    p.role >= ROOT && p.role <= SLAVE &&
    p.group >= GROUP_ONE && p.group <= GROUP_TWO &&
    p.id <= SLAVES_PER_GROUP && p.target <= GROUP_TWO &&
    p.pattern <= WAVE && p.mode <= TURN_DEG && p.state <= ERROR &&
    p.fault <= ENCODER_REQUIRED && p.progress <= 100 &&
    p.speedRpm <= 200 && p.distanceMm >= -10000 && p.distanceMm <= 10000 &&
    p.angleDeg >= -720 && p.angleDeg <= 720 &&
    p.throttle >= -100 && p.throttle <= 100 &&
    p.turn >= -100 && p.turn <= 100 &&
    p.left >= -100 && p.left <= 100 && p.right >= -100 && p.right <= 100;
}
inline bool newSequence(uint32_t incoming, uint32_t previous) {
  return static_cast<int32_t>(incoming - previous) > 0;
}
inline bool acceptsCommand(uint32_t incomingBoot, uint32_t incomingSeq,
                           uint32_t previousBoot, uint32_t previousSeq) {
  return incomingBoot != 0 && incomingSeq != 0 &&
    (incomingBoot != previousBoot || newSequence(incomingSeq, previousSeq));
}
inline Differential motorsFor(uint8_t pattern, int throttle, int turn, uint8_t localId, uint32_t frame) {
  throttle = limit(throttle); turn = limit(turn);
  if (pattern == MIRROR && (localId & 1u)) turn = -turn;
  if (pattern == SPIN) {
    int s = (throttle != 0) ? throttle : turn;
    if (localId & 1u) s = -s;
    return {static_cast<int8_t>(s), static_cast<int8_t>(-s)};
  }
  if (pattern == WAVE && (frame % (SLAVES_PER_GROUP + 1u)) != localId) return {0, 0};
  return {static_cast<int8_t>(limit(throttle + turn)), static_cast<int8_t>(limit(throttle - turn))};
}
inline int32_t ticksForMm(float distanceMm, float wheelDiameterMm, float pulsesPerWheelRev) {
  const float mmPerWheelRev = 3.14159265358979323846f * wheelDiameterMm;
  return (int32_t)lroundf((distanceMm / mmPerWheelRev) * pulsesPerWheelRev);
}
// Positive TURN_DEG is clockwise viewed from above: left forwards, right backwards.
inline int32_t targetTicks(uint8_t mode, int32_t distanceMm, int16_t angleDeg,
                           uint8_t wheelIndex, float diameterMm, float pulsesPerRev, float trackMm) {
  if (mode == DRIVE_MM) return ticksForMm((float)distanceMm, diameterMm, pulsesPerRev);
  if (mode == TURN_DEG) {
    float wheelMm = (float)angleDeg * 3.14159265358979323846f * trackMm / 360.0f;
    return ticksForMm((wheelIndex == 0 || wheelIndex == 2) ? wheelMm : -wheelMm,
                      diameterMm, pulsesPerRev);
  }
  return 0;
}
} // namespace Swarm
