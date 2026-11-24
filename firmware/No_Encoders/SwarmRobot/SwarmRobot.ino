/*
 * DUAL-MASTER ESP32 SWARM / 2 masters + 6 slaves / 4 motors per vehicle.
 * Version 4, Arduino-ESP32 3.x on a classic ESP32 DevKit.
 *
 * Phone -> ROOT Master M1 (HTTP over M1 SoftAP)
 * M1 -> 3 group-1 slaves (ESP-NOW unicast commands)
 * M1 <-> M2 (two-way ESP-NOW unicast, command + telemetry)
 * M2 -> 3 group-2 slaves (ESP-NOW unicast commands)
 * Every slave -> its own master (ESP-NOW status + applied-command ack)
 * M2 forwards its slaves' statuses to M1 for the phone dashboard.
 *
 * Two compatible builds: open-loop without encoders OR rear-left/rear-right
 * single-channel encoders, rear-wheel PI speed loops and relative goals.
 * Front wheels follow the corresponding rear-side PWM. No global positioning.
 *
 * Test with all wheels OFF THE GROUND and with hardware motor-power cutoffs.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "RobotConfig.h"

using namespace Swarm;
static_assert(SWARM_REAR_ENCODERS == 0 || SWARM_REAR_ENCODERS == 1, "Invalid variant");
static_assert(ENCODER_TICKS_PER_WHEEL_REV > 0 && WHEEL_DIAMETER_MM > 0 &&
              EFFECTIVE_TRACK_WIDTH_MM > 0, "Invalid encoder calibration");
static_assert(NODE_ROLE == ROOT || NODE_ROLE == SECONDARY || NODE_ROLE == SLAVE,
              "Invalid NODE_ROLE");
static_assert((NODE_ROLE == ROOT && NODE_GROUP == 1 && NODE_ID == 0) ||
              (NODE_ROLE == SECONDARY && NODE_GROUP == 2 && NODE_ID == 0) ||
              (NODE_ROLE == SLAVE && (NODE_GROUP == 1 || NODE_GROUP == 2) &&
               NODE_ID >= 1 && NODE_ID <= 3),
              "Invalid group/id for this role");

static const uint8_t BROADCAST[6] = {255,255,255,255,255,255};
static constexpr size_t TX_QUEUE_SIZE = 48;
static constexpr size_t RX_QUEUE_SIZE = 32;

struct Motor {
  MotorPins pins;
  int8_t lastDirection;
  int16_t signedPwm;        // logical PWM command, not encoder feedback
};
Motor motors[4] = {
  {FL_PINS, 0, 0}, {FR_PINS, 0, 0}, {RL_PINS, 0, 0}, {RR_PINS, 0, 0}
};
int8_t appliedLeft = 0, appliedRight = 0;

// Single-channel encoder: counts increase on rising edges. Direction is INFERRED
// from the preceding motor drive command; it is not directly sensed.
volatile uint32_t encoderPulses[4] = {0,0,0,0}; // only indices 2,3 are physical
uint32_t encoderSample[4] = {0,0,0,0};
uint32_t encoderLastRise[4] = {0,0,0,0};
float actualRpm[4] = {0,0,0,0};
float pidIntegral[4] = {0,0,0,0};
int32_t wheelOdomTicks[4] = {0,0,0,0};
uint32_t lastControlAt = 0;

struct LocalMotion {
  uint8_t mode = STOP, state = IDLE, fault = NO_FAULT, progress = 0;
  int8_t manualLeft = 0, manualRight = 0;
  uint32_t originBoot = 0, goalId = 0, startedAt = 0;
  uint16_t speedRpm = 35;
  int32_t startTicks[4] = {0,0,0,0};
  int32_t targetDelta[4] = {0,0,0,0};
};
LocalMotion motion;

struct MotorTelemetry {
  bool rearEncoders = false;
  uint32_t pulses[4] = {0,0,0,0};
  int16_t rpmX10[4] = {0,0,0,0};
  int32_t odomMm = 0;
  uint32_t goalId = 0;
  uint8_t mode = STOP, state = IDLE, progress = 0, fault = NO_FAULT;
};
struct PeerState {
  bool bound = false;
  uint8_t mac[6] = {};
  uint32_t lastHelloAt = 0, lastStatusAt = 0;
  uint32_t lastCmdSentSeq = 0, ack = 0;
  int8_t left = 0, right = 0;
  MotorTelemetry telemetry;
};
PeerState ownSlaves[SLAVES_PER_GROUP + 1];
PeerState otherMaster;
PeerState ownerMaster;
struct RemoteSlave {
  bool received = false, reportedOnline = false;
  uint32_t lastRelayAt = 0;
  uint16_t ageAtRelay = UINT16_MAX;
  uint32_t sent = 0, ack = 0;
  int8_t left = 0, right = 0;
  MotorTelemetry telemetry;
};
RemoteSlave group2Slaves[SLAVES_PER_GROUP + 1];

struct RxEvent { uint8_t mac[6]; Packet data; };
struct TxEvent { uint8_t mac[6]; Packet data; uint32_t queuedAt; };
QueueHandle_t rxQueue = nullptr;
TxEvent txQueue[TX_QUEUE_SIZE] = {};
size_t txHead = 0, txTail = 0, txUsed = 0;
uint32_t lastPhysicalTxAt = 0;
uint32_t txFailures = 0;
uint32_t rxQueueDrops = 0; // callback only increments; informational
uint32_t bootId = 0;
uint32_t nextSeq = 0;
uint32_t lastHelloAt = 0, lastTelemetryAt = 0, lastMotorUpdateAt = 0;
uint32_t lastPhoneStatusAt = 0, rootGoalStartedAt = 0, rootGoalCounter = 0;
bool rootGoalActive = false;
String lastMissionResult = "none";

// ROOT accepts browser drive commands; SECONDARY accepts only ROOT commands.
int8_t desiredThrottle = 0;
int8_t desiredTurn = 0;
uint8_t desiredPattern = FOLLOW;
uint8_t desiredTarget = 0;  // 0 = both, 1 = group 1, 2 = group 2
uint8_t desiredMode = STOP;
uint32_t desiredGoalId = 0, desiredOriginBoot = 0;
int32_t desiredDistanceMm = 0;
int16_t desiredAngleDeg = 0;
uint16_t desiredSpeedRpm = 35;
uint32_t lastPhoneAt = 0;
uint64_t lastBrowserSequence = 0; // Reject delayed HTTP movement after a Stop.
uint32_t lastRootCommandAt = 0;
uint32_t lastAcceptedBoot = 0, lastAcceptedSequence = 0;
uint32_t lastAppliedSequence = 0;  // Secondary: root command ack; Slave: motor command ack
uint32_t rootFrame = 0;

WebServer server(80); // Only started on ROOT.

static bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}
static bool isFresh(uint32_t last, uint32_t maxAge, uint32_t now) {
  return last != 0 && (uint32_t)(now - last) <= maxAge;
}
static const char *roleName(uint8_t role) {
  return role == ROOT ? "M1 root" : role == SECONDARY ? "M2 master" : "slave";
}

#if SWARM_REAR_ENCODERS
static void IRAM_ATTR encoderISR(void *arg) {
  auto *count = static_cast<volatile uint32_t *>(arg);
  ++(*count);
}
#endif
static void setMotor(Motor &m, int16_t signedPwm) {
  signedPwm = constrain(signedPwm, (int16_t)-255, (int16_t)255);
  const int16_t electrical = m.pins.invert ? -signedPwm : signedPwm;
  const int8_t direction = (electrical > 0) - (electrical < 0);
  if (direction != m.lastDirection) {
    ledcWrite(m.pins.en, 0);
    digitalWrite(m.pins.in1, LOW);
    digitalWrite(m.pins.in2, LOW);
    if (direction > 0) digitalWrite(m.pins.in1, HIGH);
    if (direction < 0) digitalWrite(m.pins.in2, HIGH);
    m.lastDirection = direction;
  }
  m.signedPwm = signedPwm;
  ledcWrite(m.pins.en, (uint32_t)abs(electrical));
}
static void stopMotors() {
  for (Motor &m : motors) setMotor(m, 0);
  appliedLeft = appliedRight = 0;
}
static bool initMotors() {
  for (Motor &m : motors) {
    pinMode(m.pins.in1, OUTPUT); pinMode(m.pins.in2, OUTPUT);
    digitalWrite(m.pins.in1, LOW); digitalWrite(m.pins.in2, LOW);
    pinMode(m.pins.en, OUTPUT); digitalWrite(m.pins.en, LOW);
  }
  for (Motor &m : motors) {
    if (!ledcAttach(m.pins.en, PWM_HZ, PWM_RESOLUTION)) return false;
  }
  stopMotors();
  return true;
}
static void initEncoders() {
#if SWARM_REAR_ENCODERS
  for (uint8_t side = 0; side < 2; ++side) {
    const uint8_t i = side + 2; // rear-left, rear-right only
    pinMode(REAR_ENCODER_PINS[side], INPUT); // no internal pull-ups
    attachInterruptArg(REAR_ENCODER_PINS[side], encoderISR,
                       (void *)&encoderPulses[i], RISING);
  }
#endif
  lastControlAt = millis();
}
static void failMotion(uint8_t fault) {
  if (motion.fault == NO_FAULT) motion.fault = fault;
  motion.state = ERROR;
  motion.mode = STOP;
  stopMotors();
}
static void commandLocal(uint8_t mode, Differential manual, uint32_t epoch,
                         uint32_t goalId, int32_t distanceMm, int16_t angleDeg,
                         uint16_t speedRpm, uint32_t now) {
  if (mode == STOP) {
    if (motion.state == RUNNING || motion.state == IDLE) {
      motion.state = motion.fault == NO_FAULT ? IDLE : ERROR;
      motion.mode = STOP;
    }
    stopMotors();
    return;
  }
  if (motion.fault != NO_FAULT) { stopMotors(); return; } // fault latched until reboot
  if (mode == MANUAL) {
    if (motion.mode != MANUAL || motion.state != RUNNING) {
      for (uint8_t i = 0; i < 4; ++i) {
        encoderLastRise[i] = now; // stall timeout begins at actual motion start
        pidIntegral[i] = 0;
      }
    }
    motion.mode = MANUAL;
    motion.manualLeft = manual.left;
    motion.manualRight = manual.right;
    motion.state = (manual.left || manual.right) ? RUNNING : IDLE;
    return;
  }
#if !SWARM_REAR_ENCODERS
  // Never let a car without encoders execute a distance/angle command.
  failMotion(ENCODER_REQUIRED);
  return;
#endif
  // Repeated mission heartbeats must NEVER reset measured start ticks.
  // STOP -> same goal also must not restart (e.g. a delayed command after Stop).
  if (epoch == 0 || goalId == 0 ||
      (motion.originBoot == epoch && motion.goalId == goalId)) return;
  motion.originBoot = epoch;
  motion.goalId = goalId;
  motion.mode = mode;
  motion.state = RUNNING;
  motion.progress = 0;
  motion.startedAt = now;
  motion.speedRpm = speedRpm;
  for (uint8_t i = 0; i < 4; ++i) {
    motion.startTicks[i] = wheelOdomTicks[i];
    motion.targetDelta[i] = targetTicks(mode, distanceMm, angleDeg, i,
                          WHEEL_DIAMETER_MM, ENCODER_TICKS_PER_WHEEL_REV,
                          EFFECTIVE_TRACK_WIDTH_MM);
    encoderLastRise[i] = now;
    pidIntegral[i] = 0;
  }
}
static void updateMotorController(uint32_t now) {
  const uint32_t dtMs = (uint32_t)(now - lastControlAt);
  if (dtMs < CONTROL_INTERVAL_MS) return;
  lastControlAt = now;
  const float dt = dtMs / 1000.0f;
#if SWARM_REAR_ENCODERS
  uint32_t counts[2];
  noInterrupts();
  counts[0] = encoderPulses[2]; counts[1] = encoderPulses[3];
  interrupts();
  for (uint8_t side = 0; side < 2; ++side) {
    const uint8_t idx = side + 2;
    uint32_t edges = counts[side] - encoderSample[idx];
    encoderSample[idx] = counts[side];
    // With a single A channel direction is INFERRED from commanded PWM.
    const int sign = (motors[idx].signedPwm > 0) - (motors[idx].signedPwm < 0);
    if (edges && sign) {
      wheelOdomTicks[idx] += sign * (int32_t)edges;
      encoderLastRise[idx] = now;
    }
    const float measured = (float)edges * 60.0f / (ENCODER_TICKS_PER_WHEEL_REV * dt);
    actualRpm[idx] = .45f * (float)sign * measured + .55f * actualRpm[idx];
  }
#endif
  if (motion.fault != NO_FAULT || motion.state != RUNNING) {
    stopMotors(); return;
  }
  if (motion.mode != MANUAL
#if SWARM_REAR_ENCODERS
      && motion.mode != DRIVE_MM && motion.mode != TURN_DEG
#endif
      ) {
    stopMotors(); return;
  }
#if SWARM_REAR_ENCODERS
  if ((motion.mode == DRIVE_MM || motion.mode == TURN_DEG) &&
      (uint32_t)(now - motion.startedAt) > MAX_GOAL_RUNTIME_MS) {
    failMotion(MOTION_TIMEOUT); return;
  }
#endif
  int completed = 0;
  int32_t progressSum = 0;
  for (uint8_t side = 0; side < 2; ++side) {
    const uint8_t rear = side + 2;
    const uint8_t front = side;
    float targetRpm = 0;
    int demand = side == 0 ? motion.manualLeft : motion.manualRight;
#if SWARM_REAR_ENCODERS
    if (motion.mode == MANUAL) {
      targetRpm = (float)demand * MAX_WHEEL_RPM / 100.0f;
    } else {
      const int32_t total = motion.targetDelta[rear];
      const int32_t travelled = wheelOdomTicks[rear] - motion.startTicks[rear];
      const int32_t remaining = total - travelled;
      progressSum += total == 0 ? 100 : (int32_t)constrain(
          (int)(100.0f * fabsf((float)travelled) / fabsf((float)total)), 0, 100);
      if (total == 0 || abs(remaining) <= GOAL_TOLERANCE_TICKS) {
        ++completed;
      } else {
        float taper = fminf(1.0f, fabsf((float)remaining) /
                           (ENCODER_TICKS_PER_WHEEL_REV * 0.22f));
        float rpm = fmaxf(MIN_GOAL_RPM, motion.speedRpm * taper);
        targetRpm = remaining > 0 ? rpm : -rpm;
      }
    }
    if (fabsf(targetRpm) < 0.5f) {
      setMotor(motors[front], 0); setMotor(motors[rear], 0);
      pidIntegral[rear] = 0;
      continue;
    }
    const float speedError = fabsf(targetRpm) - fabsf(actualRpm[rear]);
    pidIntegral[rear] = constrain(pidIntegral[rear] + speedError * dt, -100.0f, 100.0f);
    float duty = PWM_STATIC + PWM_PER_RPM * fabsf(targetRpm) +
                 PID_KP * speedError + PID_KI * pidIntegral[rear];
    const int pwm = constrain((int)lroundf(duty), 0, 240);
    const int direction = targetRpm > 0 ? 1 : -1;
    if (motors[rear].signedPwm != 0 &&
        (motors[rear].signedPwm > 0) != (targetRpm > 0)) {
      setMotor(motors[front], 0); setMotor(motors[rear], 0);
      pidIntegral[rear] = 0;
      encoderLastRise[rear] = now; // one controller cycle coast before reversal
      continue;
    }
    if (pwm >= 70 && (uint32_t)(now - encoderLastRise[rear]) > ENCODER_STALL_MS) {
      failMotion(ENCODER_STALL); return;
    }
    setMotor(motors[front], (int16_t)(direction * pwm));
    setMotor(motors[rear], (int16_t)(direction * pwm));
#else
    // Without encoders only PWM can be commanded. No measured RPM/odometry.
    if (demand == 0) {
      setMotor(motors[front], 0); setMotor(motors[rear], 0);
    } else {
      const int pwm = OPEN_LOOP_PWM_MIN + abs(demand) *
          (OPEN_LOOP_PWM_MAX - OPEN_LOOP_PWM_MIN) / 100;
      const int signedPwm = (demand > 0 ? 1 : -1) * pwm;
      setMotor(motors[front], signedPwm);
      setMotor(motors[rear], signedPwm);
    }
#endif
  }
#if SWARM_REAR_ENCODERS
  if (motion.mode == DRIVE_MM || motion.mode == TURN_DEG) {
    motion.progress = (uint8_t)(progressSum / 2);
    if (completed == 2) {
      motion.progress = 100;
      motion.state = DONE;
      stopMotors();
    }
  }
#endif
  appliedLeft = (int8_t)((motors[0].signedPwm + motors[2].signedPwm) * 100 / 510);
  appliedRight = (int8_t)((motors[1].signedPwm + motors[3].signedPwm) * 100 / 510);
}
static void copyTelemetryToPacket(Packet &p) {
#if SWARM_REAR_ENCODERS
  p.flags |= REAR_ENCODER_FLAG;
#endif
  noInterrupts();
  for (uint8_t i = 0; i < 4; ++i) p.pulses[i] = encoderPulses[i];
  interrupts();
  int64_t totalOdomTicks = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    p.rpmX10[i] = (int16_t)lroundf(actualRpm[i] * 10.0f);
    totalOdomTicks += wheelOdomTicks[i];
  }
  p.odomMm = (int32_t)lroundf(((float)totalOdomTicks / 2.0f) *
                             (3.14159265358979323846f * WHEEL_DIAMETER_MM /
                              ENCODER_TICKS_PER_WHEEL_REV));
  p.goalId = motion.goalId;
  p.mode = motion.mode;
  p.state = motion.state;
  p.progress = motion.progress;
  p.fault = motion.fault;
}
static void telemetryFromPacket(MotorTelemetry &t, const Packet &p) {
  t.rearEncoders = (p.flags & REAR_ENCODER_FLAG) != 0;
  for (uint8_t i = 0; i < 4; ++i) {
    t.pulses[i] = p.pulses[i]; t.rpmX10[i] = p.rpmX10[i];
  }
  t.odomMm = p.odomMm;
  t.goalId = p.goalId;
  t.mode = p.mode; t.state = p.state;
  t.progress = p.progress; t.fault = p.fault;
}
static void packetFromTelemetry(Packet &p, const MotorTelemetry &t) {
  if (t.rearEncoders) p.flags |= REAR_ENCODER_FLAG;
  for (uint8_t i = 0; i < 4; ++i) {
    p.pulses[i] = t.pulses[i]; p.rpmX10[i] = t.rpmX10[i];
  }
  p.odomMm = t.odomMm;
  p.goalId = t.goalId;
  p.mode = t.mode; p.state = t.state;
  p.progress = t.progress; p.fault = t.fault;
}
static Packet makePacket(uint8_t type) {
  Packet p = {};
  p.magic = MAGIC;
  p.version = VERSION;
  p.type = type;
  p.role = NODE_ROLE;
  p.group = NODE_GROUP;
  p.id = NODE_ID;
  p.boot = bootId;
  p.sequence = ++nextSeq;
  p.uptime = millis();
  p.ageMs = UINT16_MAX;
  return p;
}
static bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false; // Prototype only; see security notes in README.
  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) Serial.printf("[ESP-NOW] add peer failed: %d\n", (int)err);
  return err == ESP_OK;
}
static bool bindPeer(PeerState &peer, const uint8_t *mac, uint32_t now) {
  if (peer.bound && sameMac(peer.mac, mac)) {
    peer.lastHelloAt = now;
    return true;
  }
  // A duplicate ID cannot replace a recently seen robot.
  if (peer.bound && (uint32_t)(now - peer.lastHelloAt) < PEER_REPLACE_MS) return false;
  if (!addPeer(mac)) return false;
  if (peer.bound) esp_now_del_peer(peer.mac);
  peer = PeerState{};
  peer.bound = true;
  memcpy(peer.mac, mac, 6);
  peer.lastHelloAt = now;
  Serial.printf("[DISCOVERY] bound peer %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return true;
}
static bool enqueue(const uint8_t *mac, const Packet &p) {
  if (txUsed >= TX_QUEUE_SIZE) { txFailures++; return false; }
  memcpy(txQueue[txTail].mac, mac, 6);
  txQueue[txTail].data = p;
  txQueue[txTail].queuedAt = millis();
  txTail = (txTail + 1) % TX_QUEUE_SIZE;
  txUsed++;
  return true;
}
static void discardQueuedPackets() {
  // Motion STOP must not sit behind old commands in the local outbound queue.
  // This does not recall packets already transmitted by the radio.
  txHead = txTail = txUsed = 0;
}
static void transmitOne(uint32_t now) {
  if (txUsed == 0 || (uint32_t)(now - lastPhysicalTxAt) < TX_GAP_MS) return;
  TxEvent &t = txQueue[txHead];
  const uint32_t maxQueueAge =
      (t.data.type == WHEEL_COMMAND || t.data.type == GROUP_COMMAND) ? 200 :
      (t.data.type == HELLO ? 900 : 450);
  if ((uint32_t)(now - t.queuedAt) > maxQueueAge) {
    // Do not send stale commands or stale status when congestion clears.
    txHead = (txHead + 1) % TX_QUEUE_SIZE;
    txUsed--;
    txFailures++;
    return;
  }
  esp_err_t err = esp_now_send(t.mac, (const uint8_t *)&t.data, sizeof(Packet));
  if (err == ESP_ERR_ESPNOW_NO_MEM) return; // leave queued for a later retry
  if (err != ESP_OK) txFailures++;
  txHead = (txHead + 1) % TX_QUEUE_SIZE;
  txUsed--;
  lastPhysicalTxAt = now;
}

// ESP-NOW receive callbacks execute in a Wi-Fi context. Only copy into a queue.
static void receiveCallback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !info->src_addr || !data || len != (int)sizeof(Packet) || !rxQueue) return;
  RxEvent e;
  memcpy(e.mac, info->src_addr, 6);
  memcpy(&e.data, data, sizeof(e.data));
  if (xQueueSend(rxQueue, &e, 0) != pdTRUE) rxQueueDrops++;
}

static void receiveHello(const RxEvent &e, uint32_t now) {
  const Packet &p = e.data;
  if (NODE_ROLE == ROOT) {
    if (p.role == SECONDARY && p.group == GROUP_TWO && p.id == 0) {
      bindPeer(otherMaster, e.mac, now);
    } else if (p.role == SLAVE && p.group == GROUP_ONE &&
               p.id >= 1 && p.id <= SLAVES_PER_GROUP) {
      bindPeer(ownSlaves[p.id], e.mac, now);
    }
  } else if (NODE_ROLE == SECONDARY) {
    if (p.role == ROOT && p.group == GROUP_ONE && p.id == 0) {
      bindPeer(otherMaster, e.mac, now);
    } else if (p.role == SLAVE && p.group == GROUP_TWO &&
               p.id >= 1 && p.id <= SLAVES_PER_GROUP) {
      bindPeer(ownSlaves[p.id], e.mac, now);
    }
  } else {
    const uint8_t expectedRole = NODE_GROUP == GROUP_ONE ? ROOT : SECONDARY;
    if (p.role == expectedRole && p.group == NODE_GROUP && p.id == 0) {
      bindPeer(ownerMaster, e.mac, now);
    }
  }
}
static void receiveWheel(const RxEvent &e, uint32_t now) {
  if (NODE_ROLE != SLAVE) return;
  const Packet &p = e.data;
  if (!ownerMaster.bound || !sameMac(e.mac, ownerMaster.mac) ||
      p.role != (NODE_GROUP == GROUP_ONE ? ROOT : SECONDARY) ||
      p.group != NODE_GROUP || p.id != NODE_ID ||
      !acceptsCommand(p.boot, p.sequence, lastAcceptedBoot, lastAcceptedSequence)) return;
  lastAcceptedBoot = p.boot;
  lastAcceptedSequence = p.sequence;
  lastAppliedSequence = p.sequence;
  ownerMaster.lastStatusAt = now;  // time of last accepted motor command
  commandLocal(p.mode, {p.left, p.right}, p.originBoot, p.goalId,
               p.distanceMm, p.angleDeg, p.speedRpm, now);
}
static void receiveGroupCommand(const RxEvent &e, uint32_t now) {
  if (NODE_ROLE != SECONDARY) return;
  const Packet &p = e.data;
  if (!otherMaster.bound || !sameMac(e.mac, otherMaster.mac) ||
      p.role != ROOT || p.group != GROUP_ONE || p.id != 0 ||
      !acceptsCommand(p.boot, p.sequence, lastAcceptedBoot, lastAcceptedSequence)) return;
  lastAcceptedBoot = p.boot;
  lastAcceptedSequence = p.sequence;
  lastAppliedSequence = p.sequence;
  lastRootCommandAt = now;
  desiredTarget = p.target;
  desiredThrottle = p.throttle;
  desiredTurn = p.turn;
  desiredPattern = p.pattern;
  desiredMode = p.mode;
  desiredGoalId = p.goalId; desiredOriginBoot = p.originBoot;
  desiredDistanceMm = p.distanceMm; desiredAngleDeg = p.angleDeg;
  desiredSpeedRpm = p.speedRpm;
  rootFrame = p.frame;
}
static void receiveStatus(const RxEvent &e, uint32_t now) {
  const Packet &p = e.data;
  if (NODE_ROLE == SLAVE) return;
  if (p.role == SLAVE && p.group == NODE_GROUP &&
      p.id >= 1 && p.id <= SLAVES_PER_GROUP) {
    PeerState &peer = ownSlaves[p.id];
    if (peer.bound && sameMac(peer.mac, e.mac)) {
      peer.lastStatusAt = now;
      peer.ack = p.ack;
      peer.left = p.left;
      peer.right = p.right;
      telemetryFromPacket(peer.telemetry, p);
    }
  } else if (NODE_ROLE == ROOT && p.role == SECONDARY && p.group == GROUP_TWO && p.id == 0 &&
             otherMaster.bound && sameMac(otherMaster.mac, e.mac)) {
    otherMaster.lastStatusAt = now;
    otherMaster.ack = p.ack;
    otherMaster.left = p.left;
    otherMaster.right = p.right;
    telemetryFromPacket(otherMaster.telemetry, p);
  }
}
static void receiveRelay(const RxEvent &e, uint32_t now) {
  if (NODE_ROLE != ROOT || !otherMaster.bound || !sameMac(otherMaster.mac, e.mac)) return;
  const Packet &p = e.data;
  if (p.role != SECONDARY || p.group != GROUP_TWO || p.id < 1 || p.id > SLAVES_PER_GROUP) return;
  RemoteSlave &r = group2Slaves[p.id];
  r.received = true;
  r.reportedOnline = (p.flags & ONLINE_FLAG) != 0;
  r.lastRelayAt = now;
  r.ageAtRelay = p.ageMs;
  r.sent = p.sequence;
  r.ack = p.ack;
  r.left = p.left;
  r.right = p.right;
  telemetryFromPacket(r.telemetry, p);
}
static void processReceived(uint32_t now) {
  RxEvent e;
  uint8_t handled = 0;
  while (handled++ < 16 && xQueueReceive(rxQueue, &e, 0) == pdTRUE) {
    const Packet &p = e.data;
    if (!valid(p)) continue;
    switch (p.type) {
      case HELLO: receiveHello(e, now); break;
      case GROUP_COMMAND: receiveGroupCommand(e, now); break;
      case WHEEL_COMMAND: receiveWheel(e, now); break;
      case STATUS: receiveStatus(e, now); break;
      case RELAY_STATUS: receiveRelay(e, now); break;
      default: break;
    }
  }
}

static void sendHello(uint32_t now) {
  if ((uint32_t)(now - lastHelloAt) < DISCOVERY_MS) return;
  lastHelloAt = now;
  enqueue(BROADCAST, makePacket(HELLO));
}
static void sendGroupWheels(uint8_t mode, int throttle, int turn, uint8_t pattern,
                            uint32_t frame, uint32_t originBoot, uint32_t goalId,
                            int32_t distanceMm, int16_t angleDeg, uint16_t speedRpm,
                            uint32_t now) {
  Differential d = motorsFor(pattern, throttle, turn, 0, frame);
  commandLocal(mode, d, originBoot, goalId, distanceMm, angleDeg, speedRpm, now);
  for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
    PeerState &peer = ownSlaves[i];
    if (!peer.bound) continue;
    Packet p = makePacket(WHEEL_COMMAND);
    p.id = i; // target slave ID
    p.mode = mode;
    p.pattern = pattern;
    p.originBoot = originBoot; p.goalId = goalId;
    p.distanceMm = distanceMm;
    p.angleDeg = (mode == TURN_DEG && pattern == MIRROR && (i & 1u)) ?
                  -angleDeg : angleDeg;
    p.speedRpm = speedRpm;
    d = motorsFor(pattern, throttle, turn, i, frame);
    p.left = d.left; p.right = d.right;
    if (enqueue(peer.mac, p)) peer.lastCmdSentSeq = p.sequence;
  }
}
static bool isPeerOnline(const PeerState &p, uint32_t now) {
  return isFresh(p.lastStatusAt, PEER_ONLINE_MS, now);
}
static bool selectedRobotOnlineForTarget(uint8_t target, uint32_t now) {
  if (target != GROUP_TWO) {
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (!isPeerOnline(ownSlaves[i], now)) return false;
  }
  if (target != GROUP_ONE) {
    if (!isPeerOnline(otherMaster, now)) return false;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
      RemoteSlave &s = group2Slaves[i];
      if (!s.received || !s.reportedOnline ||
          !isFresh(s.lastRelayAt, PEER_ONLINE_MS, now) ||
          s.ageAtRelay == UINT16_MAX ||
          (uint32_t)(s.ageAtRelay + (now - s.lastRelayAt)) > PEER_ONLINE_MS) return false;
    }
  }
  return true;
}
static bool selectedRobotOnline(uint32_t now) {
  return selectedRobotOnlineForTarget(desiredTarget, now);
}
static bool selectedRobotsHaveRearEncoders(uint8_t target) {
  if (target != GROUP_TWO) {
#if !SWARM_REAR_ENCODERS
    return false;
#endif
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (!ownSlaves[i].telemetry.rearEncoders) return false;
  }
  if (target != GROUP_ONE) {
    if (!otherMaster.telemetry.rearEncoders) return false;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (!group2Slaves[i].telemetry.rearEncoders) return false;
  }
  return true;
}
static bool anySelectedFault(uint8_t target, uint32_t now) {
  (void)now;
  if (target != GROUP_TWO) {
    if (motion.fault) return true;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (ownSlaves[i].telemetry.fault) return true;
  }
  if (target != GROUP_ONE) {
    if (otherMaster.telemetry.fault) return true;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (group2Slaves[i].telemetry.fault) return true;
  }
  return false;
}
static bool allGoalDone(uint32_t now) {
  if (desiredTarget != GROUP_TWO) {
    if (motion.goalId != desiredGoalId || motion.state != DONE) return false;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
      const PeerState &s = ownSlaves[i];
      if (!isPeerOnline(s, now) || s.telemetry.goalId != desiredGoalId ||
          s.telemetry.state != DONE) return false;
    }
  }
  if (desiredTarget != GROUP_ONE) {
    if (!isPeerOnline(otherMaster, now) ||
        otherMaster.telemetry.goalId != desiredGoalId ||
        otherMaster.telemetry.state != DONE) return false;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
      const RemoteSlave &s = group2Slaves[i];
      if (s.telemetry.goalId != desiredGoalId || s.telemetry.state != DONE) return false;
    }
  }
  return true;
}
static bool anyGoalError(uint32_t now) {
  if (desiredTarget != GROUP_TWO) {
    if (motion.state == ERROR) return true;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (isPeerOnline(ownSlaves[i], now) && ownSlaves[i].telemetry.fault) return true;
  }
  if (desiredTarget != GROUP_ONE) {
    if (otherMaster.telemetry.fault) return true;
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i)
      if (group2Slaves[i].telemetry.fault) return true;
  }
  return false;
}
static void abortRootGoal(const char *reason) {
  rootGoalActive = false;
  desiredMode = STOP;
  desiredThrottle = desiredTurn = 0;
  lastMissionResult = reason;
  discardQueuedPackets(); // abort beats locally queued movement commands
  commandLocal(STOP, {0,0}, 0, 0, 0, 0, 0, millis());
  lastMotorUpdateAt = millis() - MOTOR_UPDATE_MS;
}
static void checkRootGoal(uint32_t now) {
  if (NODE_ROLE != ROOT || !rootGoalActive) return;
  if (!isFresh(lastPhoneStatusAt, PHONE_MISSION_HEARTBEAT_MS, now)) {
    abortRootGoal("aborted: dashboard heartbeat lost"); return;
  }
  if ((uint32_t)(now - rootGoalStartedAt) > MAX_GOAL_RUNTIME_MS) {
    abortRootGoal("aborted: maximum mission duration"); return;
  }
  if (anyGoalError(now)) {
    abortRootGoal("aborted: encoder or link fault"); return;
  }
  if ((uint32_t)(now - rootGoalStartedAt) > GOAL_GROUP_GRACE_MS &&
      !selectedRobotOnline(now)) {
    abortRootGoal("aborted: selected robot offline"); return;
  }
  if (allGoalDone(now)) {
    abortRootGoal("completed: all selected robots reached encoder targets");
  }
}
static void updateMotorsAndCommands(uint32_t now) {
  if (NODE_ROLE == SLAVE) {
    if (motion.state == RUNNING &&
        !isFresh(ownerMaster.lastStatusAt, SLAVE_LINK_DEADMAN_MS, now))
      failMotion(LINK_TIMEOUT);
    return;
  }
  if ((uint32_t)(now - lastMotorUpdateAt) < MOTOR_UPDATE_MS) return;
  lastMotorUpdateAt = now;
  uint8_t mode = STOP;
  int t = 0, turn = 0;
  uint8_t pattern = FOLLOW;
  uint32_t frame = 0;
  if (NODE_ROLE == ROOT) {
    if (desiredMode == MANUAL && isFresh(lastPhoneAt, PHONE_DEADMAN_MS, now)) {
      mode = MANUAL;
      t = desiredThrottle; turn = desiredTurn; pattern = desiredPattern;
    } else if (rootGoalActive && (desiredMode == DRIVE_MM || desiredMode == TURN_DEG)) {
      mode = desiredMode;
      pattern = desiredPattern;
    }
    frame = now / 750;
    const uint8_t localMode = (desiredTarget != 0 && desiredTarget != GROUP_ONE) ? (uint8_t)STOP : mode;
    const int lt = localMode == MANUAL ? t : 0;
    const int lr = localMode == MANUAL ? turn : 0;
    const int16_t localAngle = (localMode == TURN_DEG &&
                   pattern == MIRROR && (NODE_ID & 1u)) ? -desiredAngleDeg : desiredAngleDeg;
    sendGroupWheels(localMode, lt, lr, pattern, frame, desiredOriginBoot,
                    desiredGoalId, desiredDistanceMm, localAngle,
                    desiredSpeedRpm, now);
    if (otherMaster.bound) {
      Packet p = makePacket(GROUP_COMMAND);
      p.target = desiredTarget;
      p.mode = mode;
      p.pattern = pattern;
      p.frame = frame;
      p.throttle = t; p.turn = turn;
      p.originBoot = desiredOriginBoot; p.goalId = desiredGoalId;
      p.distanceMm = desiredDistanceMm; p.angleDeg = desiredAngleDeg;
      p.speedRpm = desiredSpeedRpm;
      if (enqueue(otherMaster.mac, p)) otherMaster.lastCmdSentSeq = p.sequence;
    }
  } else { // SECONDARY
    if (motion.state == RUNNING &&
        !isFresh(lastRootCommandAt, MASTER_LINK_DEADMAN_MS, now))
      failMotion(LINK_TIMEOUT);
    const bool active = isFresh(lastRootCommandAt, MASTER_LINK_DEADMAN_MS, now);
    if (active && (desiredTarget == 0 || desiredTarget == GROUP_TWO)) {
      mode = desiredMode;
      pattern = desiredPattern;
      if (mode == MANUAL) { t = desiredThrottle; turn = desiredTurn; }
    }
    frame = rootFrame;
    sendGroupWheels(mode, t, turn, pattern, frame, desiredOriginBoot,
                    desiredGoalId, desiredDistanceMm, desiredAngleDeg,
                    desiredSpeedRpm, now);
  }
}
static void sendTelemetry(uint32_t now) {
  if ((uint32_t)(now - lastTelemetryAt) < TELEMETRY_MS) return;
  lastTelemetryAt = now;
  if (NODE_ROLE == SLAVE) {
    if (ownerMaster.bound) {
      Packet p = makePacket(STATUS);
      p.ack = lastAppliedSequence;
      p.left = appliedLeft; p.right = appliedRight;
      copyTelemetryToPacket(p);
      enqueue(ownerMaster.mac, p);
    }
  } else if (NODE_ROLE == SECONDARY) {
    if (!otherMaster.bound) return;
    Packet m = makePacket(STATUS);
    m.ack = lastAppliedSequence;
    m.left = appliedLeft; m.right = appliedRight;
    copyTelemetryToPacket(m);
    enqueue(otherMaster.mac, m);
    for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
      const PeerState &peer = ownSlaves[i];
      Packet p = makePacket(RELAY_STATUS);
      p.id = i;
      p.flags = isFresh(peer.lastStatusAt, PEER_ONLINE_MS, now) ? ONLINE_FLAG : 0;
      const uint32_t age = (uint32_t)(now - peer.lastStatusAt);
      p.ageMs = peer.lastStatusAt == 0 || age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
      p.sequence = peer.lastCmdSentSeq;
      p.ack = peer.ack; p.left = peer.left; p.right = peer.right;
      packetFromTelemetry(p, peer.telemetry);
      enqueue(otherMaster.mac, p);
    }
  }
}
// ----------------------- PHONE DASHBOARD (M1 ONLY) -----------------------
static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Swarm V4</title><style>
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#0c1220;color:#e9f2fe}*{box-sizing:border-box}
body{max-width:980px;margin:auto;padding:16px}h1{font-size:1.55rem}h2{font-size:1.06rem;margin:0 0 12px}
.panel{padding:16px;border-radius:14px;background:#172338;border:1px solid #30405d;margin:12px 0}
label{display:block;margin:9px 0 5px;color:#c1cfe0}input,select{width:100%;min-height:40px;font-size:16px;background:#101b2d;color:#fff;border:1px solid #475c7a;border-radius:8px;padding:7px}
button:disabled{opacity:.35;cursor:not-allowed}button{border:0;border-radius:9px;background:#3678d6;color:white;padding:13px;font-size:16px;touch-action:none;user-select:none}
.buttons{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));max-width:370px;margin:auto;gap:9px}.pad{height:65px;font-size:24px}
.stop{background:#c2394f;width:100%;font-weight:bold;margin-top:12px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.row{display:grid;grid-template-columns:1fr 1fr;gap:8px}.wide{grid-column:1 / -1}
small,.note{font-size:13px;color:#a7b8d2}.ok{color:#83efb3}.bad{color:#ff969f}
table{border-collapse:collapse;width:100%;font-size:13px;min-width:690px}td,th{text-align:left;padding:7px 5px;border-bottom:1px solid #32445e}#tablewrap{overflow:auto}
@media(max-width:640px){.grid{grid-template-columns:1fr}}
</style></head><body>
<h1>8-Car Dual-Master Swarm V4</h1><p class="note">M1 ⇄ M2 via ESP-NOW | Each master ⇄ its three slaves | One phone dashboard</p>
<div class="panel"><h2>Target robots</h2>
<select id="target"><option value="0">Both groups (8 cars)</option><option value="1">M1 + S1–S3</option><option value="2">M2 + S4–S6</option></select>
<label for="pattern">Manual pattern / turn pattern</label><select id="pattern">
<option value="0">Follow</option><option value="1">Mirror (odd slave turns opposite)</option><option value="2">Spin (manual only)</option><option value="3">Wave (manual only)</option></select></div>
<div class="grid">
<div class="panel"><h2>Manual drive (rear encoder PI or open-loop PWM)</h2>
<label>Motor demand <b id="powerval">35</b>% of calibrated maximum RPM</label><input id="power" type="range" min="20" max="70" value="35">
<div class="buttons" style="margin-top:13px"><span></span><button class="pad" data-key="forward">▲</button><span></span>
<button class="pad" data-key="left">◀</button><button class="pad" data-key="reverse">▼</button><button class="pad" data-key="right">▶</button></div>
<p class="note">Hold to drive, release to stop. W/A/S/D and arrow keys on desktop. Any new manual input interrupts autonomous movement.</p></div>
<div class="panel"><h2>Distance / turn (rear-encoder cars only)</h2>
<label for="distance">Drive distance (cm; negative = reverse)</label><input id="distance" type="number" min="-300" max="300" step="1" value="50">
<label for="angle">Turn angle (degrees; positive = clockwise)</label><input id="angle" type="number" min="-360" max="360" value="90" step="1">
<label for="rpm">Target wheel speed (RPM)</label><input id="rpm" type="number" min="20" max="70" value="35" step="1">
<div class="row" style="margin-top:12px"><button id="goDist">Drive cm</button><button id="goTurn">Turn degrees</button></div>
<p class="note">Relative encoder targets; wheel slip and track-width calibration affect real motion. Do not run without working encoders.</p></div>
</div>
<div class="panel"><button id="stop" class="stop">STOP BOTH GROUPS</button>
<p id="mission" class="note">No mission started.</p><p id="health" class="note">Waiting for telemetry...</p></div>
<div class="panel"><h2>Robot feedback (— = no sensor)</h2><div id="tablewrap">
<table><thead><tr><th>Robot</th><th>Encoders</th><th>Online</th><th>Telemetry age</th><th>Ack / Sent seq</th><th>Applied L/R %</th><th>Goal status</th><th>Progress</th><th>Encoder counts RL/RR</th><th>Measured RPM RL/RR</th><th>Wheel odometry</th><th>Fault</th></tr></thead><tbody id="nodes"></tbody></table></div>
<p class="note">Encoder travel is wheel rotation, not GPS/absolute position. One-channel rear-encoder direction is inferred from motor command. Uninstrumented cars report no measured distance.</p></div>
<small>3.3-V encoder signals only. Use a physical motor-power cutoff. Dashboard loss stops goals after timeout.</small>
<script>
const target=document.getElementById('target'),pattern=document.getElementById('pattern'),power=document.getElementById('power');
power.addEventListener('input',()=>document.getElementById('powerval').textContent=power.value);
const held={forward:false,reverse:false,left:false,right:false}, keys={ArrowUp:'forward',ArrowDown:'reverse',ArrowLeft:'left',ArrowRight:'right',w:'forward',s:'reverse',a:'left',d:'right'};
let seq=0,sending=false,lastMotion=false,goalStarted=false;
function nextSeq(){seq=Math.max(Date.now(),seq+1);return seq;}
async function post(url){const r=await fetch(url,{method:'POST',cache:'no-store'});if(!r.ok)throw new Error(await r.text());return r.json();}
function clearHeld(){for(const key in held)held[key]=false;}
function stopAll(){clearHeld();lastMotion=false;goalStarted=false;post('/api/stop?seq='+nextSeq()).catch(()=>{});}
for(const b of document.querySelectorAll('[data-key]')){
 const key=b.dataset.key;
 b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);held[key]=true;});
 for(const event of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(event,()=>{held[key]=false;});
}
window.addEventListener('keydown',e=>{const key=keys[e.key];if(key){e.preventDefault();held[key]=true;}});
window.addEventListener('keyup',e=>{const key=keys[e.key];if(key){e.preventDefault();held[key]=false;}});
window.addEventListener('blur',stopAll);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll();});
for(const x of [target,pattern])x.addEventListener('change',stopAll);
document.getElementById('stop').addEventListener('click',stopAll);
async function drive(){
 if(sending)return;
 const v=Number(power.value),t=(held.forward?v:0)-(held.reverse?v:0),r=(held.right?v:0)-(held.left?v:0);
 if(!t&&!r&&!lastMotion)return;
 lastMotion=Boolean(t||r);sending=true;
 try {await post('/api/drive?throttle='+t+'&turn='+r+'&pattern='+pattern.value+'&target='+target.value+'&seq='+nextSeq());}
 catch(e){document.getElementById('mission').textContent='Manual command error: '+e.message;}
 finally{sending=false;}
}
setInterval(drive,120);
async function launch(kind){
 if(sending)return;
 clearHeld();lastMotion=false;sending=true;
 const n=kind==='distance'?Number(document.getElementById('distance').value)*10:Number(document.getElementById('angle').value);
 const rpm=Number(document.getElementById('rpm').value);
 if(!Number.isFinite(n)||!Number.isInteger(n)||!Number.isInteger(rpm)||n===0){sending=false;alert('Enter a nonzero whole-number distance/angle and a valid RPM.');return;}
 try{
  const q='kind='+encodeURIComponent(kind)+'&value='+n+'&rpm='+rpm+'&target='+target.value+'&pattern='+pattern.value+'&seq='+nextSeq();
  const r=await post('/api/move?'+q);goalStarted=true;
  document.getElementById('mission').textContent='Goal '+r.goalId+' accepted; tracking...';
 }catch(e){document.getElementById('mission').textContent='Goal rejected: '+e.message;}
 finally{sending=false;}
}
document.getElementById('goDist').addEventListener('click',()=>launch('distance'));
document.getElementById('goTurn').addEventListener('click',()=>launch('turn'));
function esc(x){return String(x).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
const stateNames=['idle','running','done','error'],faultNames=['—','encoder stall/no pulses','motion timeout','radio timeout','encoders required'];
async function refresh(){
 try{
  const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
  const s=await r.json();
  document.getElementById('health').textContent='Phone manual '+(s.phoneActive?'active':'idle')+' · Goal '+(s.goalActive?'active':'idle')+' · TX errors '+s.txErrors+' · RX drops '+s.rxDrops;
  document.getElementById('mission').textContent=s.missionResult+' | Goal ID '+s.goalId;
  document.getElementById('nodes').innerHTML=s.nodes.map(n=>'<tr><td>'+esc(n.name)+'</td><td>'+(n.rearEncoders?'rear × 2':'none')+'</td><td class="'+(n.online?'ok':'bad')+'">'+(n.online?'yes':'NO')+'</td><td>'+(n.ageMs==null?'—':n.ageMs+' ms')+'</td><td>'+n.ackSeq+' / '+n.sentSeq+'</td><td>'+n.leftPwm+' / '+n.rightPwm+'</td><td>'+esc(stateNames[n.state]||'?')+'</td><td>'+n.progress+'%</td><td>'+(n.rearEncoders?esc(n.ticks.slice(2).join('/')):'—')+'</td><td>'+(n.rearEncoders?esc(n.rpm.slice(2).map(v=>v.toFixed(1)).join('/')):'—')+'</td><td>'+(n.rearEncoders?n.odomMm+' mm':'—')+'</td><td class="'+(n.fault?'bad':'ok')+'">'+esc(faultNames[n.fault]||'?')+'</td></tr>').join('');
  const chosen=Number(target.value), members=chosen===1?s.nodes.slice(0,4):chosen===2?s.nodes.slice(4,8):s.nodes;
  const canGoal=members.length===4*(chosen===0?2:1)&&members.every(n=>n.online&&n.rearEncoders&&!n.fault);
  for(const id of ['goDist','goTurn'])document.getElementById(id).disabled=!canGoal;
 }catch(e){document.getElementById('health').textContent='Dashboard disconnected. Stop watchdogs should activate.';}
}
setInterval(refresh,450);refresh();
</script></body></html>
)HTML";

static void appendNode(String &out, const char *name, bool online, int32_t ageMs,
                       const MotorTelemetry &t, uint32_t sent, uint32_t ack, int8_t left, int8_t right) {
  if (out.length() > 10 && out[out.length() - 1] != '[') out += ',';
  out += "{\"name\":\""; out += name;
  out += "\",\"online\":"; out += online ? "true" : "false";
  out += ",\"rearEncoders\":"; out += t.rearEncoders ? "true" : "false";
  out += ",\"ageMs\":";
  if (ageMs < 0) out += "null"; else out += String(ageMs);
  out += ",\"sentSeq\":" + String(sent);
  out += ",\"ackSeq\":" + String(ack);
  out += ",\"leftPwm\":" + String(left);
  out += ",\"rightPwm\":" + String(right);
  out += ",\"goalId\":" + String(t.goalId);
  out += ",\"mode\":" + String(t.mode);
  out += ",\"state\":" + String(t.state);
  out += ",\"progress\":" + String(t.progress);
  out += ",\"fault\":" + String(t.fault);
  out += ",\"odomMm\":" + String(t.odomMm);
  out += ",\"ticks\":[";
  for (uint8_t i = 0; i < 4; ++i) {if(i)out+=',';out+=String(t.pulses[i]);}
  out += "],\"rpm\":[";
  for (uint8_t i = 0; i < 4; ++i) {if(i)out+=',';out+=String(t.rpmX10[i] / 10.0f, 1);}
  out += "]}";
}
static void handleStatus() {
  uint32_t now = millis();
  lastPhoneStatusAt = now; // autonomous mission heartbeat while dashboard is visible
  String json;
  json.reserve(3400);
  json += "{\"phoneActive\":";
  json += isFresh(lastPhoneAt, PHONE_DEADMAN_MS, now) ? "true" : "false";
  json += ",\"goalActive\":"; json += rootGoalActive ? "true" : "false";
  json += ",\"goalId\":" + String(desiredGoalId);
  json += ",\"missionResult\":\"" + lastMissionResult + "\"";
  json += ",\"txErrors\":" + String(txFailures);
  json += ",\"rxDrops\":" + String(rxQueueDrops);
  json += ",\"nodes\":[";
  Packet own = {};
  copyTelemetryToPacket(own);
  MotorTelemetry local;
  telemetryFromPacket(local, own);
  appendNode(json, "M1 (group 1)", true, 0, local, 0, 0, appliedLeft, appliedRight);
  for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
    char name[16]; snprintf(name, sizeof(name), "S%u (group 1)", i);
    PeerState &v = ownSlaves[i];
    appendNode(json, name, isPeerOnline(v, now),
               v.lastStatusAt ? (int32_t)(now - v.lastStatusAt) : -1,
               v.telemetry, v.lastCmdSentSeq, v.ack, v.left, v.right);
  }
  appendNode(json, "M2 (group 2)", isPeerOnline(otherMaster, now),
             otherMaster.lastStatusAt ? (int32_t)(now - otherMaster.lastStatusAt) : -1,
             otherMaster.telemetry, otherMaster.lastCmdSentSeq, otherMaster.ack,
             otherMaster.left, otherMaster.right);
  for (uint8_t i = 1; i <= SLAVES_PER_GROUP; ++i) {
    char name[16]; snprintf(name, sizeof(name), "S%u (group 2)", i + 3);
    RemoteSlave &v = group2Slaves[i];
    int32_t age = v.received && v.ageAtRelay != UINT16_MAX ?
                  (int32_t)(v.ageAtRelay + (uint32_t)(now - v.lastRelayAt)) : -1;
    bool online = isPeerOnline(otherMaster, now) && v.received &&
                  v.reportedOnline && isFresh(v.lastRelayAt, PEER_ONLINE_MS, now) &&
                  age >= 0 && age <= (int32_t)PEER_ONLINE_MS;
    appendNode(json, name, online, age, v.telemetry, v.sent, v.ack, v.left, v.right);
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}
static uint64_t parseBrowserSequence() {
  if (!server.hasArg("seq")) return 0;
  const String raw = server.arg("seq");
  if (raw.isEmpty()) return 0;
  for (size_t i = 0; i < raw.length(); ++i)
    if (raw[i] < '0' || raw[i] > '9') return 0;
  return strtoull(raw.c_str(), nullptr, 10);
}
static bool acceptBrowserSequence() {
  uint64_t v = parseBrowserSequence();
  if (!v || v <= lastBrowserSequence) return false;
  lastBrowserSequence = v;
  return true;
}
static void handleDrive() {
  if (!server.hasArg("throttle") || !server.hasArg("turn") ||
      !server.hasArg("pattern") || !server.hasArg("target") ||
      !server.hasArg("seq")) {
    server.send(400, "text/plain", "Missing parameters"); return;
  }
  int p = server.arg("pattern").toInt(), target = server.arg("target").toInt();
  if (p < FOLLOW || p > WAVE || target < 0 || target > 2) {
    server.send(400, "text/plain", "Invalid pattern or target"); return;
  }
  if (!acceptBrowserSequence()) {
    server.send(409, "text/plain", "Old browser command ignored"); return;
  }
  rootGoalActive = false;
  desiredMode = MANUAL;
  desiredThrottle = limit(server.arg("throttle").toInt());
  desiredTurn = limit(server.arg("turn").toInt());
  desiredPattern = (uint8_t)p;
  desiredTarget = (uint8_t)target;
  lastPhoneAt = millis();
  lastMissionResult = "manual control";
  // High priority: don't let a queued goal resume after manual takeover.
  discardQueuedPackets();
  lastMotorUpdateAt = millis() - MOTOR_UPDATE_MS;
  server.send(200, "application/json", "{\"ok\":true}");
}
static void handleMove() {
  if (!server.hasArg("kind") || !server.hasArg("value") || !server.hasArg("rpm") ||
      !server.hasArg("pattern") || !server.hasArg("target") || !server.hasArg("seq")) {
    server.send(400, "text/plain", "Missing parameters"); return;
  }
  String kind = server.arg("kind");
  int32_t amount = server.arg("value").toInt();
  int rpm = server.arg("rpm").toInt();
  int p = server.arg("pattern").toInt(), target = server.arg("target").toInt();
  if ((kind != "distance" && kind != "turn") ||
      (kind == "distance" && (amount < -3000 || amount > 3000 || amount == 0)) ||
      (kind == "turn" && (amount < -360 || amount > 360 || amount == 0)) ||
      rpm < 20 || rpm > 70 || p > MIRROR || p < FOLLOW || target < 0 || target > 2) {
    server.send(400, "text/plain", "Use 1..300 cm, 1..360 degrees, 20..70 RPM, Follow/Mirror"); return;
  }
  const uint32_t now = millis();
  if (!selectedRobotOnlineForTarget((uint8_t)target, now)) {
    server.send(409, "text/plain", "A selected robot has no recent encoder telemetry; check all 8 links"); return;
  }
  if (rootGoalActive) {
    server.send(409, "text/plain", "Goal already active: press STOP before starting a new one"); return;
  }
  if (!selectedRobotsHaveRearEncoders((uint8_t)target)) {
    server.send(409, "text/plain", "Selected group has a car without rear encoders: distance/angle unavailable"); return;
  }
  if (anySelectedFault((uint8_t)target, now)) {
    server.send(409, "text/plain", "A robot has a latched fault: inspect wiring and reboot safely"); return;
  }
  if (!acceptBrowserSequence()) {
    server.send(409, "text/plain", "Old browser command ignored"); return;
  }
  discardQueuedPackets();
  desiredMode = kind == "distance" ? DRIVE_MM : TURN_DEG;
  desiredDistanceMm = desiredMode == DRIVE_MM ? amount : 0;
  desiredAngleDeg = desiredMode == TURN_DEG ? (int16_t)amount : 0;
  desiredSpeedRpm = (uint16_t)rpm;
  desiredPattern = (uint8_t)p;
  desiredTarget = (uint8_t)target;
  desiredThrottle = desiredTurn = 0;
  desiredOriginBoot = bootId;
  desiredGoalId = ++rootGoalCounter;
  if (!desiredGoalId) desiredGoalId = ++rootGoalCounter;
  rootGoalStartedAt = now;
  lastPhoneStatusAt = now;
  lastMissionResult = "running";
  rootGoalActive = true;
  lastMotorUpdateAt = now - MOTOR_UPDATE_MS;
  server.send(200, "application/json", "{\"ok\":true,\"goalId\":" + String(desiredGoalId) + "}");
}
static void handleStop() {
  uint64_t v = parseBrowserSequence();
  if (v > lastBrowserSequence) lastBrowserSequence = v;
  rootGoalActive = false;
  discardQueuedPackets();
  desiredMode = STOP;
  desiredThrottle = desiredTurn = 0;
  desiredTarget = 0;
  lastPhoneAt = 0;
  lastMissionResult = "stopped by operator";
  commandLocal(STOP, {0,0}, 0, 0, 0, 0, 0, millis());
  lastMotorUpdateAt = millis() - MOTOR_UPDATE_MS;
  server.send(200, "application/json", "{\"stopped\":true}");
}
static void startWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/move", HTTP_POST, handleMove);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

static void printIdentity() {
  uint8_t mac[6] = {};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("Role=%s group=%u id=%u channel=%u STA MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
     roleName(NODE_ROLE), NODE_GROUP, NODE_ID, ESPNOW_CHANNEL,
     mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
void setup() {
  Serial.begin(115200);
  delay(150);
  if (!initMotors()) {
    Serial.println("FATAL: Failed to attach four LEDC PWM channels");
    while (true) { stopMotors(); delay(250); }
  }
  initEncoders();
  bootId = esp_random();
  if (!bootId) bootId = 1;
  rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(RxEvent));
  if (!rxQueue) {
    Serial.println("FATAL: Cannot allocate receive queue");
    while (true) { stopMotors(); delay(250); }
  }
  WiFi.persistent(false);
  WiFi.setSleep(false);
  if (NODE_ROLE == ROOT) {
    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAP(PHONE_AP_NAME, PHONE_AP_PASSWORD, ESPNOW_CHANNEL, 0, 2)) {
      Serial.println("FATAL: Cannot start the phone Wi-Fi network");
      while (true) { stopMotors(); delay(250); }
    }
    Serial.printf("Connect phone to %s, open http://%s\n",
                  PHONE_AP_NAME, WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
  }
  // Set the same radio channel for ESP-NOW on every board.
  esp_err_t channelErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelErr != ESP_OK) {
    Serial.printf("FATAL: Wi-Fi channel could not be set (%d)\n", (int)channelErr);
    while (true) { stopMotors(); delay(250); }
  }
  if (esp_now_init() != ESP_OK || !addPeer(BROADCAST)) {
    Serial.println("FATAL: ESP-NOW init failed");
    while (true) { stopMotors(); delay(250); }
  }
  esp_now_register_recv_cb(receiveCallback);
  printIdentity();
  if (NODE_ROLE == ROOT) startWeb();
  Serial.printf("Firmware V4 | rear encoders: %s\n", SWARM_REAR_ENCODERS ? "YES" : "NO");
  Serial.println("Ready; motors stopped. Discovery starts automatically.");
}
void loop() {
  if (NODE_ROLE == ROOT) server.handleClient();
  const uint32_t now = millis();
  processReceived(now);
  updateMotorController(now);
  checkRootGoal(now);
  sendHello(now);
  updateMotorsAndCommands(now);
  sendTelemetry(now);
  transmitOne(now);
  delay(1);
}
