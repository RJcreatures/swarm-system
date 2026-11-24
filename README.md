# ESP32 dual-master swarm — final V4 prototype package

**Two build variants | 2 masters + 6 slaves | 4 driven wheels per car | two-way ESP-NOW | phone web dashboard**

> **Status:** Host-side simulations, source syntax checks with ESP32 API stubs, and dashboard JavaScript syntax tests pass. **Real ESP32 compilation and physical/wireless/motor testing have NOT yet been performed**. Do **not** call this hardware-verified or deploy unsupervised. Test with wheels elevated and a separate physical motor-power disconnect.

## Choose your firmware

| Version | Source directory | Encoder hardware | What the car can do |
|---|---|---|---|
| **A — No Encoders** | [`firmware/No_Encoders/SwarmRobot`](firmware/No_Encoders/SwarmRobot) | No encoder pins | Manual forward/reverse/turn, Follow/Mirror/Spin/Wave patterns, phone feedback for connection, ack, applied PWM/command and faults. **Cannot measure distance or speed.** |
| **B — Rear Encoders** | [`firmware/Rear_Encoders/SwarmRobot`](firmware/Rear_Encoders/SwarmRobot) | One encoder A signal on each *rear* wheel (2 total per car) | Everything above plus rear-wheel PI speed tracking, estimated relative travel, distance and turn goals, left/right pulse and RPM telemetry and missing-encoder stall detection. Front motors copy the corresponding rear-side drive PWM and **are not speed-measured independently**. |

Both versions use **the exact same V4 ESP-NOW packet format**, so you can mix the two car types. Flash *all eight* devices with V4; V2/V3 firmware cannot interoperate. **Drive N cm** and **Turn N degrees** work only if *every vehicle in the selected group* is encoder-equipped and online. You can operate a mixed group in manual mode.

## Architecture

```text
Phone -- Wi-Fi SoftAP + HTTP dashboard --> M1 (ROOT, Group 1)
                                             |             \
                        ESP-NOW two-way       |              \ two-way ESP-NOW
                          M1 <-> M2           |               S1 S2 S3
                                             M2 (SECONDARY, Group 2)
                                              \ two-way ESP-NOW
                                                S4 S5 S6

Phone -> M1 -> M2 -> group 2 slaves: commands
Group 2 slaves -> M2 -> M1 -> Phone: status and feedback
M1 controls *only* Group 1 slaves; M2 controls *only* Group 2 slaves.
```

The phone connects only to M1. M1 is also a driving car. M2 is also a driving car. Acknowledgment means **a command was accepted/processed by firmware**, not proof that a non-encoder wheel rotated. For the rear-encoder version the dashboard also displays locally measured encoder pulses/RPM/estimated distance. ESP-NOW is not a time-synchronized trajectory engine; no physical spacing, absolute localization, or obstacle detection is provided.

## Hardware assumptions — verify before applying power

**Per four-wheel car:** one **classic ESP32-WROOM-32 DevKit** (not ESP32-C3/S3), four DC gear motors, **TWO L298N dual-channel modules** (front board A and rear board B), a motor battery, appropriately sized regulator for ESP32, a fuse and hardware motor-power cutoff. Variant B additionally requires two rear-wheel encoders with A pulse outputs. There are **16 L298N boards for all eight cars** if built as written.

The L298N has two independently controlled H-bridge channels; a **single L298N is not four independent motor channels**. Do not parallel two motors on a channel without checking their **combined stall current**, wire sizing and actual driver's thermal limits; L298N boards often overheat under 4-wheel loads. A lower-loss modern driver may be a better hardware choice.

### Identical L298N motor wiring on all eight cars

**Remove ENA and ENB enable jumpers on BOTH L298N boards** to allow ESP32 PWM. Add a suitable ~10-kΩ pulldown on each enable line to keep the motors disabled during ESP32 boot. Each motor connects directly to its listed driver's `OUT1/2` or `OUT3/4` pair.

| Location / motor | Driver board | Enable pin | Direction pin 1 | Direction pin 2 |
|---|---|---:|---:|---:|
| Front left (FL) | A, channel A (`ENA/IN1/IN2`) | GPIO25 | GPIO26 | GPIO27 |
| Front right (FR) | A, channel B (`ENB/IN3/IN4`) | GPIO33 | GPIO32 | GPIO23 |
| Rear left (RL) | B, channel A (`ENA/IN1/IN2`) | GPIO18 | GPIO19 | GPIO13 |
| Rear right (RR) | B, channel B (`ENB/IN3/IN4`) | GPIO17 | GPIO16 | GPIO14 |

**Power and grounding:** join ESP32 GND, both L298N GNDs, encoder ground (if used), DC-DC ground and motor battery negative; feed motors from the appropriate motor power supply, **never from an ESP32 GPIO or its 3.3-V output**. Power ESP32 from USB or a correctly regulated input pin. **Do not connect arbitrary 5-V outputs to the ESP32 3.3-V pin.** Read your L298N board's `5V-EN` / regulator jumper instructions; modules vary. Size the fuse, battery, cables, and drivers for **measured motor stall current**, and add a physical motor-power cutoff usable without Wi-Fi.

### Variant B: just TWO encoder connections per car

| Actual rear encoder | A output | ESP32 | VCC/GND |
|---|---|---|---|
| Rear-left (RL) | `A` | **GPIO34** | Supply according to encoder datasheet; shared GND |
| Rear-right (RR) | `A` | **GPIO35** | Supply according to encoder datasheet; shared GND |

Only one A signal is used per rear wheel, even if your motor has quadrature A/B. GPIO34 and GPIO35 are input-only and have **no internal pull-up**. If the sensor's output is open-collector, fit an external 4.7-kΩ to 10-kΩ pull-up **to 3.3 V**. If the sensor produces 5-V push-pull output, use a logic level shifter. Never apply 5 V directly to ESP32 GPIO. **GPIO36/39 are NOT used in V4.** Front wheel encoders, even if fitted, are not sampled.

Single-channel encoders report pulses but not the actual rotation direction: sign is **inferred from the issued motor direction**. Odometer estimates will be wrong while someone pushes the car, when it rolls after motor stop, or while wheels slip. With four-wheel skid steering, turns need calibration for the surface; accurate formation keeping requires additional sensors.

## Install from scratch (Arduino IDE)

1. Install [Arduino IDE 2](https://www.arduino.cc/en/software) on a Windows, macOS or Linux PC, and connect **one ESP32 at a time** using a known *data-capable* USB cable. Do not connect motor power during first flashing.
2. In Arduino IDE open **File → Preferences → Additional Boards Manager URLs** and add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Open **Tools → Board → Boards Manager**, search `esp32` by Espressif Systems and install an **Arduino-ESP32 3.3.x** release (e.g. `3.3.12`). **Do not choose 4.x until tested**; these sketches target API 3.x.
4. Download/unzip this package. Choose `firmware/No_Encoders/SwarmRobot/SwarmRobot.ino` for a car with no sensors, or `firmware/Rear_Encoders/SwarmRobot/SwarmRobot.ino` for a car with two rear encoder signals. **Do not copy `.ino` alone:** `RobotConfig.h` and `SwarmProtocol.h` are required in the SAME sketch directory.
5. In the IDE set board **ESP32 Dev Module** (classic ESP32) and select the USB serial port under **Tools → Port**.
6. Edit the first three `NODE_...` values in `RobotConfig.h` using the identity table below. Set a personal 8+-character `PHONE_AP_PASSWORD` for M1 (copy config to other builds if convenient). Check the firmware variant flag stays `0` for No Encoders or `1` for Rear Encoders.
7. Click **Verify** to compile **before connecting motors**. Correct compile errors if any; the provided host stub checks are **not** proof of an actual Espressif board build.
8. Click **Upload**, then open **Tools → Serial Monitor** at **115200 baud**. Confirm the line includes the correct role/group/id, the same ESP-NOW channel (`6`), and `Firmware V4 | rear encoders: YES/NO`.
9. Disconnect USB/motor power as appropriate, change ID and repeat for **all eight ESP32s**. Never use the same group+ID for two active nodes.
10. Test every motor and encoder **with wheels off the ground** before ground movement. Follow the acceptance checklist below.

### Set identity for each upload

These are the three editable lines at the top of `RobotConfig.h`:

```cpp
#define NODE_ROLE Swarm::ROOT
#define NODE_GROUP 1
#define NODE_ID 0
```

| Robot | Role | Group | ID | Commands from |
|---|---|---:|---:|---|
| M1 | `Swarm::ROOT` | 1 | 0 | phone dashboard |
| S1 | `Swarm::SLAVE` | 1 | 1 | M1 only |
| S2 | `Swarm::SLAVE` | 1 | 2 | M1 only |
| S3 | `Swarm::SLAVE` | 1 | 3 | M1 only |
| M2 | `Swarm::SECONDARY` | 2 | 0 | M1 only |
| S4 | `Swarm::SLAVE` | 2 | 1 | M2 only |
| S5 | `Swarm::SLAVE` | 2 | 2 | M2 only |
| S6 | `Swarm::SLAVE` | 2 | 3 | M2 only |

Set `FL_PINS`, `FR_PINS`, `RL_PINS`, `RR_PINS` `invert` fields to `true` if a motor runs in the opposite physical direction for the logical forward command. The left/right signs must be checked with wheels elevated. NEVER change wiring with the battery on.

## Start and operate the phone dashboard

1. Put each car on a stable stand, test the physical motor-power cutoff, apply power, and give devices time to discover peers. Start with M1, M2 and one slave at a time, then build up to the full system.
2. On your phone connect to the Wi-Fi SSID **`SwarmLab-Control`** (password from M1's `RobotConfig.h`). The phone can show **No Internet**: this is expected; remain connected.
3. In a browser open **http://192.168.4.1** (not an HTTPS URL).
4. Select **Both groups**, **M1 group**, or **M2 group**. The dashboard lists M1, S1–S3, M2, S4–S6, online indicators, faults and sensor capability.
5. Use the direction arrows on the page. Hold to move; release to stop. Start with a low power slider position; test each direction carefully. **STOP BOTH GROUPS** is the software stop; the physical motor cutoff is the backup.
6. Follow/Mirror/Spin/Wave are **PWM or RPM-demand patterns**, not geometric formations. They coordinate issued motor commands; without inter-robot localization, physical spacing inevitably drifts.
7. If every selected robot has **two working rear encoders**, the Drive cm / Turn degrees buttons become enabled. Calibrate FIRST; see next section. If the target group includes a non-encoder car, the UI disables those buttons and the server also rejects the goal.
8. Watch status rows: `rear × 2` means measured rear encoder counts/RPM are available; `none` means the car cannot measure distance. An online state means recent telemetry, not guaranteed motor motion or successful radio delivery of every packet.

### Calibrate the rear-encoder version **per car**

Your motor's actual pulses per wheel revolution, wheel diameter and effective turn track are not known. `RobotConfig.h` contains **starting placeholders**, not certified values:

```cpp
static constexpr float ENCODER_TICKS_PER_WHEEL_REV = 330.0f;
static constexpr float WHEEL_DIAMETER_MM = 65.0f;
static constexpr float EFFECTIVE_TRACK_WIDTH_MM = 165.0f;
```

1. With motor **power physically disconnected**, keep the ESP32 and encoders supplied. View the target car's dashboard row and note **RL/RR counts**.
2. Turn the **rear-left wheel exactly 10 complete wheel revolutions** by hand (not just the motor shaft). Subtract the initial RL count from the final RL count, divide by **10**. Repeat for RR. This firmware counts **rising edges of A only**. If they differ substantially, verify encoder wiring and reduction ratios; this shared single calibration constant assumes roughly equivalent left/right motors.
3. Measure the actual wheel diameter or (better) loaded distance rolled by 10 wheel revolutions; replace `WHEEL_DIAMETER_MM` with the equivalent rolling diameter.
4. Verify the front and rear motors on each side physically rotate in the same forward direction, and check rear pulses actually increase while powered. If a rear counter stays zero or changes erratically, **do not drive on the ground**.
5. With wheels on stands, drive slowly and inspect measured RL/RR RPM. Adjust PI values (`PID_KP`, `PID_KI`) and feedforward (`PWM_STATIC`, `PWM_PER_RPM`) conservatively for your actual motors, supply and gearing. Never assume example values are tuned.
6. On a large clear level floor, test **Drive 20 cm**, measure distance with a tape and adjust calibration. Then test **Turn 45°**; a turn command uses an approximate track width, so adjust `EFFECTIVE_TRACK_WIDTH_MM` based on measured result and retest both directions. Keep a safe distance between cars.
7. Reflash the changed config to that car, then repeat across cars. **Two rear encoders do not detect slip on unencoded front wheels or car-to-car distances.**

## Physical acceptance checklist (perform before a group run)

- [ ] Both motor drivers stay disabled while ESP32 boots or resets; external enable pulldowns and physical cutoff work.
- [ ] Each wheel rotates in the correct logical forward/reverse direction with the vehicle raised; driver/motors do not overheat.
- [ ] Each car shows the correct role/group/ID/firmware variant in serial output; exactly eight unique identities exist.
- [ ] Phone loads dashboard, each M1/M2/S1–S6 eventually appears online, and status ages refresh.
- [ ] M1 responds to group 1 only; M2 responds to group 2 only; either group can be selected independently; STOP BOTH stops both.
- [ ] Hold/release arrow: release stops motion; turn off phone Wi-Fi while driving, verify lost-link stop on EACH hop and both groups (nominal ~550 ms local watchdog, actual latency to be measured).
- [ ] Reset M1 while rolling on a stand: each moving node stops when its command watchdog expires; it cannot be trusted as a hardware safety system.
- [ ] Rear variant: only RL/RR counters rise; front counters remain zero, RPM telemetry is measured on rear only; check wheel-diameter/pulse values.
- [ ] Rear variant: disconnect one encoder signal during a raised-wheel test; moving car detects no pulses, latches error and stops; fix and reboot safely.
- [ ] No-encoder variant: no fake RPM/distance appears; group distance commands are rejected. Mixed groups continue supporting manual mode.
- [ ] Send same goal again via repeated control frames; it must not restart travel; intentional STOP cancels an active goal.
- [ ] With wheels on the ground, first test a single car, then one four-car group, then all eight **only after all individual tests pass**.

## Troubleshooting

| Problem | Likely issue / next action |
|---|---|
| Sketch cannot compile on ESP32 | Check **Espressif 3.3.x** core (not 2.x/4.x), classic ESP32 board, and all three files in the sketch folder. Run real `arduino-cli` checks if available. |
| `SwarmProtocol.h` missing | You opened/copied only `.ino`. Unzip the complete `SwarmRobot` sketch folder. |
| ESP32 not found as a serial port | Use a data USB cable, install CP210x/CH340 driver appropriate to board, and check port permissions. |
| Phone cannot see SSID | Did you flash **M1 ROOT** (group 1, ID 0)? Confirm Serial Monitor and AP password length ≥8. |
| M2/slaves offline | All V4, same channel 6, correct ID/role, sufficient shared ground/clean power, close physical distance, check 115200 Serial Monitor. |
| Robot moves backward for Forward | Invert the corresponding motor in `RobotConfig.h` (or reverse its two motor leads with power removed). |
| Motors start at reboot | ENA/ENB jumpers still installed or EN inputs floating. Add safe pulldowns and physical motor disconnect. |
| Motor won't start or driver overheats | Check supply, battery sag, wiring, stall current and driver capacity. L298N can be unsuitable for high-current motors. |
| Encoder always zero / wrong RPM | Verify only rear-wheel A outputs connect to GPIO34/35 at 3.3-V logic, proper supply, common GND, needed pull-ups, wheel vs motor shaft revolution count. |
| Distance / turn buttons greyed out | At least one selected car is offline, lacks rear encoders, or reports a fault. This is deliberate. |
| Turn angle drifts | Calibrate effective track width on the same floor, decrease RPM, account for skid steering and front-wheel slip. |
| Latched fault after radio loss/stall | Stop and physically inspect, then reboot the specific car safely. Do not bypass fault handling to make it move. |

## Tests, limitations, and security

Run `bash tests/run_checks.sh` on a PC with **g++, Python 3 and Node.js** for protocol math, two variant controller simulations, host **stub** syntax, and embedded dashboard JavaScript. The repo also includes [GitHub Actions CI](.github/workflows/arduino.yml) designed to install **Arduino-ESP32 3.3.12** and **actually compile both sketches** once published; **the workflow itself is not yet run/verified**.

There is no encryption or command authentication on ESP-NOW, and discovery is based on broadcast identity/MAC. The phone AP is password protected but HTTP has no separate authentication. This is **lab prototype only**; not suitable in public untrusted radio environments or near people without additional controls. The firmware is not a safety-certified emergency-stop system. No remote localization, coordinated collision avoidance, obstacle detection, absolute map, motion capture, synchronized start time, or proof of movement from a non-encoder car. A physical cutoff and supervised testing are mandatory.

Read [AUDIT.md](AUDIT.md) for precise evidence and unverified items, and [PUBLISH_TO_GITHUB.md](PUBLISH_TO_GITHUB.md) for getting this into a dedicated GitHub repository.
