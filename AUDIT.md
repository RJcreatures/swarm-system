# V4 software audit — 24 November 2026

## Scope

Baseline: prior V3 dual-master four-encoder sketch provided in conversation ZIP. Target: two code variants using same packet protocol: (A) no encoder, (B) encoder A input on rear-left and rear-right only. Both maintain 2 masters / 6 slaves, M1 phone dashboard, ESP-NOW commands and relayed feedback. The baseline V3 was **not** compatible with the new hardware without changes.

## Implemented changes

- Replaced four-channel encoder speed control with **two rear encoder input channels** in variant B; pairs front wheel PWM with its matching rear wheel. Front wheel speed is **not independently controlled/measured**.
- Provided true **open-loop** fallback in variant A, with no fake RPM or odometry.
- Added per-node encoder-capability flag through master/status/relay telemetry; source and browser reject autonomous distance/turn goals for any group missing encoder-equipped nodes.
- Added explicit stop + latched error for goal packets erroneously sent to a no-encoder vehicle.
- Changed encoder wiring to rear-left GPIO34 and rear-right GPIO35, avoiding V3's front and rear 4-channel assumptions.
- Bumped over-the-air protocol to **V4** (all eight must be reflashed), maintained frame IDs and duplicate-goal protection.
- Maintained link watchdogs, stale-sequence guards, bounded queues and lost-encoder stall fault.

## Evidence matrix

| Check | Result | Evidence method |
|---|---|---|
| V4 packet validation, stale sequence, signed motion, pattern math | PASS | `tests/test_protocol.cpp` C++ host tests |
| No-encoder four-motor drive, reverse/stop, no artificial sensor values, goal rejection | PASS | `tests/test_normal.cpp` host stub simulation |
| Rear-encoder front/rear side PWM, rear-only pulses and feedback | PASS | `tests/test_rear.cpp` host stub simulation |
| Mission duplicate, completion, missing encoder stall, fault latch | PASS | `tests/test_rear.cpp` host stub simulation |
| Group-level encoder capability checks for selected groups | PASS | `tests/test_rear.cpp` host stub simulation |
| Both Arduino sketch sources C++ syntax | PASS *under host test stubs only* | `g++ -fsyntax-only` with `tests/stubs/` |
| Slave and M2 command-loss watchdogs, stop + latched error | PASS | `tests/test_slave_watchdog.cpp`, `tests/test_secondary_watchdog.cpp` |
| Both variants with all eight role/ID configurations (16 host source builds) | PASS *under host stubs only* | `tests/run_checks.sh` |
| Dashboard JavaScript syntax and required routes | PASS | `node --check` via `tests/check_web_script.py` |
| Real Arduino-ESP32 3.3.12 compile | **NOT RUN** | Board toolchain not installed; cannot fetch toolchain in local sandbox |
| ESP-NOW discovery/packet timing with 8 ESP32s | **NOT RUN** | No physical radio boards accessible |
| L298N pin assignments and motor speed/current/heat | **NOT RUN** | Need exact motors/driver modules and bench equipment |
| Rear-encoder electrical voltage, pulses/rev, PI tuning and distance accuracy | **NOT RUN** | Need actual encoder model and physical measurements |
| Watchdog stop timing, radio loss, physical e-stop verification | **NOT RUN** | Mandatory physical acceptance tests |
| GitHub Actions compile | **NOT RUN** | Requires publishing repository; do not report green until CI runs |

**Verdict:** Software-level prototype passes offline tests, **hardware acceptance remains blocked**. Do not claim "fully works" or accuracy of centimeters/degrees until both Arduino builds compile on the target toolchain and physical acceptance passes.

## Hardware/design risks explicitly carried

- L298N is inefficient; each channel's stall current/heat and supply noise must be measured. Two L298N boards per car are required for individual four-motor channels with the wiring supplied.
- Manual mode for a non-encoder car is open-loop; telemetry proves command delivery/application only, never physical displacement.
- A-only encoders infer direction from PWM. Brake/coasting/reverse/pushing/slip compromise odometry. PI controls rear measurements, not front wheel motion.
- Only 2 of 4 motors sensed; uneven front/rear loads can drift. Inter-robot distance and avoidance are not measured.
- ESP-NOW packets are unencrypted and discovered through unauthenticated hello messages. Harden MAC allowlists/keyed messaging before unsupervised demos.
- MCU reset/hang and motor-driver hardware faults may defeat software stop. Require external motor-power cutoff, EN pulldowns, fuse and supervision.
- Browser 'STOP' is not a safety-certified E-stop; a missed frame relies on autonomous watchdog expiration.

## Local reproducibility

```bash
bash tests/run_checks.sh
```

The checked sketches are `firmware/No_Encoders/SwarmRobot` and `firmware/Rear_Encoders/SwarmRobot`. The Arduino toolchain is separate from the host stub tests.
