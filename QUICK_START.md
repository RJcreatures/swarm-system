# First 20 minutes: bring-up checklist

**Start with one ESP32 car and its wheels lifted. Motor power must have a separate mechanical OFF switch.**

1. Identify hardware: classic ESP32-WROOM DevKit, four motors, **two L298N boards** (one for front, one for rear). Each driver has two independent channels.
2. Choose firmware: `firmware/No_Encoders/SwarmRobot/` for no encoders, or `firmware/Rear_Encoders/SwarmRobot/` for a rear-left + rear-right A-only encoder car.
3. Install Arduino IDE 2 and `esp32` by Espressif **version 3.3.x** via Boards Manager. Additional URL:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
4. Open chosen `SwarmRobot.ino`, which must be in the same directory as `RobotConfig.h` and `SwarmProtocol.h`.
5. In `RobotConfig.h` set `NODE_ROLE`, `NODE_GROUP`, `NODE_ID` according to the table in README. M1=ROOT/1/0; M2=SECONDARY/2/0; S1..3=SLAVE/1/1..3; S4..6=SLAVE/2/1..3.
6. Set M1's `PHONE_AP_PASSWORD` to a unique string >=8 characters. Verify **ESP32 Dev Module**, selected COM/tty port, compile and upload. Monitor Serial at 115200.
7. Attach motor drivers using `README.md` wiring map. Remove enable jumpers; add EN pulldowns; provide proper motor battery and regulator and **common GND**. Do not feed 5V encoder signals to ESP32.
8. Variant B only: rear-left encoder `A` -> **GPIO34** and rear-right `A` -> **GPIO35**, with appropriate 3.3-V logic and pull-ups if required. No front encoder is connected.
9. With wheels raised, verify forward and reverse. Fix `invert` in config per motor. Test physical power cutoff.
10. Program all eight unique IDs. Open phone Wi-Fi `SwarmLab-Control`, visit **http://192.168.4.1**. Check all eight online and observe telemetry; control **group 1** and **group 2** separately first.
11. Encoder cars only: calibrate rear pulses per wheel revolution and wheel diameter (do not use placeholder values), then try 20 cm on clear level floor. An unencoded car must never receive a distance goal.
12. Test disconnect of phone, M1->M2 radio, and master->slave links on a wheel stand. Verify commanded movement stops and faults are reported. Only then try multi-car ground tests.

**If compilation fails:** Actual Arduino-ESP32 core compilation is not yet verified in this environment. Use the pinned `3.3.12` build specified in GitHub Actions and report the first compiler error. Never equate host simulation with successful upload.

See `README.md` for exhaustive wiring, calibration, troubleshooting, and safety checks. See `AUDIT.md` for the audit status.
