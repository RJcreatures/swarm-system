#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p tests/.build
trap 'rm -rf tests/.build' EXIT
common=( -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -pedantic )
g++ "${common[@]}" tests/test_protocol.cpp -o tests/.build/protocol
tests/.build/protocol
for variant in No_Encoders Rear_Encoders; do
  g++ "${common[@]}" -fsyntax-only -Itests/stubs -I"firmware/$variant/SwarmRobot" \
    -x c++ "firmware/$variant/SwarmRobot/SwarmRobot.ino"
  echo "PASS $variant host-stub syntax"
done
for name in normal rear slave_watchdog secondary_watchdog; do
  g++ "${common[@]}" -Itests/stubs tests/test_$name.cpp -o tests/.build/$name
  tests/.build/$name
done
# Compile all eight hardware identity configurations with both variant modes.
roles=( 'ROOT 1 0' 'SECONDARY 2 0' 'SLAVE 1 1' 'SLAVE 1 2' 'SLAVE 1 3' 'SLAVE 2 1' 'SLAVE 2 2' 'SLAVE 2 3' )
for variant in No_Encoders Rear_Encoders; do
  for spec in "${roles[@]}"; do
    read -r role grp ident <<< "$spec"
    g++ "${common[@]}" -fsyntax-only -Itests/stubs -I"firmware/$variant/SwarmRobot" \
      -DNODE_ROLE="Swarm::$role" -DNODE_GROUP="$grp" -DNODE_ID="$ident" \
      -x c++ "firmware/$variant/SwarmRobot/SwarmRobot.ino"
  done
  echo "PASS $variant: all 8 identity builds against host stubs"
done
cmp firmware/No_Encoders/SwarmRobot/SwarmProtocol.h firmware/Rear_Encoders/SwarmRobot/SwarmProtocol.h
python3 tests/check_web_script.py
echo 'PASS all offline checks (NOT a hardware/Arduino-core compilation)'
