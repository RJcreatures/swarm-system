#define NODE_ROLE Swarm::SECONDARY
#define NODE_GROUP 2
#define NODE_ID 0
#include "../firmware/Rear_Encoders/SwarmRobot/SwarmRobot.ino"
#include <cassert>
#include <iostream>
int main() {
  assert(initMotors());initEncoders();fakeMillis=20000;
  lastRootCommandAt=fakeMillis;
  commandLocal(MANUAL,{40,40},0,0,0,0,0,fakeMillis);
  updateMotorController(fakeMillis+60);
  assert(motion.state==RUNNING);
  fakeMillis+=MASTER_LINK_DEADMAN_MS+1;
  updateMotorsAndCommands(fakeMillis);
  assert(motion.state==ERROR && motion.fault==LINK_TIMEOUT);
  for(auto&m:motors)assert(m.signedPwm==0);
  std::cout<<"PASS secondary master command-loss watchdog stops and latches fault\n";
}
