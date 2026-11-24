#include "../firmware/No_Encoders/SwarmRobot/SwarmRobot.ino"
#include <cassert>
#include <iostream>
static void tick(){fakeMillis+=60;updateMotorController(fakeMillis);}
int main() {
  assert(SWARM_REAR_ENCODERS==0);
  assert(initMotors());initEncoders();fakeMillis=20000;
  commandLocal(MANUAL,{50,25},0,0,0,0,0,fakeMillis);
  tick();
  assert(motors[0].signedPwm>0 && motors[1].signedPwm>0);
  assert(motors[0].signedPwm==motors[2].signedPwm);
  assert(motors[1].signedPwm==motors[3].signedPwm);
  assert(motors[0].signedPwm>motors[1].signedPwm);
  Packet p={};copyTelemetryToPacket(p);
  assert(!(p.flags & REAR_ENCODER_FLAG));
  assert(p.odomMm==0 && p.pulses[0]==0 && p.pulses[2]==0);
  assert(!selectedRobotsHaveRearEncoders(1));
  commandLocal(STOP,{0,0},0,0,0,0,0,fakeMillis);tick();
  for(auto&m:motors)assert(m.signedPwm==0);
  commandLocal(MANUAL,{-50,0},0,0,0,0,0,fakeMillis);tick();
  assert(motors[0].signedPwm<0 && motors[1].signedPwm==0);
  commandLocal(DRIVE_MM,{0,0},900,21,100,0,35,fakeMillis);
  assert(motion.state==ERROR && motion.fault==ENCODER_REQUIRED);
  for(auto&m:motors)assert(m.signedPwm==0);
  commandLocal(MANUAL,{50,50},0,0,0,0,0,fakeMillis);tick();
  for(auto&m:motors)assert(m.signedPwm==0);
  std::cout<<"PASS normal four-wheel open-loop, stop, reverse, no fake telemetry, reject autonomous goal\n";
}
