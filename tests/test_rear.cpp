#include "../firmware/Rear_Encoders/SwarmRobot/SwarmRobot.ino"
#include <cassert>
#include <iostream>
static void tick(uint32_t left=0, uint32_t right=0) {
  fakeMillis+=60;
  encoderPulses[2]+=left; encoderPulses[3]+=right;
  updateMotorController(fakeMillis);
}
int main() {
  assert(SWARM_REAR_ENCODERS==1);
  assert(initMotors()); initEncoders(); fakeMillis=20000;
  commandLocal(MANUAL,{35,35},0,0,0,0,0,fakeMillis);
  tick(); assert(motion.state==RUNNING);
  assert(motors[0].signedPwm==motors[2].signedPwm);
  assert(motors[1].signedPwm==motors[3].signedPwm);
  for(int i=0;i<5;i++)tick(3,3);
  assert(wheelOdomTicks[2]==15 && wheelOdomTicks[3]==15);
  assert(wheelOdomTicks[0]==0 && wheelOdomTicks[1]==0);
  assert(actualRpm[2]>0 && actualRpm[3]>0 && actualRpm[0]==0);
  Packet p={};copyTelemetryToPacket(p);
  assert(p.flags & REAR_ENCODER_FLAG);
  assert(p.pulses[0]==0 && p.pulses[1]==0);
  MotorTelemetry t; telemetryFromPacket(t,p);assert(t.rearEncoders);
  Packet relay={};packetFromTelemetry(relay,t);assert(relay.flags & REAR_ENCODER_FLAG);
  assert(!selectedRobotsHaveRearEncoders(1));
  for(int i=1;i<=3;i++)ownSlaves[i].telemetry.rearEncoders=true;
  assert(selectedRobotsHaveRearEncoders(1));
  assert(!selectedRobotsHaveRearEncoders(2));
  otherMaster.telemetry.rearEncoders=true;
  for(int i=1;i<=3;i++)group2Slaves[i].telemetry.rearEncoders=true;
  assert(selectedRobotsHaveRearEncoders(0));
  commandLocal(STOP,{0,0},0,0,0,0,0,fakeMillis);
  commandLocal(DRIVE_MM,{0,0},999,101,100,0,40,fakeMillis);
  assert(motion.state==RUNNING);
  int32_t start=motion.startTicks[2], aim=motion.targetDelta[2];
  assert(aim>100);
  tick();
  commandLocal(DRIVE_MM,{0,0},999,101,100,0,40,fakeMillis);
  assert(motion.startTicks[2]==start);
  while (motion.state==RUNNING && wheelOdomTicks[2]-start<aim) {
    int32_t n=aim-(wheelOdomTicks[2]-start);
    tick(n>8?8:n,n>8?8:n);
  }
  assert(motion.state==DONE && motion.progress==100);
  for(auto&m:motors)assert(m.signedPwm==0);
  commandLocal(DRIVE_MM,{0,0},999,101,100,0,40,fakeMillis);
  assert(motion.state==DONE); // repeated command must not restart
  commandLocal(DRIVE_MM,{0,0},999,102,100,0,40,fakeMillis);
  tick();
  for(int i=0;i<40&&motion.state==RUNNING;i++)tick();
  assert(motion.fault==ENCODER_STALL && motion.state==ERROR);
  for(auto&m:motors)assert(m.signedPwm==0);
  commandLocal(MANUAL,{30,30},0,0,0,0,0,fakeMillis);
  assert(motion.state==ERROR);
  std::cout<<"PASS two-rear-encoder PI, front matching, telemetry, capability gating, goal dedupe, stall latch\n";
}
