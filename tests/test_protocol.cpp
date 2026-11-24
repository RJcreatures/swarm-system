#include "../firmware/Rear_Encoders/SwarmRobot/SwarmProtocol.h"
#include <cassert>
#include <iostream>
using namespace Swarm;
int main() {
  Packet p={}; p.magic=MAGIC; p.version=VERSION; p.type=GROUP_COMMAND;
  p.role=ROOT; p.group=1; p.target=0; p.mode=DRIVE_MM; p.distanceMm=500; p.speedRpm=35;
  assert(VERSION==4 && sizeof(Packet)<=250 && valid(p));
  p.version=3; assert(!valid(p)); p.version=VERSION;
  p.mode=9; assert(!valid(p)); p.mode=TURN_DEG;
  p.angleDeg=800; assert(!valid(p)); p.angleDeg=90;
  p.fault=5; assert(!valid(p)); p.fault=NO_FAULT;
  p.type=0; assert(!valid(p)); p.type=GROUP_COMMAND;
  assert(!acceptsCommand(4,2,4,2));
  assert(acceptsCommand(4,3,4,2));
  assert(acceptsCommand(5,1,4,2));
  assert(!acceptsCommand(0,100,4,2));
  assert(!newSequence(1,2));
  auto d=motorsFor(FOLLOW,35,10,1,0);assert(d.left==45&&d.right==25);
  d=motorsFor(MIRROR,35,10,1,0); assert(d.left==25 && d.right==45);
  d=motorsFor(SPIN,35,0,1,0);assert(d.left==-35&&d.right==35);
  d=motorsFor(WAVE,35,0,1,0);assert(d.left==0&&d.right==0);
  d=motorsFor(WAVE,35,0,1,1);assert(d.left==35&&d.right==35);
  const float c=3.14159265358979323846f*65.f;
  assert(ticksForMm(c,65.f,330.f)==330);
  assert(targetTicks(DRIVE_MM,-205,0,2,65.f,330.f,165.f)<0);
  assert(targetTicks(TURN_DEG,0,90,2,65.f,330.f,165.f)>0);
  assert(targetTicks(TURN_DEG,0,90,3,65.f,330.f,165.f)<0);
  assert(targetTicks(TURN_DEG,0,-90,2,65.f,330.f,165.f)<0);
  std::cout<<"PASS V4 packet validation, sequence, four patterns, relative goals\n";
}
