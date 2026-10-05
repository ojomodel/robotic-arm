#include "../BridgeState.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>

int checks = 0;
void check(bool pass, const char* label) { ++checks; if (!pass) throw std::runtime_error(label); }
bool near(float a, float b, float tolerance = .0003f) { return fabsf(a-b) <= tolerance; }
std::string send(cf1::BridgeState& state, const std::string& line, uint32_t now) {
  char result[224]; state.command(line.c_str(), now, result, sizeof(result)); return result;
}
std::string pose(uint32_t seq, const float* q) {
  std::ostringstream line; line << "POSE 17 " << seq << std::fixed << std::setprecision(5);
  for (size_t i=0;i<5;++i) line << ' ' << q[i];
  return line.str();
}
cf1::BridgeState fresh(float maximum=60, uint32_t now=0) {
  cf1::BridgeState s; float home[5]={90,90,90,90,90}; s.begin(17,home,now,true);
  std::ostringstream limits; limits << "LIMITS 17 0 180 0 180 0 180 0 180 0 180 " << maximum;
  check(send(s,limits.str(),now)=="ACK LIMITS 0","limits accepted");
  check(send(s,"ARM 17",now)=="ACK ARM 0","explicit arm accepted"); s.takeDirtyMask(); return s;
}
void identityAndBounds() {
  auto s=fresh(); auto before=s;
  check(send(s,"MOTION",10)=="MOTION CF1 400 40","read-only capability text");
  check(!memcmp(&s,&before,sizeof(s)),"capability query cannot alter even trajectory/watchdog state");
  send(s,"HOLD 17",10);
  check(send(s,"LIMITS 17 0 180 0 180 0 180 0 180 0 180 100",10)=="ACK LIMITS 0" && s.speed==100,"100deg/s accepted without arming");
  check(send(s,"LIMITS 17 0 180 0 180 0 180 0 180 0 180 60",10)=="ACK LIMITS 0","60deg/s accepted");
  before=s;
  check(send(s,"LIMITS 17 0 180 0 180 0 180 0 180 0 180 400.001",10)=="ERR RANGE"&&!memcmp(&s,&before,sizeof(s)),"above400 rejects atomically");
  send(s,"ARM 17",10); float endpoints[5]={0,180,45,135,90}; send(s,pose(1,endpoints),10);
  for(uint32_t now=10;now<=1600;now+=10) {
    send(s,"PING 17",now); float previous[5]; memcpy(previous,s.current,sizeof(previous)); s.tick(now);
    for(size_t i=0;i<5;i++) {
      check(isfinite(s.current[i])&&s.current[i]>=0&&s.current[i]<=180,"finite closed servo bounds");
      check(fabsf(s.current[i]-previous[i])<=.6001f,"per-joint rate capped60");
    }
  }
  for(size_t i=0;i<5;i++)check(s.current[i]==endpoints[i],"exact arrival at endpoint without wrapping");
  auto slow=fresh(1e-37f);send(slow,pose(1,endpoints),0);slow.tick(50);
  check(isfinite(slow.segmentDurationMs)&&slow.segmentDurationMs>1e40,"very small finite accepted speed does not overflow trajectory duration");
  for(size_t i=0;i<5;i++)check(isfinite(slow.current[i]),"tiny finite speed never emits NaN or infinity");
}
void synchronizationAndRetarget() {
  auto s=fresh(30); float target[5]={91,92,94,89,90}; send(s,pose(1,target),0);
  check(near(static_cast<float>(s.segmentDurationMs),4.f/30.f*1000.f),"duration follows longest joint delta");
  for(uint32_t time=10;time<=130;time+=10) {
    s.tick(time); float fraction=(s.current[2]-90)/4;
    check(near(s.current[0],90+fraction)&&near(s.current[1],90+2*fraction)&&near(s.current[3],90-fraction),"uneven signed deltas share a common segment fraction");
    check(s.current[4]==90,"stationary joint remains stationary");
  }
  s.tick(140); for(size_t i=0;i<5;i++)check(s.current[i]==target[i],"all moving joints finish together");
  const auto finished=s; send(s,pose(2,target),140);
  check(!s.segmentActive&&s.segmentElapsedMs==finished.segmentElapsedMs,"duplicate arrived target cannot restart motion");
  float small[5]={91.12f,92.06f,94,89,90}; send(s,pose(3,small),140);
  check(s.segmentDurationMs==40,"short packet uses minimum40ms horizon"); s.tick(160);
  check(near(s.current[0],91.06f)&&near(s.current[1],92.03f),"short packet spreads across40ms instead of sprint then stop");
  const double elapsed=s.segmentElapsedMs; send(s,pose(4,small),160);
  check(s.segmentElapsedMs==elapsed,"duplicate in-flight target preserves progress"); s.tick(180);
  check(s.current[0]==small[0]&&!s.segmentActive,"duplicate refresh still reaches exact target");
  float ahead[5]={110,100,100,100,90}; send(s,pose(5,ahead),180); s.tick(200);
  float atRetarget[5];memcpy(atRetarget,s.current,sizeof(atRetarget));
  float reverse[5]={70,75,80,85,90};send(s,pose(6,reverse),200);
  for(size_t i=0;i<5;i++)check(s.current[i]==atRetarget[i]&&s.segmentStart[i]==atRetarget[i],"retarget preserves exact current position and replans from it");
  for(uint32_t time=201;time<1900;time++) {
    if(time%100==0)send(s,"PING 17",time);
    float prior=s.current[0];s.tick(time);
    check(s.current[0]<=prior+.00001f&&s.current[0]>=reverse[0],"reversal is monotonic toward new goal without overshoot");
    check(prior-s.current[0]<=.03002f,"retarget retains30deg/s cap");
  }
}
struct StreamStats { int moving=0, legacyMoving=0; float maximumStep=0, maximumLegacyStep=0; };
StreamStats stream(uint32_t interval, float configuredSpeed, float requestedRate, bool irregular=false) {
  auto s=fresh(configuredSpeed);float goal[5]={90,90,90,90,90};
  float legacy[5]={90,90,90,90,90};uint32_t next=0,seq=0;StreamStats result;
  for(uint32_t time=0;time<=2000;time++) {
    if(time==next) {
      goal[0]=90+requestedRate*time*.001f;goal[1]=90-requestedRate*.5f*time*.001f;
      check(send(s,pose(++seq,goal),time).rfind("ACK POSE",0)==0,"stream pose accepted");
      next+=irregular?((seq%2)==0?33U:17U):interval;
    }
    const float before=s.current[0],old=legacy[0];s.tick(time);
    for(size_t i=0;i<5;i++) {
      const float distance=goal[i]-legacy[i],step=configuredSpeed*.001f;
      legacy[i]+=fabsf(distance)<=step?distance:(distance>0?step:-step);
    }
    if(time>200) {
      float step=fabsf(s.current[0]-before),oldStep=fabsf(legacy[0]-old);
      result.moving+=step>.00002f;result.legacyMoving+=oldStep>.00002f;
      if(step>result.maximumStep)result.maximumStep=step;
      if(oldStep>result.maximumLegacyStep)result.maximumLegacyStep=oldStep;
    }
    check(isfinite(s.current[0])&&s.current[0]<=goal[0]+.0001f,"stream never exceeds commanded ramp");
  }
  check(s.current[0]>90+requestedRate*1.90f,"stream has bounded lag rather than restarting into a freeze");
  return result;
}
void cadenceEvidence() {
  const auto oldCadence=stream(50,30,3);
  const auto newCadence=stream(20,30,3);
  const auto jitter=stream(20,60,3,true);
  check(oldCadence.moving>oldCadence.legacyMoving*5,"40ms interpolation reduces short-target dwell even at prior20Hz host cadence");
  check(newCadence.moving>1700&&newCadence.moving>newCadence.legacyMoving*6,"50Hz streaming keeps small motion continuous instead of many stopped milliseconds");
  check(newCadence.maximumStep<newCadence.maximumLegacyStep*.25f,"small commanded motion no longer chases every packet at full configured speed");
  check(jitter.moving>1700&&jitter.maximumStep<.01f,"40ms horizon bridges normal17/33ms host render cadence without large small-motion steps");
  std::cout<<"Cadence evidence moving milliseconds/1800:20Hz "<<oldCadence.moving<<" vslegacy "<<oldCadence.legacyMoving
    <<";50Hz "<<newCadence.moving<<" vslegacy "<<newCadence.legacyMoving<<";jitter "<<jitter.moving
    <<".50Hz peak step="<<newCadence.maximumStep<<" vslegacy="<<newCadence.maximumLegacyStep<<" deg/ms\n";
}
void stopsAndStalls() {
  auto s=fresh();float target[5]={150,130,110,70,30};send(s,pose(1,target),0);s.tick(10);
  const auto before=s;s.tick(410);
  check(near(s.current[0]-before.current[0],3),"loop stall advances at most50ms of60deg/s trajectory");
  const auto after=s;s.tick(411);check(near(s.current[0]-after.current[0],.06f),"discarded stall time never catches up later");
  float held[5];memcpy(held,s.current,sizeof(held));s.tick(500);
  check(!s.armed&&!s.segmentActive&&!s.poseMode,"watchdog clears trajectory before movement");
  for(size_t i=0;i<5;i++)check(s.current[i]==held[i]&&s.target[i]==held[i],"watchdog freezes exact command");
  send(s,"ARM 17",600);send(s,pose(2,target),600);s.tick(610);send(s,"HOLD 17",610);
  memcpy(held,s.current,sizeof(held));s.tick(650);send(s,"PING 17",650);
  check(!s.armed&&s.outputsEnabled&&!s.segmentActive,"HOLD retains PWM and clears segment");
  for(size_t i=0;i<5;i++)check(s.current[i]==held[i],"HOLD does not coast");
  send(s,"ARM 17",650);send(s,pose(3,target),650);s.tick(660);send(s,"RELEASE 17",660);
  check(!s.outputsEnabled&&!s.armed&&!s.segmentActive&&!s.dirtyMask&&!s.limitsSet&&s.releaseRequested,"RELEASE clears segment, dirty writes and limits immediately");
  s=fresh(60,UINT32_MAX-30);send(s,pose(1,target),UINT32_MAX-30);s.tick(9);
  check(near(s.current[0],92.4f)&&s.armed,"trajectory elapsed safely crosses millis wrap");
  s.tick(469);check(!s.armed&&!s.segmentActive,"watchdog still expires500ms across wrap");
  s=fresh();send(s,"JOG 17 1 0 1",0);s.tick(50);
  check(!s.poseMode&&near(s.current[0],90.15f)&&s.current[1]==90,"JOG preserves original independent3deg/s behavior");
  auto beforeInvalid=s;check(send(s,"POSE 17 2 100 100 100 100 181",50)=="ERR RANGE"&&!memcmp(&s,&beforeInvalid,sizeof(s)),"invalid last joint cannot mutate interpolation state");
}
int main() {
  try { identityAndBounds();synchronizationAndRetarget();cadenceEvidence();stopsAndStalls();
    std::cout<<"PASS "<<checks<<" synchronized motion assertions; simulated parser/command positions only, no hardware\n";return 0;
  }catch(const std::exception&e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}
}
