#include "../BridgeState.h"
#include "../StartupHome.h"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <limits>

int checks = 0;
void check(bool condition, const char* message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
bool near(float a, float b) { return fabsf(a-b) < .0002f; }
std::string command(cf1::BridgeState& s, const std::string& line, uint32_t now = 10) {
  char result[224]; s.command(line.c_str(), now, result, sizeof(result)); return result;
}
void unchangedReject(cf1::BridgeState& s, const std::string& line, const char* expected = nullptr) {
  const cf1::BridgeState before = s;
  const std::string response = command(s, line, 333);
  check(response.rfind("ERR ", 0) == 0, "invalid command not rejected");
  if (expected) check(response == expected, "unexpected rejection code");
  check(memcmp(&before, &s, sizeof(s)) == 0, "invalid command mutated state");
}
cf1::BridgeState state(bool ready = true, uint32_t now = 0) {
  cf1::BridgeState s; s.begin(17, nullptr, now, ready); return s;
}
const char* limits = "LIMITS 17 70 110 70 110 80 130 40 90 140 180 30";

void independentStartupHome() {
  auto s = state();
  const auto before = s;
  check(!s.initializeStartupPose(nullptr), "null startup pose accepted");
  check(memcmp(&before, &s, sizeof(s)) == 0, "null startup pose changed state");
  for (const float invalid : {NAN, INFINITY, -INFINITY, -.001f, 180.001f}) {
    // Put the failure last, with all earlier values different, to detect a partial commit.
    float pose[5] = {80, 81, 82, 83, invalid};
    check(!s.initializeStartupPose(pose), "invalid startup pose accepted");
    check(memcmp(&before, &s, sizeof(s)) == 0, "invalid startup pose partially changed state");
  }
  check(s.initializeStartupPose(cf1::StartupHome), "valid operational home rejected");
  check(!s.armed && !s.outputsEnabled && !s.dirtyMask && !s.limitsSet && s.lastSeq == 0,
        "operational home initialization enabled or scheduled outputs");
  for (size_t i=0; i<5; ++i) {
    check(s.current[i] == cf1::StartupHome[i] && s.target[i] == cf1::StartupHome[i] &&
          s.segmentStart[i] == cf1::StartupHome[i], "operational home not initialized consistently");
    check(s.neutral[i] == cf1::Fallback[i], "operational home changed calibration anchor");
  }
  check(command(s,"HELLO") == "HELLO CF1 17 90.800 92.900 104.500 60.900 180.000",
        "operational home changed calibration HELLO");
  check(command(s,"STATUS") == "STATE 17 0 0 0 0 90.800 92.900 118.500 60.900 180.000",
        "startup STATUS did not expose the new resting commands");
  const auto initialized = s;
  check(!s.initializeStartupPose(cf1::Fallback) && memcmp(&initialized, &s, sizeof(s)) == 0,
        "startup pose can be silently initialized twice");

  command(s,limits); command(s,"ARM 17");
  const auto armed = s;
  check(!s.initializeStartupPose(cf1::Fallback) && memcmp(&armed, &s, sizeof(s)) == 0,
        "startup initializer changed armed state");
  check(s.current[2] == 118.5f && s.takeDirtyMask() == 31, "first ARM did not retain the new home");
  command(s,"POSE 17 1 91.8 92.9 118.5 60.9 180",10); s.tick(60);
  check(near(s.current[0],91.8f), "test pose did not reach its endpoint");
  command(s,"RELEASE 17",60); s.takeReleaseRequest();
  const auto released = s;
  check(!s.initializeStartupPose(cf1::StartupHome) && memcmp(&released, &s, sizeof(s)) == 0,
        "startup initializer reset position after RELEASE");
  command(s,"HELLO"); command(s,"STATUS"); command(s,"HOLD 17"); command(s,"ARM 17",70);
  check(near(s.current[0],91.8f) && near(s.target[0],91.8f) && s.current[2] == 118.5f,
        "reconnect or re-arm reset the held pose to startup home");

  s=state(false); const auto unavailable=s;
  check(!s.initializeStartupPose(cf1::StartupHome) && memcmp(&unavailable,&s,sizeof(s))==0,
        "startup home was accepted before outputs-off confirmation");
  s=state(); command(s,limits); const auto limited=s;
  check(!s.initializeStartupPose(cf1::StartupHome) && memcmp(&limited,&s,sizeof(s))==0,
        "startup home was accepted after configuring a session");
}

void startupAndParser() {
  auto s = state();
  check(!s.armed && !s.outputsEnabled && !s.limitsSet && !s.dirtyMask, "startup must be disarmed/off");
  check(s.lastSeq == 0 && s.boot == 17, "initial identity");
  for (size_t i=0;i<5;i++) check(s.current[i] == cf1::Fallback[i] && s.target[i] == cf1::Fallback[i], "neutral loaded");
  check(command(s,"HELLO") == "HELLO CF1 17 90.800 92.900 104.500 60.900 180.000", "hello format");
  check(command(s,"STATUS") == "STATE 17 0 0 0 0 90.800 92.900 104.500 60.900 180.000", "state format");
  check(!s.outputsEnabled, "read-only handshake enabled outputs");
  check(command(s,"MOTION") == "MOTION CF1 400 40" && !s.armed && !s.outputsEnabled && !s.limitsSet, "read-only motion capability");
  unchangedReject(s,"MOTION extra","ERR COMMAND");
  for (const auto& bad : {"", " ", "HELLO x", "STATUS 17", "ARM", "ARM 17 extra", "ARM -17", "ARM +17", "ARM 17.0", "ARM 0x11", "ARM 4294967296", "ARM 17x", "ARM 18", "arm 17", "POSE 17 1 1 2 3 4", "ARM 17\x01"}) unchangedReject(s,bad);
  unchangedReject(s, std::string(193,'x'), "ERR LINE");
  unchangedReject(s,"POSE 17 1 91 93 105 61 179","ERR DISARMED");
  check(command(s,"PING 17").empty() && !s.armed && s.lastValidMs == 0, "disarmed ping must not arm/refresh");
  check(command(s,"\tARM 17\r ",100) == "ACK ARM 0", "whitespace/CRLF support");
  check(s.armed && s.outputsEnabled && s.takeDirtyMask() == 31, "arm enables exactly five outputs");
  check(s.lastValidMs == 100, "arm watchdog refresh");
  unchangedReject(s,"POSE 17 1 91 93 105 61 179","ERR LIMITS");
  check(command(s,"HOLD 17") == "ACK HOLD 0" && !s.armed && s.outputsEnabled, "hold disarms while pulses remain");

  float saved[5] = {NAN,-1,181,45,120}; s.begin(0,saved,30,true);
  check(s.boot != 0 && s.current[0] == cf1::Fallback[0] && s.current[1] == cf1::Fallback[1] && s.current[2] == cf1::Fallback[2], "fallback bad NVS");
  check(s.current[3] == 45 && s.current[4] == 120 && !s.outputsEnabled, "valid NVS preserved without enabling");

  uint32_t u = 99; float f=99;
  check(cf1::unsignedDecimal("4294967295",u) && u == UINT32_MAX, "max sequence parsing");
  check(!cf1::unsignedDecimal("4294967296",u), "unsigned overflow");
  for (const auto& bad : {"nan","NaN","inf","-inf","Infinity","1x","0x1p0",".","+","1e","1e+","1e100","1e-100","1,2"}) check(!cf1::finiteDecimal(bad,f), "bad float accepted");
  check(cf1::finiteDecimal("+1.25e1",f) && f == 12.5f, "valid exponent");
}

void atomicAndLimits() {
  auto s = state();
  unchangedReject(s,"LIMITS 17 70 110 70 110 80 130 40 90 140 179 30","ERR RANGE");
  unchangedReject(s,"LIMITS 17 70 110 70 110 80 130 40 90 140 180 400.01","ERR RANGE");
  unchangedReject(s,"LIMITS 17 70 110 70 110 80 130 40 90 140 180 0","ERR RANGE");
  unchangedReject(s,"LIMITS 17 70 110 70 110 80 130 40 90 140 180 NaN","ERR NUMBER");
  check(command(s,limits) == "ACK LIMITS 0" && s.limitsSet && s.speed == 30, "valid limits");
  command(s,"ARM 17"); s.takeDirtyMask();
  unchangedReject(s,limits,"ERR ARMED");
  for (const auto& bad : {"POSE 17 1 91 94 106 61 181", "POSE 17 1 91 94 106 61 nan", "POSE 17 1 91 94 106 61 180x", "POSE 17 -1 91 94 106 61 180", "POSE 17 0 91 94 106 61 180", "POSE 18 1 91 94 106 61 180", "POSE 17 1 91 94 106 61 180 extra"}) unchangedReject(s,bad);
  check(command(s,"POSE 17 7 100 100 120 70 160",20) == "ACK POSE 7", "pose acknowledgment");
  check(s.lastSeq == 7 && s.target[0] == 100 && s.target[4] == 160 && s.current[0] == cf1::Fallback[0], "pose atomic targets, no teleport");
  unchangedReject(s,"POSE 17 7 90 92 104 60 170","ERR SEQUENCE");
  unchangedReject(s,"POSE 17 6 90 92 104 60 170","ERR SEQUENCE");
  check(command(s,"HOLD 17",30) == "ACK HOLD 7", "hold retains seq");
  for (size_t i=0;i<5;i++) check(s.current[i] == s.target[i], "hold cancels all pending moves");
  check(command(s,"ARM 17",40) == "ACK ARM 7", "rearm retains seq");
  check(command(s,"POSE 17 4294967295 90 92 104 60 170",50) == "ACK POSE 4294967295", "max seq allowed");
  unchangedReject(s,"POSE 17 0 90 92 104 60 170","ERR SEQUENCE");
  unchangedReject(s,"POSE 17 4294967296 90 92 104 60 170","ERR NUMBER");
}

void ratesAndWatchdog() {
  auto s=state(); command(s,limits,0); command(s,"ARM 17",0); s.takeDirtyMask();
  command(s,"POSE 17 1 100 100 120 70 160",0);
  s.tick(20); check(near(s.current[0],90.8f + 9.2f*.03f) && near(s.current[4],179.4f), "synchronized pose within30deg/s maximum");
  s.tick(120); check(near(s.current[0],90.8f + 9.2f*.105f), "elapsed capped50ms");
  s.tick(121); check(near(s.current[0],90.8f + 9.2f*.1065f), "no catchup after stall");
  check(s.takeDirtyMask()==31, "pose writes all changed channels");
  const float before=s.current[0]; s.tick(500);
  check(!s.armed && s.outputsEnabled && s.current[0] == before && s.target[0] == before, "500ms watchdog holds without extra movement");
  s.tick(1000); check(s.current[0] == before, "watchdog stays held");
  check(command(s,"PING 17",1100).empty() && !s.armed, "ping cannot rearm");
  command(s,"ARM 17",1200); s.tick(1200); command(s,"POSE 17 2 100 100 120 70 160",1200);
  command(s,"PING 17",1699); s.tick(1699); check(s.armed, "valid ping refreshes watchdog");
  unchangedReject(s,"PING 18"); s.tick(2199); check(!s.armed, "wrong token ping did not refresh");

  s=state(true,UINT32_MAX-100); command(s,"ARM 17",UINT32_MAX-100);
  s.tick(398); check(s.armed, "wrap at499ms"); s.tick(399); check(!s.armed, "wrap at500ms");
  s=state(); command(s,"ARM 17",0); s.takeDirtyMask(); command(s,"JOG 17 1 0 1",0);
  s.tick(50); check(near(s.current[0],90.95f), "jog3deg/s");
  check(s.takeDirtyMask()==1 && s.current[1] == cf1::Fallback[1], "jog touches only selected channel");
  unchangedReject(s,"JOG 17 2 1 1","ERR BUSY");
  command(s,"JOG 17 2 0 -0.5",50); check(near(s.target[0],90.45f), "jog relative to current not prior target");
  s.tick(100); s.tick(150); s.tick(200); command(s,"PING 17",200); s.tick(250);
  check(near(s.current[0],90.45f) && s.current[0] == s.target[0], "jog stops at exact target");
  check(command(s,"JOG 17 3 1 1",250) == "ACK JOG 3", "next channel after stationary");
  unchangedReject(s,"JOG 17 4 4 1","ERR RANGE");
}

void jogBoundsAndNoHardware() {
  auto s=state(); command(s,"ARM 17",0);
  for (const auto& bad : {"JOG 17 1 5 0.1","JOG 17 1 -1 0.1","JOG 17 1 0 1.001","JOG 17 1 0 -1.001","JOG 17 1 0 NaN","JOG 17 1 4 0.1"}) unchangedReject(s,bad);
  uint32_t now=0;
  for (uint32_t seq=1;seq<=10;++seq) {
    std::string text="JOG 17 "+std::to_string(seq)+" 0 1";
    check(command(s,text,now).rfind("ACK JOG",0)==0, "valid cumulative jog");
    for (int t=0;t<7;t++) { now+=50; s.tick(now); }
  }
  check(near(s.current[0],100.8f), "ten degree jog envelope");
  unchangedReject(s,"JOG 17 11 0 0.1","ERR RANGE");
  command(s,"HOLD 17",now);
  check(command(s,"LIMITS 17 100.8 101 92 94 104 105 60 62 179 180 1",now).rfind("ACK LIMITS",0)==0, "narrow current-containing limits");
  command(s,"ARM 17",now);
  unchangedReject(s,"JOG 17 11 0 -0.1","ERR RANGE");

  s=state(false);
  check(command(s,"HELLO").rfind("HELLO CF1",0)==0 && command(s,"STATUS").rfind("STATE",0)==0, "absent PCA read-only status");
  unchangedReject(s,"ARM 17","ERR PCA");
  unchangedReject(s,"JOG 17 1 0 1","ERR PCA");
  unchangedReject(s,"POSE 17 1 90 92 104 60 170","ERR PCA");
  check(command(s,"HOLD 17") == "ACK HOLD 0" && s.outputsEnabled && !s.armed, "hold available when absent without claiming confirmed off");
}

void releaseAndRearm() {
  auto s=state();
  unchangedReject(s,"RELEASE 18","ERR BOOT");
  unchangedReject(s,"RELEASE 17 extra","ERR TOKENS");
  check(command(s,"RELEASE 17") == "ACK RELEASE 0" && !s.armed && !s.outputsEnabled && !s.limitsSet, "release accepted when already off");
  check(s.takeReleaseRequest() && !s.takeReleaseRequest(), "release hardware action consumed once");
  command(s,limits,0); command(s,"ARM 17",0);
  command(s,"POSE 17 1 100 100 120 70 160",0); s.tick(50);
  const float releasedAt = s.current[0];
  check(s.dirtyMask != 0 && s.limitsSet, "movement queued before release");
  check(command(s,"RELEASE 17",50) == "ACK RELEASE 1", "release retains sequence");
  check(!s.armed && !s.outputsEnabled && !s.limitsSet && s.dirtyMask == 0 && s.releaseRequested, "release clears targets, enable, limits, pending output");
  for (size_t i=0;i<5;i++) check(s.target[i] == s.current[i], "release cancels pending target");
  check(command(s,"STATUS").rfind("STATE 17 0 0 0 1 ",0) == 0, "released telemetry has outputs off");
  command(s,"PING 17",200); s.tick(200); command(s,"HOLD 17",200);
  check(!s.outputsEnabled && !s.armed && s.current[0] == releasedAt, "ping/hold cannot reactivate released pulses");
  s.takeReleaseRequest();
  check(command(s,"ARM 17",250) == "ACK ARM 1" && s.outputsEnabled && s.armed, "only explicit arm reenables");
  check(s.current[0] == releasedAt && s.target[0] == releasedAt && s.dirtyMask == 31, "arm resumes command reference, not startup neutral");
  unchangedReject(s,"POSE 17 2 100 100 120 70 160","ERR LIMITS");
  command(s,"HOLD 17",260);
  check(command(s,"STATUS").rfind("STATE 17 0 0 1 1 ",0) == 0, "HOLD telemetry distinguishes holding outputs");
  command(s,limits,270); command(s,"ARM 17",280);
  check(command(s,"POSE 17 2 100 100 120 70 160",290) == "ACK POSE 2", "limits must be explicitly restored after release");
}

void framing() {
  auto s=state(); cf1::LineReceiver receiver; char out[224];
  auto feed=[&](const std::string& bytes) {
    std::vector<std::string> result;
    for (char c:bytes) if(receiver.feed(c,s,10,out,sizeof(out))) result.emplace_back(out);
    return result;
  };
  check(feed("HE").empty(), "partial input not executed");
  auto lines=feed("LLO\r\nSTATUS\n"); check(lines.size()==2 && lines[0].rfind("HELLO CF1",0)==0 && lines[1].rfind("STATE",0)==0, "fragmented and multi-line frames");
  lines=feed(std::string("HELLO")+std::string(187,' ')+"\n"); check(lines.size()==1 && lines[0].rfind("HELLO CF1",0)==0, "exact192bytes accepted");
  lines=feed(std::string("ARM 17")+std::string(187,' ')+"\nHELLO\n");
  check(lines.size()==2 && lines[0]=="ERR LINE" && lines[1].rfind("HELLO CF1",0)==0 && !s.outputsEnabled, "overflow discard through newline and recover");
  std::string nul="ARM 17"; nul.push_back(0); nul+="\nHELLO\n"; lines=feed(nul);
  check(lines.size()==2 && lines[0]=="ERR SYNTAX" && !s.outputsEnabled, "embedded NUL cannot truncate into ARM");
  lines=feed("ARM 17\xff\n"); check(lines[0]=="ERR SYNTAX" && !s.outputsEnabled, "nonascii cannot arm");
  // All single-byte mutations of a valid ARM command except original byte must either
  // reject or be valid whitespace/decimal formatting; replay uses wrong boot token.
  for (int c=0;c<256;c++) {
    auto fresh=state(); cf1::LineReceiver r;
    std::string bad="ARM 18"; bad.push_back(static_cast<char>(c)); bad+='\n';
    for(char byte:bad) r.feed(byte,fresh,10,out,sizeof(out));
    check(!fresh.outputsEnabled && !fresh.armed,"malformed/stale identity enabled PWM");
  }
}

int main() {
  try {
    independentStartupHome(); startupAndParser(); atomicAndLimits(); ratesAndWatchdog(); jogBoundsAndNoHardware(); releaseAndRearm(); framing();
    std::cout << "PASS " << checks << " assertions: startup, parser, framing, atomic rejection, limits, sequence, rates, watchdog, PCA absence\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << "FAIL after " << checks << ": " << e.what() << '\n'; return 1; }
}
