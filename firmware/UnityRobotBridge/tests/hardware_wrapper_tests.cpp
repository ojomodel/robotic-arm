#include <iostream>
#include <algorithm>
#include "../UnityRobotBridge.ino"
int checks=0;
void check(bool value,const char* text) { ++checks; if(!value) throw std::runtime_error(text); }
void fresh() { fake::reset(); Serial.incoming.clear(); Serial.replies.clear(); receiver=cf1::LineReceiver(); setup(); }
void send(const char* text) { Serial.incoming+=text; do { loop(); } while(Serial.available()); }
int main() {
  try {
    fresh();
    check(fake::events.size()>=5 && fake::events[0]=="all-off" && fake::events[1]=="pca-begin" && fake::events[2]=="oscillator27000000" && fake::events[3]=="50Hz" && fake::events[4]=="all-off", "all-off must bracket PCA initialization with legacy oscillator");
    check(fake::allOff() && fake::pulseWrites==0 && !bridge.outputsEnabled && !bridge.armed, "startup generated servo pulse");
    check(fake::readOnlyNvs, "neutral namespace not loaded read-only");
    for(size_t i=0;i<cf1::Channels;++i) {
      check(bridge.current[i]==cf1::StartupHome[i] && bridge.target[i]==cf1::StartupHome[i] &&
            bridge.segmentStart[i]==cf1::StartupHome[i], "startup resting pose mismatch");
    }
    check(bridge.neutral[2]==cf1::Fallback[2] && bridge.current[2]==118.5f,
          "new home must not overwrite the calibration anchor");
    check(!bridge.initializeStartupPose(cf1::Fallback), "startup pose must only initialize once");
    send("HELLO\nSTATUS\nPING 42\n");
    check(fake::allOff() && fake::pulseWrites==0,"connect/status/ping wrote PWM");
    send("TIMING\n");
    check(Serial.replies.back()=="TIMING CF1 27000000 131" && fake::pulseWrites==0 && !bridge.armed, "TIMING should read configuration without PWM");
    send("MOTION\n");
    check(Serial.replies.back()=="MOTION CF1 400 40" && fake::pulseWrites==0 && !bridge.armed, "MOTION must advertise capability without enabling or writing PWM");
    send("ARM 42\n");
    check(fake::pulseWrites==5 && bridge.outputsEnabled,"explicit ARM didn't write five pulses");
    check(bridge.current[2]==118.5f && bridge.target[2]==118.5f, "first ARM reset the saved home");
    const uint16_t expectedHomePulseUs[5] = {1508,1529,1785,1209,2400};
    for(int i=0;i<5;i++) check(fake::pulseMicroseconds[i]==expectedHomePulseUs[i],
                             "first ARM pulse does not match the new operational home");
    for(int i=5;i<16;i++) check(!fake::output[i],"ARM enabled unused channel");
    send("HOLD 42\n");
    check(bridge.outputsEnabled && !bridge.armed && fake::output[0], "HOLD should retain pulse");
    fake::events.clear();
    send("RELEASE 42\n");
    check(fake::allOff() && !bridge.outputsEnabled && !bridge.armed && !bridge.limitsSet, "RELEASE failed to disable all PWM");
    auto off=std::find(fake::events.begin(),fake::events.end(),"all-off");
    auto ack=std::find(fake::events.begin(),fake::events.end(),"reply:ACK RELEASE 0");
    check(off!=fake::events.end() && ack!=fake::events.end() && off<ack, "RELEASE ACK preceded electrical write");
    const int previousPulses=fake::pulseWrites;
    send("HELLO\nPING 42\nSTATUS\nHOLD 42\n");
    check(fake::allOff() && fake::pulseWrites==previousPulses,"later read-only commands reenabled pulses");
    send("ARM 42\n"); check(fake::pulseWrites==previousPulses+5,"explicit rearm must be required");
    fake::failWrite=true; send("RELEASE 42\n");
    check(!bridge.hardwareReady && !bridge.armed && bridge.outputsEnabled && Serial.replies.back()=="ERR PCA", "failed release must report PCA unavailable and off unconfirmed");
    const int onFailure=fake::pulseWrites; send("ARM 42\n");
    check(fake::pulseWrites==onFailure && Serial.replies.back()=="ERR PCA","failed release permitted rearm");

    fake::reset(); fake::failWrite=true; Serial.incoming.clear(); Serial.replies.clear(); receiver=cf1::LineReceiver(); setup();
    check(fake::pcaBegins==0 && fake::pulseWrites==0 && !bridge.hardwareReady && bridge.outputsEnabled && !bridge.armed, "failed preflight off must skip PCA begin and not claim off");
    send("ARM 42\n"); check(fake::pulseWrites==0 && Serial.replies.back()=="ERR PCA", "startup off failure allowed ARM");
    send("TIMING\n"); check(Serial.replies.back()=="ERR PCA", "TIMING must reject absent PCA");

    fresh(); fake::events.clear(); send("ARM 42\nRELEASE 42\nARM 42\n");
    off=std::find(fake::events.begin(),fake::events.end(),"all-off");
    ack=std::find(fake::events.begin(),fake::events.end(),"reply:ACK RELEASE 0");
    check(off!=fake::events.end() && ack!=fake::events.end() && off<ack,"batched RELEASE not flushed before next command");
    check(bridge.armed && fake::pulseWrites==10,"batched explicit second ARM should reenable five channels");
    std::cout << "PASS " << checks << " hardware-wrapper assertions: startup register ordering, no automatic PWM, RELEASE before ACK, failed I2C guards, explicit rearm, NVS read-only\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n'; return 1; }
}
