#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>
#include <stdexcept>

namespace fake {
inline uint32_t now=0;
inline bool failWrite=false, failBegin=false, readOnlyNvs=false;
inline bool output[16]={};
inline uint16_t pulseMicroseconds[16]={};
inline int pcaBegins=0, pulseWrites=0, fullOffWrites=0;
inline std::vector<std::string> events;
inline uint8_t registers[256]={};
inline bool allOff() { for(bool enabled:output) if(enabled) return false; return true; }
inline void reset() {
  now=0; failWrite=failBegin=readOnlyNvs=false;
  pcaBegins=pulseWrites=fullOffWrites=0; events.clear();
  for(uint8_t& value:registers) value=0;
  for(bool& enabled:output) enabled=true; // Simulate PCA surviving prior firmware.
  for(uint16_t& value:pulseMicroseconds) value=0;
}
}
inline uint32_t millis() { return fake::now; }
inline void delay(unsigned ms) { fake::now+=ms; }

struct FakeSerial {
  std::string incoming;
  std::vector<std::string> replies;
  void begin(unsigned) {}
  int available() { return static_cast<int>(incoming.size()); }
  int read() { const char c=incoming.front(); incoming.erase(0,1); return c; }
  void println(const char* value) { replies.emplace_back(value); fake::events.emplace_back(std::string("reply:")+value); }
};
inline FakeSerial Serial;

struct FakeWire {
  std::vector<uint8_t> data;
  void begin(int,int) {}
  void setTimeOut(int) {}
  void beginTransmission(uint8_t address) { if(address != 0x40) throw std::runtime_error("wrong PCA address"); data.clear(); }
  size_t write(uint8_t byte) { data.push_back(byte); return 1; }
  int endTransmission() {
    if(fake::failWrite) { fake::events.emplace_back("off-failed"); return 4; }
    if(data.size()!=2 || data[0]!=0xFD || data[1]!=0x10) throw std::runtime_error("wrong all-off register write");
    for(int i=0;i<16;i++) { fake::output[i]=false; fake::registers[0x09+4*i]=0x10; }
    fake::registers[0xFD]=0x10;
    ++fake::fullOffWrites; fake::events.emplace_back("all-off"); return 0;
  }
};
inline FakeWire Wire;
