#pragma once
#include "TestHardware.h"
#define PCA9685_ALLLED_OFF_H 0xFD
struct Adafruit_PWMServoDriver {
  uint32_t oscillator=25000000;
  explicit Adafruit_PWMServoDriver(uint8_t) {}
  bool begin() {
    ++fake::pcaBegins; fake::events.emplace_back("pca-begin");
    if(!fake::allOff()) throw std::runtime_error("PCA restart happened before outputs off");
    oscillator=25000000;
    return !fake::failBegin;
  }
  void setOscillatorFrequency(uint32_t hz) { oscillator=hz; fake::events.emplace_back("oscillator"+std::to_string(hz)); }
  uint32_t getOscillatorFrequency() { return oscillator; }
  uint8_t readPrescale() { return 131; }
  void setPWMFreq(float hz) { if(hz != 50 || oscillator != 27000000) throw std::runtime_error("wrong PWM timing convention"); fake::events.emplace_back("50Hz"); }
  void writeMicroseconds(uint8_t channel,uint16_t us) {
    if(channel>4 || us<600 || us>2400) throw std::runtime_error("wrong servo pulse");
    fake::output[channel]=true; fake::pulseMicroseconds[channel]=us;
    ++fake::pulseWrites; fake::events.emplace_back("pulse"+std::to_string(channel));
  }
};
