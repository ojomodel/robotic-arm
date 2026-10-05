#pragma once
#include "TestHardware.h"
struct Preferences {
  bool begin(const char* name,bool readOnly) {
    if(std::string(name)!="arm-neutral" || !readOnly) throw std::runtime_error("NVS was not read-only original namespace");
    fake::readOnlyNvs=true; return true;
  }
  float getFloat(const char* key,float fallback) {
    if(std::string(key).rfind("neutral",0)!=0) throw std::runtime_error("wrong NVS key");
    return fallback;
  }
  void end() {}
};
