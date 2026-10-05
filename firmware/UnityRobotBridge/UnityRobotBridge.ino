// CF1 USB-only bridge. Never starts motion on boot or serial connection.
// Keep servo V+ OFF during upload/reset. Existing calibration sketch is untouched.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <esp_system.h>
#include "BridgeState.h"
#include "StartupHome.h"

Adafruit_PWMServoDriver pwm(0x40);
cf1::BridgeState bridge;
cf1::LineReceiver receiver;
uint32_t previousStateMs = 0;
char response[224];

bool forceAllOutputsOff() {
  // ALL_LED_OFF_H writes the full-off flag into all16 LEDn_OFF_H registers.
  // One addressed write works even before the driver's auto-increment setup.
  Wire.beginTransmission(0x40);
  Wire.write(PCA9685_ALLLED_OFF_H);
  Wire.write(0x10);
  return Wire.endTransmission() == 0;
}

void applyPendingOutputs() {
  // RELEASE is applied before its ACK, and before consuming another command.
  if (bridge.takeReleaseRequest()) {
    if (!forceAllOutputsOff()) {
      bridge.hardwareReady = false;
      bridge.outputsEnabled = true; // Off is unconfirmed; never claim signals OFF.
      snprintf(response, sizeof(response), "ERR PCA");
    }
  }
  const uint8_t dirty = bridge.takeDirtyMask();
  if (bridge.hardwareReady && bridge.outputsEnabled) {
    for (uint8_t channel = 0; channel < cf1::Channels; ++channel) {
      if (dirty & (1U << channel)) {
        // Preserve the calibration sketch's exact arithmetic and clock convention.
        const uint16_t pulseUs = static_cast<uint16_t>(lroundf(
          600.0f + (bridge.current[channel] / 180.0f) * (2400.0f - 600.0f)));
        pwm.writeMicroseconds(channel, pulseUs);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setTimeOut(25);
  // Adafruit begin() restarts the oscillator and briefly selects1000Hz. Clear
  // stale PCA registers BEFORE that call if the PCA survived an ESP32 reset.
  bool ready = forceAllOutputsOff();
  if (ready) ready = pwm.begin();
  if (ready) {
    // Existing neutral NVS values were physically calibrated with this clock
    // parameter. It is a conversion reference, not an oscillator measurement.
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    ready = forceAllOutputsOff();
  }
  float saved[cf1::Channels];
  for (size_t i = 0; i < cf1::Channels; ++i) saved[i] = cf1::Fallback[i];
  Preferences storage;
  // Read-only: retain original sketch's neutral values and storage namespace.
  if (storage.begin("arm-neutral", true)) {
    for (size_t i = 0; i < cf1::Channels; ++i) {
      char key[16]; snprintf(key, sizeof(key), "neutral%u", static_cast<unsigned>(i));
      saved[i] = storage.getFloat(key, cf1::Fallback[i]);
    }
    storage.end();
  }
  uint32_t token;
  do { token = esp_random(); } while (!token);
  const uint32_t now = millis();
  bridge.begin(token, saved, now, ready);
  // Prepare the user's saved resting pose without generating any PWM.
  // HELLO still reports the original NVS calibration reference; STATE reports home.
  // First deliberate ARM writes this pose. HOLD/RELEASE never reset it.
  if (ready && !bridge.initializeStartupPose(cf1::StartupHome)) {
    // Do not enable a partially validated or unexpected startup profile.
    bridge.hardwareReady = false;
  }
  previousStateMs = now;
}

void loop() {
  uint32_t now = millis();
  bridge.tick(now);
  // Bound serial work to prevent long command bursts starving the watchdog.
  for (uint8_t bytes = 0; bytes < 64 && Serial.available(); ++bytes) {
    now = millis();
    bridge.tick(now);
    if (receiver.feed(static_cast<char>(Serial.read()), bridge, now, response, sizeof(response))) {
      // TIMING is read-only; the state parser delegates actual register reading.
      if (!strcmp(response, "TIMING CF1")) {
        if (bridge.hardwareReady) snprintf(response, sizeof(response), "TIMING CF1 %lu %u",
          static_cast<unsigned long>(pwm.getOscillatorFrequency()), static_cast<unsigned>(pwm.readPrescale()));
        else snprintf(response, sizeof(response), "ERR PCA");
      }
      applyPendingOutputs();
      if (response[0]) Serial.println(response);
    }
  }
  applyPendingOutputs();
  now = millis();
  if (static_cast<uint32_t>(now - previousStateMs) >= 100) {
    previousStateMs = now;
    bridge.state(response, sizeof(response));
    Serial.println(response);
  }
  delay(1);
}
