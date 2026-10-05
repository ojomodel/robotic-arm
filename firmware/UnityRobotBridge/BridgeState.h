#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "TimestampedTrajectory.h"

// Hardware-independent CF1 parser/state machine, also exercised by host tests.
namespace cf1 {
constexpr size_t Channels = 5, MaxLine = 192;
constexpr uint32_t WatchdogMs = 500;
constexpr uint16_t MaximumPoseSpeed = 400;
constexpr uint32_t MinimumPoseSegmentMs = 40;
constexpr float Fallback[Channels] = {90.8f, 92.9f, 104.5f, 60.9f, 180.0f};

inline bool unsignedDecimal(const char* value, uint32_t& result) {
  if (!value || !*value) return false;
  uint32_t number = 0;
  for (const char* p = value; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const uint32_t digit = static_cast<uint32_t>(*p - '0');
    if (number > (UINT32_MAX - digit) / 10U) return false;
    number = number * 10U + digit;
  }
  result = number;
  return true;
}

inline bool finiteDecimal(const char* value, float& result) {
  if (!value || !*value) return false;
  const char* p = value;
  if (*p == '+' || *p == '-') ++p;
  bool digits = false;
  while (*p >= '0' && *p <= '9') { digits = true; ++p; }
  if (*p == '.') {
    ++p;
    while (*p >= '0' && *p <= '9') { digits = true; ++p; }
  }
  if (!digits) return false;
  if (*p == 'e' || *p == 'E') {
    ++p;
    if (*p == '+' || *p == '-') ++p;
    const char* exponentStart = p;
    while (*p >= '0' && *p <= '9') ++p;
    if (p == exponentStart) return false;
  }
  if (*p) return false;
  errno = 0;
  char* end = nullptr;
  const float number = strtof(value, &end);
  if (!end || *end || errno == ERANGE || !isfinite(number)) return false;
  result = number;
  return true;
}

inline bool timestampDecimal(const char* value, uint64_t& result) {
  if (!value || !*value) return false;
  uint64_t n = 0;
  for (const char* p = value; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const uint64_t digit = static_cast<uint64_t>(*p - '0');
    if (n > (TimestampedTrajectory::MaximumTimestampUs - digit) / 10) return false;
    n = n * 10 + digit;
  }
  result = n; return true;
}

struct BridgeState {
  uint32_t boot = 1, lastSeq = 0, lastValidMs = 0, lastTickMs = 0;
  bool armed = false, limitsSet = false, outputsEnabled = false, hardwareReady = false, releaseRequested = false;
  float neutral[Channels] = {}, current[Channels] = {}, target[Channels] = {};
  float low[Channels] = {}, high[Channels] = {}, speed = 3.0f, motionSpeed = 3.0f;
  float segmentStart[Channels] = {};
  double segmentElapsedMs = 0, segmentDurationMs = 0;
  bool poseMode = false, segmentActive = false;
  bool startupPoseInitialized = false;
  uint8_t dirtyMask = 0;
  TimestampedTrajectory timestamped;

  void begin(uint32_t token, const float* saved, uint32_t now, bool pcaReady) {
    boot = token ? token : 1;
    lastSeq = 0; lastValidMs = lastTickMs = now;
    armed = limitsSet = false;
    // If initialization failed, software cannot confirm the PCA outputs are off.
    // Conservatively report enabled/unknown, while ARM remains unavailable.
    outputsEnabled = !pcaReady;
    hardwareReady = pcaReady; dirtyMask = 0; releaseRequested = false;
    speed = motionSpeed = 3.0f;
    poseMode = segmentActive = false; segmentElapsedMs = segmentDurationMs = 0;
    startupPoseInitialized = false;
    timestamped.clear();
    for (size_t i = 0; i < Channels; ++i) {
      const float n = saved ? saved[i] : Fallback[i];
      neutral[i] = current[i] = target[i] = isfinite(n) && n >= 0 && n <= 180 ? n : Fallback[i];
      segmentStart[i] = current[i];
      low[i] = 0; high[i] = 180;
    }
  }

  // A saved resting pose is independent of the immutable calibration anchor.
  // Called once by setup, while outputs are confirmed OFF; never a serial action.
  bool initializeStartupPose(const float* pose) {
    if (!pose || !hardwareReady || outputsEnabled || armed || limitsSet || lastSeq != 0 ||
        dirtyMask != 0 || releaseRequested || segmentActive || startupPoseInitialized) return false;
    for (size_t i = 0; i < Channels; ++i)
      if (!isfinite(pose[i]) || pose[i] < 0 || pose[i] > 180) return false;
    for (size_t i = 0; i < Channels; ++i)
      current[i] = target[i] = segmentStart[i] = pose[i];
    startupPoseInitialized = true;
    return true;
  }

  void hold() {
    armed = false;
    poseMode = segmentActive = false; segmentElapsedMs = segmentDurationMs = 0;
    timestamped.clear();
    for (size_t i = 0; i < Channels; ++i) target[i] = current[i];
    // Holding PWM stays enabled after ARM; this is not electrical power-off.
  }

  void release() {
    hold();
    outputsEnabled = false;
    limitsSet = false;
    dirtyMask = 0; // Never flush queued movement after a release.
    releaseRequested = true;
  }

  void tick(uint32_t now) {
    const uint32_t elapsed = now - lastTickMs;
    lastTickMs = now; // Discard excess time; never catch up after a loop stall.
    if (armed && static_cast<uint32_t>(now - lastValidMs) >= WatchdogMs) hold();
    if (!armed || !outputsEnabled || !hardwareReady) return;
    const float elapsedMs = static_cast<float>(elapsed > 50 ? 50 : elapsed);
    if (timestamped.active) {
      // A long firmware stall cannot replay a delayed backlog in a catch-up jump.
      if (elapsed > 50) { hold(); return; }
      float next[Channels]; timestamped.advance(elapsedMs, next);
      if (elapsedMs == 0) return;
      for (size_t i = 0; i < Channels; ++i) {
        const float bounded = next[i] < low[i] ? low[i] : (next[i] > high[i] ? high[i] : next[i]);
        if (bounded != current[i]) { current[i] = bounded; dirtyMask |= static_cast<uint8_t>(1U << i); }
      }
      return;
    }
    if (poseMode) {
      if (!segmentActive || elapsedMs == 0) return;
      segmentElapsedMs += elapsedMs;
      const bool arrived = segmentElapsedMs >= segmentDurationMs;
      const float fraction = arrived ? 1.0f : static_cast<float>(segmentElapsedMs / segmentDurationMs);
      for (size_t i = 0; i < Channels; ++i) {
        float next = arrived ? target[i] : segmentStart[i] + (target[i] - segmentStart[i]) * fraction;
        // Both endpoints were validated together. Keep arithmetic inside that
        // closed interval; no target overshoot or 180-to-zero wrapping.
        const float a = segmentStart[i] < target[i] ? segmentStart[i] : target[i];
        const float b = segmentStart[i] > target[i] ? segmentStart[i] : target[i];
        next = next < a ? a : (next > b ? b : next);
        if (next != current[i]) { current[i] = next; dirtyMask |= static_cast<uint8_t>(1U << i); }
      }
      if (arrived) { segmentActive = false; segmentElapsedMs = segmentDurationMs; }
      return;
    }
    const float step = motionSpeed * elapsedMs * .001f;
    for (size_t i = 0; i < Channels; ++i) {
      const float delta = target[i] - current[i];
      if (delta == 0 || step == 0) continue;
      current[i] += fabsf(delta) <= step ? delta : (delta > 0 ? step : -step);
      dirtyMask |= static_cast<uint8_t>(1U << i);
    }
  }

  uint8_t takeDirtyMask() { const uint8_t result = dirtyMask; dirtyMask = 0; return result; }
  bool takeReleaseRequest() { const bool result = releaseRequested; releaseRequested = false; return result; }

  void state(char* out, size_t capacity) const {
    snprintf(out, capacity, "STATE %lu %u %u %u %lu %.3f %.3f %.3f %.3f %.3f",
      static_cast<unsigned long>(boot), armed ? 1U : 0U, limitsSet ? 1U : 0U,
      outputsEnabled ? 1U : 0U,
      static_cast<unsigned long>(lastSeq), current[0], current[1], current[2], current[3], current[4]);
  }

  // No tick here: invalid packets have no side effects; loop ticks independently.
  void command(const char* line, uint32_t now, char* out, size_t capacity) {
    if (!capacity) return;
    out[0] = 0;
    if (!line || strlen(line) > MaxLine) { error(out, capacity, "LINE"); return; }
    char copy[MaxLine + 1];
    strcpy(copy, line);
    const char* tokens[16]; size_t count = 0;
    bool inToken = false;
    for (char* p = copy; *p; ++p) {
      const unsigned char c = static_cast<unsigned char>(*p);
      if (c == ' ' || c == '\t' || c == '\r') { *p = 0; inToken = false; }
      else {
        if (c < 33 || c > 126) { error(out, capacity, "SYNTAX"); return; }
        if (!inToken) {
          if (count == 16) { error(out, capacity, "TOKENS"); return; }
          tokens[count++] = p; inToken = true;
        }
      }
    }
    if (!count) { error(out, capacity, "SYNTAX"); return; }
    const char* verb = tokens[0];
    if (!strcmp(verb, "HELLO") && count == 1) {
      snprintf(out, capacity, "HELLO CF1 %lu %.3f %.3f %.3f %.3f %.3f",
        static_cast<unsigned long>(boot), neutral[0], neutral[1], neutral[2], neutral[3], neutral[4]);
      return;
    }
    if (!strcmp(verb, "STATUS") && count == 1) { state(out, capacity); return; }
    if (!strcmp(verb, "TIMING") && count == 1) { snprintf(out, capacity, "TIMING CF1"); return; }
    if (!strcmp(verb, "MOTION") && count == 1) {
      snprintf(out, capacity, "MOTION CF1 %u %lu", static_cast<unsigned>(MaximumPoseSpeed),
        static_cast<unsigned long>(MinimumPoseSegmentMs));
      return;
    }
    size_t expected = 0;
    if (!strcmp(verb, "ARM") || !strcmp(verb, "HOLD") || !strcmp(verb, "RELEASE") || !strcmp(verb, "PING") || !strcmp(verb, "TRAJECTORY")) expected = 2;
    else if (!strcmp(verb, "JOG")) expected = 5;
    else if (!strcmp(verb, "LIMITS")) expected = 13;
    else if (!strcmp(verb, "POSE")) expected = 8;
    else if (!strcmp(verb, "TPOSE")) expected = 9;
    else { error(out, capacity, "COMMAND"); return; }
    if (count != expected) { error(out, capacity, "TOKENS"); return; }
    uint32_t token;
    if (!unsignedDecimal(tokens[1], token)) { error(out, capacity, "NUMBER"); return; }
    if (token != boot) { error(out, capacity, "BOOT"); return; }
    if (!strcmp(verb, "TRAJECTORY")) {
      snprintf(out, capacity, "TRAJECTORY CF1 %lu TPOSE1 40 40 %u", static_cast<unsigned long>(boot), static_cast<unsigned>(MaximumPoseSpeed)); return;
    }
    if (!strcmp(verb, "HOLD")) { hold(); ack(out, capacity, "HOLD", lastSeq); return; }
    if (!strcmp(verb, "RELEASE")) { release(); ack(out, capacity, "RELEASE", lastSeq); return; }
    if (!strcmp(verb, "PING")) { if (armed) lastValidMs = now; return; }
    if (!strcmp(verb, "ARM")) {
      if (!hardwareReady) { error(out, capacity, "PCA"); return; }
      armed = outputsEnabled = true; lastValidMs = now; dirtyMask = 31;
      ack(out, capacity, "ARM", lastSeq); return;
    }
    if (!strcmp(verb, "LIMITS")) {
      if (armed) { error(out, capacity, "ARMED"); return; }
      float parsed[11];
      for (size_t i = 0; i < 11; ++i) {
        if (!finiteDecimal(tokens[i + 2], parsed[i])) { error(out, capacity, "NUMBER"); return; }
      }
      for (size_t i = 0; i < Channels; ++i) {
        if (parsed[2*i] < 0 || parsed[2*i+1] > 180 || parsed[2*i] > current[i] || parsed[2*i+1] < current[i]) {
          error(out, capacity, "RANGE"); return;
        }
      }
      if (parsed[10] <= 0 || parsed[10] > MaximumPoseSpeed) { error(out, capacity, "RANGE"); return; }
      for (size_t i = 0; i < Channels; ++i) { low[i] = parsed[2*i]; high[i] = parsed[2*i+1]; }
      speed = parsed[10]; limitsSet = true;
      ack(out, capacity, "LIMITS", lastSeq); return;
    }
    if (!hardwareReady) { error(out, capacity, "PCA"); return; }
    if (!armed) { error(out, capacity, "DISARMED"); return; }
    uint32_t seq;
    if (!unsignedDecimal(tokens[2], seq)) { error(out, capacity, "NUMBER"); return; }
    if (seq <= lastSeq) { error(out, capacity, "SEQUENCE"); return; }
    if (!strcmp(verb, "TPOSE")) {
      if (!limitsSet) { error(out, capacity, "LIMITS"); return; }
      if (poseMode || segmentActive) { hold(); error(out, capacity, "STREAM_MODE"); return; }
      uint64_t stamp; float proposed[Channels];
      if (!timestampDecimal(tokens[3], stamp)) { hold(); error(out, capacity, "STREAM_TIME"); return; }
      for (size_t i = 0; i < Channels; ++i) {
        if (!finiteDecimal(tokens[i + 4], proposed[i])) { error(out, capacity, "NUMBER"); return; }
        if (proposed[i] < low[i] || proposed[i] > high[i]) { error(out, capacity, "RANGE"); return; }
      }
      const char* failure = timestamped.append(stamp, proposed, current, speed, now);
      if (failure) { hold(); error(out, capacity, failure); return; }
      for (size_t i = 0; i < Channels; ++i) target[i] = proposed[i];
      lastSeq = seq; lastValidMs = now; ack(out, capacity, "TPOSE", seq); return;
    }
    if (timestamped.active) { hold(); error(out, capacity, "STREAM_MODE"); return; }
    if (!strcmp(verb, "JOG")) {
      uint32_t channel; float delta;
      if (!unsignedDecimal(tokens[3], channel) || !finiteDecimal(tokens[4], delta)) { error(out, capacity, "NUMBER"); return; }
      if (channel >= Channels || fabsf(delta) > 1.0f) { error(out, capacity, "RANGE"); return; }
      const float proposed = current[channel] + delta;
      if (proposed < 0 || proposed > 180 || proposed < neutral[channel]-10 || proposed > neutral[channel]+10 ||
          (limitsSet && (proposed < low[channel] || proposed > high[channel]))) { error(out, capacity, "RANGE"); return; }
      for (size_t i = 0; i < Channels; ++i) {
        if (i != channel && current[i] != target[i]) { error(out, capacity, "BUSY"); return; }
      }
      target[channel] = proposed; motionSpeed = 3.0f;
      poseMode = segmentActive = false; segmentElapsedMs = segmentDurationMs = 0;
      lastSeq = seq; lastValidMs = now; ack(out, capacity, "JOG", seq); return;
    }
    if (!limitsSet) { error(out, capacity, "LIMITS"); return; }
    float proposed[Channels];
    for (size_t i = 0; i < Channels; ++i) {
      if (!finiteDecimal(tokens[i+3], proposed[i])) { error(out, capacity, "NUMBER"); return; }
      if (proposed[i] < low[i] || proposed[i] > high[i]) { error(out, capacity, "RANGE"); return; }
    }
    bool changed = !poseMode;
    for (size_t i = 0; i < Channels; ++i) changed |= proposed[i] != target[i];
    // Identical repeated targets refresh liveness/sequence without restarting
    // the trajectory forever. A genuinely new target replans from current.
    if (changed) {
      float largestDelta = 0;
      for (size_t i = 0; i < Channels; ++i) {
        segmentStart[i] = current[i]; target[i] = proposed[i];
        const float distance = fabsf(target[i] - current[i]);
        if (distance > largestDelta) largestDelta = distance;
      }
      segmentDurationMs = static_cast<double>(largestDelta) / speed * 1000.0;
      if (segmentDurationMs < MinimumPoseSegmentMs) segmentDurationMs = MinimumPoseSegmentMs;
      segmentElapsedMs = 0; segmentActive = largestDelta > 0;
    }
    poseMode = true; motionSpeed = speed; lastSeq = seq; lastValidMs = now;
    ack(out, capacity, "POSE", seq);
  }

private:
  static void error(char* out, size_t n, const char* code) { snprintf(out, n, "ERR %s", code); }
  static void ack(char* out, size_t n, const char* verb, uint32_t seq) {
    snprintf(out, n, "ACK %s %lu", verb, static_cast<unsigned long>(seq));
  }
};

struct LineReceiver {
  char line[MaxLine + 1] = {};
  size_t used = 0;
  bool overflow = false, invalid = false;
  bool feed(char c, BridgeState& state, uint32_t now, char* response, size_t capacity) {
    if (c == '\n') {
      if (overflow) snprintf(response, capacity, "ERR LINE");
      else if (invalid) snprintf(response, capacity, "ERR SYNTAX");
      else { line[used] = 0; state.command(line, now, response, capacity); }
      used = 0; overflow = invalid = false;
      return true;
    }
    const unsigned char byte = static_cast<unsigned char>(c);
    if (byte == 0 || (byte < 32 && byte != '\t' && byte != '\r') || byte > 126) invalid = true;
    if (used == MaxLine) overflow = true;
    else line[used++] = c;
    return false;
  }
};
}
