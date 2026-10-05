#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace cf1 {
// Timestamped first-order source, averaged over a positive40ms window.
// All five axes share one time coordinate. The output derivative is exactly
// (source(t)-source(t-40))/40, so a speed-bounded source remains speed-bounded.
struct TimestampedTrajectory {
  static constexpr size_t Axes = 5, Capacity = 24;
  static constexpr double WindowMs = 40, DelayMs = 40;
  static constexpr uint64_t MaximumTimestampUs = 86400000000ULL;
  struct Point { double time = 0; double q[Axes] = {}; };
  Point points[Capacity];
  size_t count = 0;
  bool active = false;
  double elapsedMs = 0;
  uint64_t lastStampUs = 0;
  uint32_t firstArrivalMs = 0;

  void clear() { active = false; count = 0; elapsedMs = 0; lastStampUs = 0; firstArrivalMs = 0; }
  double cursor() const { return elapsedMs - DelayMs; }

  // Caller validates pose bounds first. No state changes on any failure.
  const char* append(uint64_t stampUs, const float* q, const float* current,
                     double speed, uint32_t arrivalMs) {
    if (!isfinite(speed) || speed <= 0 || speed > 400) return "STREAM_SPEED";
    if (stampUs > MaximumTimestampUs) return "STREAM_TIME";
    if (!active && stampUs != 0) return "STREAM_TIME";
    if (active) {
      if (stampUs <= lastStampUs || stampUs - lastStampUs > 500000ULL) return "STREAM_TIME";
      const double wallMs = static_cast<uint32_t>(arrivalMs - firstArrivalMs);
      const double sourceMs = stampUs * .001;
      if (sourceMs > wallMs + 100 || sourceMs + 250 < wallMs) return "STREAM_STALE";
    }
    Point start;
    if (active) start = points[count - 1];
    else for (size_t i = 0; i < Axes; ++i) start.q[i] = current[i];
    const double priorTime = start.time;
    const bool anchor = active && cursor() > start.time;
    if (anchor) start.time = cursor(); // Preserve the already-played constant tail.
    double duration = 0;
    for (size_t i = 0; i < Axes; ++i) {
      if (!isfinite(q[i]) || q[i] < 0 || q[i] > 180 || !isfinite(start.q[i])) return "STREAM_RANGE";
      const double required = fabs(static_cast<double>(q[i]) - start.q[i]) / speed * 1000;
      if (required > duration) duration = required;
    }
    double end = active ? priorTime + (stampUs - lastStampUs) * .001 : 0;
    if (end < start.time + duration) end = start.time + duration;
    const bool endpoint = end > start.time;
    const size_t needed = (active ? 0 : 1) + (anchor ? 1 : 0) + (endpoint ? 1 : 0);
    if (count + needed > Capacity) return "STREAM_BUFFER";
    if (!active) {
      active = true; elapsedMs = 0; firstArrivalMs = arrivalMs;
      points[count++] = start;
    } else if (anchor) points[count++] = start;
    if (endpoint) {
      Point next; next.time = end;
      for (size_t i = 0; i < Axes; ++i) next.q[i] = q[i];
      points[count++] = next;
    }
    lastStampUs = stampUs;
    return nullptr;
  }

  double value(double time, size_t axis) const {
    if (time <= points[0].time) return points[0].q[axis];
    for (size_t i = 1; i < count; ++i) if (time < points[i].time) {
      const Point& a = points[i - 1]; const Point& b = points[i];
      const double fraction = (time - a.time) / (b.time - a.time);
      return a.q[axis] + (b.q[axis] - a.q[axis]) * fraction;
    }
    // No extrapolation. An underrun drains only toward the last known endpoint.
    return points[count - 1].q[axis];
  }

  double average(double end, size_t axis) const {
    double start = end - WindowMs, integral = 0;
    // Piecewise-linear trapezoids integrate exactly, including constant tails.
    for (size_t i = 0; i < count; ++i) {
      const double boundary = points[i].time;
      if (boundary <= start || boundary >= end) continue;
      integral += (value(start, axis) + value(boundary, axis)) * .5 * (boundary - start);
      start = boundary;
    }
    integral += (value(start, axis) + value(end, axis)) * .5 * (end - start);
    return integral / WindowMs;
  }

  void advance(double deltaMs, float* result) {
    if (!active || deltaMs <= 0) return;
    elapsedMs += deltaMs;
    const double end = cursor();
    for (size_t i = 0; i < Axes; ++i) result[i] = static_cast<float>(average(end, i));
    // Keep one predecessor for the oldest point needed by the next integral.
    while (count > 1 && points[1].time <= end - WindowMs) {
      for (size_t i = 1; i < count; ++i) points[i - 1] = points[i];
      --count;
    }
  }
};
}
