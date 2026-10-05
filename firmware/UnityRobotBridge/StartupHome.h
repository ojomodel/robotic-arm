#pragma once

// User-saved "new neutral", 2026-09-24T17:36:05.6502233Z.
// Model [0,22,-56,-20,-60], matching held command telemetry below.
// This selects the first-enable/resting pose, not a new calibration reference.
// Original arm-neutral NVS, affine mapping and travel intervals stay unchanged.
namespace cf1 {
constexpr float StartupHome[5] = {90.8f, 92.9f, 118.5f, 60.9f, 180.0f};
}
