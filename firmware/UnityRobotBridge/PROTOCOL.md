# CF1 USB command protocol

This reference describes the included source. Transport is USB serial at **115200 baud, 8N1**, newline-terminated ASCII, with a maximum incoming line length of 192 bytes. The host should leave DTR and RTS unasserted during an ordinary connection.

Commands use channel order **base, shoulder, elbow, wrist pitch, gripper**. Angle values are nominal servo command degrees. They are not Unity Euler angles or measured shaft feedback.

## Read-only queries

| Request | Reply |
| --- | --- |
| `HELLO` | `HELLO CF1 boot n0 n1 n2 n3 n4` — original calibration reference |
| `STATUS` | `STATE boot armed limitsSet outputsEnabled lastSeq a0 a1 a2 a3 a4` |
| `TIMING` | `TIMING CF1 27000000 131` when the PCA is ready |
| `MOTION` | `MOTION CF1 400 40` — maximum command-speed setting and minimum POSE segment duration |
| `TRAJECTORY boot` | `TRAJECTORY CF1 boot TPOSE1 40 40 400` — timestamped stream capability |

`boot` is a nonzero session identifier generated on ESP32 boot. It is not an authentication credential. `STATE` is also emitted every 100 ms.

## Control commands

| Request | Behavior |
| --- | --- |
| `ARM boot` | Enables channels 0–4 at current command positions; requires ready hardware |
| `HOLD boot` | Cancels motion, disarms, retains already-enabled holding pulses |
| `RELEASE boot` | Disarms, invalidates limits, disables all 16 PCA outputs before ACK |
| `PING boot` | Refreshes watchdog while armed |
| `LIMITS boot lo0 hi0 lo1 hi1 lo2 hi2 lo3 hi3 lo4 hi4 speed` | Sets bounds while disarmed; bounds must include current commands and lie within 0–180; speed must be greater than 0 and at most 400 |
| `JOG boot seq channel delta` | At most ±1 degree on one channel, at 3 degrees/second, inside calibration neutral ±10, 0–180 and any active session bounds |
| `POSE boot seq a0 a1 a2 a3 a4` | Validates all targets; follows one synchronized linear joint-space segment, at least 40 ms long |
| `TPOSE boot seq timestampUs a0 a1 a2 a3 a4` | Accepts a bounded timestamped pose stream with shared time coordinates and smoothing |

Successful control requests return `ACK verb lastSeq`; `PING` has no reply. `seq` is an increasing unsigned 32-bit sequence for JOG/POSE/TPOSE; it must not wrap or restart within a boot session. POSE/TPOSE require both armed state and valid session limits. Repeated identical POSE targets refresh liveness without restarting interpolation.

TPOSE begins at timestamp zero and then uses strictly increasing timestamps. The implementation rejects invalid, stale, overflowing or incompatible streams and holds. It does not extrapolate past the last received endpoint. Switching between timestamped and ordinary motion modes requires ending the previous motion session rather than mixing packet types.

## State and limitations

- A 500 ms watchdog without valid ARM/PING/JOG/POSE/TPOSE activity holds and disarms.
- Numeric parsing rejects nonfinite values, malformed numbers and out-of-range targets. Invalid requests return `ERR code`; stream-integrity failures also hold.
- A failed PCA initialization or output-off write prevents further arming. `outputsEnabled=1` then conservatively means **enabled or not confirmed off**. It is not power sensing.
- HOLD retains PWM; RELEASE removes PWM but does not switch servo V+.
- Only ARM enables outputs. Loading limits, querying status and reconnecting do not arm, home or move the robot.
- The 27 MHz timing value is a conversion reference and the prescale is a register value, not a direct oscillator measurement.

The PC side performs model-to-servo mapping, operator confirmation, travel selection and controller-loss/focus-loss handling. Firmware bounds cannot establish physical clearance. See the [firmware guide](../README.md) before use.
