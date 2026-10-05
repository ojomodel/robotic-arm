using System;
using System.Globalization;

namespace CareerFair.Robot.Hardware
{
    /// <summary>Servo COMMAND calibration, not measured joint feedback. Unverified by default.</summary>
    [Serializable]
    public sealed class RobotHardwareMapping
    {
        public const int ChannelCount = 5;
        public float[] servoNeutral = { 90.8f, 92.9f, 104.5f, 60.9f, 180f };
        public float[] modelHome = { 0f, 22f, -70f, -20f, -60f };
        public int[] directions = new int[ChannelCount];
        public bool[] directionVerified = new bool[ChannelCount];
        public float[] minimumCommand = { 90.8f, 92.9f, 104.5f, 60.9f, 180f };
        public float[] maximumCommand = { 90.8f, 92.9f, 104.5f, 60.9f, 180f };
        public bool[] boundsVerified = new bool[ChannelCount];

        public static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        public bool ValidateStructure(out string reason)
        {
            if (!Five(servoNeutral) || !Five(modelHome) || !Five(directions) || !Five(directionVerified) ||
                !Five(minimumCommand) || !Five(maximumCommand) || !Five(boundsVerified))
                return Fail("Calibration must contain exactly five channels.", out reason);
            for (int i = 0; i < ChannelCount; i++)
            {
                if (!Finite(servoNeutral[i]) || servoNeutral[i] < 0f || servoNeutral[i] > 180f || !Finite(modelHome[i]))
                    return Fail("Invalid neutral/home on channel " + i + ".", out reason);
                if (directions[i] < -1 || directions[i] > 1 || (directionVerified[i] && directions[i] == 0))
                    return Fail("Invalid verified direction on channel " + i + ".", out reason);
                if (!Finite(minimumCommand[i]) || !Finite(maximumCommand[i]) || minimumCommand[i] < 0f ||
                    maximumCommand[i] > 180f || minimumCommand[i] > servoNeutral[i] || maximumCommand[i] < servoNeutral[i])
                    return Fail("Command limits must contain neutral within 0..180 on channel " + i + ".", out reason);
            }
            reason = null; return true;
        }

        public bool HasVerifiedCalibration(out string reason)
        {
            if (!ValidateStructure(out reason)) return false;
            for (int i = 0; i < ChannelCount; i++)
            {
                if (!directionVerified[i] || directions[i] == 0)
                    return Fail("Verify movement direction for channel " + i + ".", out reason);
                if (!boundsVerified[i]) return Fail("Verify minimum and maximum commands for channel " + i + ".", out reason);
            }
            reason = null; return true;
        }

        public bool NeutralMatches(float[] observed, out string reason, float tolerance = 0.15f)
        {
            if (!ValidateStructure(out reason)) return false;
            if (!Five(observed) || !Finite(tolerance) || tolerance < 0f)
                return Fail("Invalid neutral handshake.", out reason);
            for (int i = 0; i < ChannelCount; i++)
                if (!Finite(observed[i]) || observed[i] < 0f || observed[i] > 180f || Math.Abs(observed[i] - servoNeutral[i]) > tolerance)
                    return Fail("ESP32 neutral differs from this profile on channel " + i + ".", out reason);
            reason = null; return true;
        }

        public bool TryMapPose(float[] modelPose, out float[] commands, out string reason)
        {
            commands = null;
            if (!HasVerifiedCalibration(out reason)) return false;
            if (!Five(modelPose)) return Fail("A pose must contain exactly five angles.", out reason);
            float[] candidate = new float[ChannelCount];
            for (int i = 0; i < ChannelCount; i++)
            {
                float value = servoNeutral[i] + directions[i] * (modelPose[i] - modelHome[i]);
                // Model/servo subtraction can lose a few float ULPs at the same calibrated endpoint.
                // Only absorb arithmetic noise; transmitted commands still stay inside the exact bounds.
                const float roundoffTolerance = .0001f;
                if (!Finite(modelPose[i]) || !Finite(value) || value < minimumCommand[i] - roundoffTolerance || value > maximumCommand[i] + roundoffTolerance)
                    return Fail("Pose exceeds verified servo command bounds on channel " + i + ".", out reason);
                candidate[i] = Math.Max(minimumCommand[i], Math.Min(maximumCommand[i], value));
            }
            commands = candidate; reason = null; return true;
        }

        public bool TryMapTelemetry(float[] commands, out float[] modelPose, out string reason)
        {
            modelPose = null;
            if (!ValidateStructure(out reason)) return false;
            if (!Five(commands)) return Fail("Telemetry must contain exactly five commands.", out reason);
            float[] candidate = new float[ChannelCount];
            for (int i = 0; i < ChannelCount; i++)
            {
                if (!directionVerified[i] || directions[i] == 0)
                    return Fail("Telemetry direction is unverified on channel " + i + ".", out reason);
                if (!Finite(commands[i]) || commands[i] < 0f || commands[i] > 180f)
                    return Fail("Invalid servo command telemetry on channel " + i + ".", out reason);
                candidate[i] = modelHome[i] + directions[i] * (commands[i] - servoNeutral[i]);
                if (!Finite(candidate[i])) return Fail("Invalid mapped model angle.", out reason);
            }
            modelPose = candidate; reason = null; return true;
        }

        private static bool Five(Array array) => array != null && array.Length == ChannelCount;
        private static bool Fail(string message, out string reason) { reason = message; return false; }
    }

    public enum CF1MessageKind { Hello, State, Ack, Error, Motion, Trajectory }

    public sealed class CF1Message
    {
        public CF1MessageKind Kind;
        public uint Boot, LastSequence;
        public bool Armed, LimitsSet, OutputsEnabled;
        public float[] Commands;
        public string Verb, ErrorCode;
        public float MaximumMotionSpeed;
        public uint MinimumPoseSegmentMilliseconds;
    }

    /// <summary>Strict, bounded CF1 firmware response parser. It never opens a port or sends a command.</summary>
    public static class CF1Protocol
    {
        public const int MaximumLineBytes = 192;
        private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;
        private const NumberStyles DecimalStyle = NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint;

        public static string Number(float value)
        {
            if (!RobotHardwareMapping.Finite(value)) throw new ArgumentOutOfRangeException(nameof(value));
            return value.ToString("0.###", Invariant);
        }

        public static bool IsSafeLine(string line)
        {
            if (string.IsNullOrEmpty(line) || line.Length > MaximumLineBytes) return false;
            foreach (char c in line) if (c < 32 || c > 126) return false;
            return true;
        }

        public static bool TryParse(string line, out CF1Message message, out string reason)
        {
            message = null; reason = "Malformed CF1 response.";
            if (!IsSafeLine(line)) { reason = "Non-ASCII, empty, or oversized CF1 response."; return false; }
            string[] tokens = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length == 0) return false;
            CF1Message candidate = new CF1Message();
            switch (tokens[0])
            {
                case "TRAJECTORY":
                    if (tokens.Length != 7 || tokens[1] != "CF1" || !Unsigned(tokens[2], out candidate.Boot) || candidate.Boot == 0 ||
                        tokens[3] != "TPOSE1" || tokens[4] != "40" || tokens[5] != "40" || (tokens[6] != "100" && tokens[6] != "400")) return false;
                    candidate.Kind = CF1MessageKind.Trajectory;
                    candidate.MaximumMotionSpeed = tokens[6] == "400" ? 400f : 100f;
                    break;
                case "MOTION":
                    // Recognize only the motion contract implemented and tested here.
                    // Unknown revisions cannot silently raise the legacy speed ceiling.
                    if (tokens.Length != 4 || tokens[1] != "CF1" || (tokens[2] != "60" && tokens[2] != "100" && tokens[2] != "400") || tokens[3] != "40") return false;
                    candidate.Kind = CF1MessageKind.Motion;
                    candidate.MaximumMotionSpeed = tokens[2] == "400" ? 400f : tokens[2] == "100" ? 100f : 60f; candidate.MinimumPoseSegmentMilliseconds = 40;
                    break;
                case "HELLO":
                    if (tokens.Length != 8 || tokens[1] != "CF1" || !Unsigned(tokens[2], out candidate.Boot) || candidate.Boot == 0 ||
                        !Angles(tokens, 3, out candidate.Commands)) return false;
                    candidate.Kind = CF1MessageKind.Hello; break;
                case "STATE":
                    if (tokens.Length != 11 || !Unsigned(tokens[1], out candidate.Boot) || candidate.Boot == 0 ||
                        !Boolean(tokens[2], out candidate.Armed) || !Boolean(tokens[3], out candidate.LimitsSet) ||
                        !Boolean(tokens[4], out candidate.OutputsEnabled) || !Unsigned(tokens[5], out candidate.LastSequence) ||
                        !Angles(tokens, 6, out candidate.Commands) || (candidate.Armed && !candidate.OutputsEnabled)) return false;
                    candidate.Kind = CF1MessageKind.State; break;
                case "ACK":
                    if (tokens.Length != 3 || !Unsigned(tokens[2], out candidate.LastSequence) ||
                        (tokens[1] != "ARM" && tokens[1] != "HOLD" && tokens[1] != "RELEASE" && tokens[1] != "JOG" && tokens[1] != "LIMITS" && tokens[1] != "POSE" && tokens[1] != "TPOSE")) return false;
                    candidate.Kind = CF1MessageKind.Ack; candidate.Verb = tokens[1]; break;
                case "ERR":
                    if (tokens.Length != 2 || tokens[1].Length > 48) return false;
                    foreach (char c in tokens[1]) if (!(char.IsLetterOrDigit(c) || c == '_')) return false;
                    candidate.Kind = CF1MessageKind.Error; candidate.ErrorCode = tokens[1]; break;
                default: return false;
            }
            message = candidate; reason = null; return true;
        }

        private static bool Unsigned(string text, out uint value) => uint.TryParse(text, NumberStyles.None, Invariant, out value);
        private static bool Boolean(string text, out bool value) { value = text == "1"; return text == "0" || text == "1"; }
        private static bool Angles(string[] tokens, int offset, out float[] angles)
        {
            angles = new float[5];
            for (int i = 0; i < angles.Length; i++)
                if (!float.TryParse(tokens[offset + i], DecimalStyle, Invariant, out angles[i]) ||
                    !RobotHardwareMapping.Finite(angles[i]) || angles[i] < 0f || angles[i] > 180f) { angles = null; return false; }
            return true;
        }
    }
}
