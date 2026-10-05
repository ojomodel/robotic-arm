using System;

namespace CareerFair.Robot.Hardware
{
    /// <summary>Temporary operator execution bounds, not a physically verified calibration.
    /// Snapshots the confirmed direction/reference and allows neutral +/-90 command degrees,
    /// clipped at servo 0..180. Model angles retain the saved home and direction mapping.
    /// Never changes or serializes the source mapping's ranges or verification flags.</summary>
    public sealed class DirectOperatorTravel
    {
        private const float RoundoffTolerance = .0001f;
        private readonly float[] neutral, home, commandMinimum, commandMaximum, modelMinimum, modelMaximum;
        private readonly int[] directions;
        private DirectOperatorTravel(RobotHardwareMapping source, float[] low, float[] high)
        {
            neutral = (float[])source.servoNeutral.Clone(); home = (float[])source.modelHome.Clone();
            directions = (int[])source.directions.Clone(); commandMinimum = low; commandMaximum = high;
            modelMinimum = new float[5]; modelMaximum = new float[5];
            for (int i = 0; i < 5; i++)
            {
                float a = home[i] + directions[i] * (low[i] - neutral[i]);
                float b = home[i] + directions[i] * (high[i] - neutral[i]);
                modelMinimum[i] = Math.Min(a,b); modelMaximum[i] = Math.Max(a,b);
            }
        }
        public float MinimumCommand(int channel) => commandMinimum[channel];
        public float MaximumCommand(int channel) => commandMaximum[channel];
        public float MinimumModelAngle(int channel) => modelMinimum[channel];
        public float MaximumModelAngle(int channel) => modelMaximum[channel];

        public static bool TryCreate(RobotHardwareMapping source, float[] cadMinimum, float[] cadMaximum,
            float[] currentCommands, out DirectOperatorTravel execution, out float[] currentModelPose, out string reason)
        {
            execution = null; currentModelPose = null;
            if (source == null) return Fail("A direction/reference mapping is required.", out reason);
            if (!source.ValidateStructure(out reason)) return false;
            if (cadMinimum == null || cadMaximum == null || cadMinimum.Length != 5 || cadMaximum.Length != 5)
                return Fail("Five original CAD limits are required.", out reason);
            var low = new float[5]; var high = new float[5];
            for (int i = 0; i < 5; i++)
            {
                if (!source.directionVerified[i] || Math.Abs(source.directions[i]) != 1)
                    return Fail("Movement direction is unconfirmed on channel " + i + ".", out reason);
                if (!RobotHardwareMapping.Finite(cadMinimum[i]) || !RobotHardwareMapping.Finite(cadMaximum[i]) || cadMinimum[i] > cadMaximum[i])
                    return Fail("Invalid original CAD travel on channel " + i + ".", out reason);
                // Decimal arithmetic preserves the entered command reference (for example,
                // 90.8f must produce 0.8, not 0.801 after binary float rounding).
                decimal lower = Math.Max(0m, (decimal)source.servoNeutral[i] - 90m);
                decimal upper = Math.Min(180m, (decimal)source.servoNeutral[i] + 90m);
                // CF1 has three decimal places. Round inward; never wrap an endpoint
                // through 180 to zero or silently reverse the requested direction.
                low[i] = (float)(Math.Ceiling(lower * 1000m) / 1000m);
                high[i] = (float)(Math.Floor(upper * 1000m) / 1000m);
                if (low[i] > high[i]) return Fail("No representable direct travel on channel " + i + ".", out reason);
            }
            var candidate = new DirectOperatorTravel(source, low, high);
            if (!candidate.TryMapTelemetry(currentCommands, out currentModelPose, out reason)) return false;
            execution = candidate; return true;
        }

        public bool TryMapTelemetry(float[] commands, out float[] pose, out string reason)
        {
            pose = null;
            if (commands == null || commands.Length != 5) return Fail("Five current servo commands are required.", out reason);
            var candidate = new float[5];
            for (int i = 0; i < 5; i++)
            {
                if (!RobotHardwareMapping.Finite(commands[i]) || commands[i] < commandMinimum[i] || commands[i] > commandMaximum[i])
                    return Fail("Current command is outside direct neutral-relative travel on channel " + i + "; no move was requested.", out reason);
                candidate[i] = home[i] + directions[i] * (commands[i] - neutral[i]);
            }
            pose = candidate; reason = null; return true;
        }

        public bool TryMapPose(float[] pose, out float[] commands, out string reason)
        {
            commands = null;
            if (pose == null || pose.Length != 5) return Fail("Five model angles are required.", out reason);
            var candidate = new float[5];
            for (int i = 0; i < 5; i++)
            {
                float command = neutral[i] + directions[i] * (pose[i] - home[i]);
                if (!RobotHardwareMapping.Finite(pose[i]) || !RobotHardwareMapping.Finite(command) ||
                    command < commandMinimum[i] - RoundoffTolerance || command > commandMaximum[i] + RoundoffTolerance)
                    return Fail("Direct operator travel limit reached on channel " + i + ".", out reason);
                candidate[i] = Math.Max(commandMinimum[i],Math.Min(commandMaximum[i],command));
            }
            commands = candidate; reason = null; return true;
        }
        private static bool Fail(string message, out string reason) { reason = message; return false; }
    }
}
