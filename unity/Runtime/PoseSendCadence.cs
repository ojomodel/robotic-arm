using System;

namespace CareerFair.Robot.Hardware
{
    /// <summary>Phase-preserving 50 Hz latest-pose schedule. A delayed frame sends once, never a backlog.</summary>
    public sealed class PoseSendCadence
    {
        public const double IntervalSeconds = .02;
        private double nextDeadline = double.NaN;

        public void Reset() { nextDeadline = double.NaN; }

        public bool TryTake(double now)
        {
            if (double.IsNaN(now) || double.IsInfinity(now) || now < 0) return false;
            if (double.IsNaN(nextDeadline)) { nextDeadline = now + IntervalSeconds; return true; }
            if (now + 1e-9 < nextDeadline) return false;
            double missed = Math.Floor((now + 1e-9 - nextDeadline) / IntervalSeconds) + 1;
            nextDeadline += missed * IntervalSeconds;
            return true;
        }
    }
}
