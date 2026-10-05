using System;

namespace CareerFair.Robot.Hardware
{
    // Source-sample timestamps are captured on Unity's main thread, before the
    // USB worker queue. They never describe packet arrival or transmission time.
    public sealed class MotionSampleClock
    {
        public const ulong MaximumMicroseconds = 86400000000UL;
        private double epoch = double.NaN;
        private ulong previous;
        public void Reset() { epoch = double.NaN; previous = 0; }
        public bool TryStamp(double sampleSeconds, out ulong microseconds)
        {
            microseconds = 0;
            if (double.IsNaN(sampleSeconds) || double.IsInfinity(sampleSeconds) || sampleSeconds < 0) return false;
            if (double.IsNaN(epoch)) { epoch = sampleSeconds; previous = 0; return true; }
            double delta = Math.Round((sampleSeconds - epoch) * 1000000.0);
            if (delta <= previous || delta > MaximumMicroseconds || delta - previous > 500000) return false;
            microseconds = (ulong)delta; previous = microseconds; return true;
        }
    }
}
