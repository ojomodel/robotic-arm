using System;

namespace CareerFair.Robot
{
    /// <summary>Simulation-only, bounded-speed trapezoidal angular motion. Angles are degrees.</summary>
    public static class JointMotion
    {
        public static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

        /// <summary>
        /// Integrates acceleration, cruise and braking events analytically. A changed target can
        /// require braking before reversing; RobotJoint separately enforces its positional limits.
        /// </summary>
        public static void Step(ref float position, ref float velocity, float target,
            float maximumSpeed, float acceleration, float deltaTime)
        {
            if (!IsFinite(position) || !IsFinite(velocity) || !IsFinite(target))
                throw new ArgumentException("Motion state and target must be finite.");
            if (!IsFinite(deltaTime) || deltaTime <= 0f) return;
            if (!IsFinite(maximumSpeed) || !IsFinite(acceleration) || maximumSpeed <= 0f || acceleration <= 0f)
            {
                velocity = 0f;
                return;
            }

            double x = position, v = velocity, remaining = deltaTime;
            double speed = maximumSpeed, a = acceleration;
            const double positionEpsilon = 0.000001;
            const double velocityEpsilon = 0.000001;
            for (int iteration = 0; iteration < 48 && remaining > 0.000000001; iteration++)
            {
                double difference = target - x;
                if (Math.Abs(difference) < positionEpsilon && Math.Abs(v) < velocityEpsilon)
                {
                    x = target; v = 0.0; break;
                }
                double direction = difference >= 0.0 ? 1.0 : -1.0;
                double distance = Math.Abs(difference), forwardSpeed = v * direction;
                double accelerationSigned, eventTime, eventVelocity;
                if (forwardSpeed < -velocityEpsilon)
                {
                    // First brake motion away from a newly assigned target.
                    accelerationSigned = direction * a;
                    eventTime = -forwardSpeed / a;
                    eventVelocity = 0.0;
                }
                else if (forwardSpeed * forwardSpeed / (2.0 * a) >= distance - positionEpsilon && forwardSpeed > velocityEpsilon)
                {
                    accelerationSigned = -direction * a;
                    eventTime = forwardSpeed / a;
                    eventVelocity = 0.0;
                }
                else if (forwardSpeed > speed + velocityEpsilon)
                {
                    // A reduced speed setting also decelerates instead of changing velocity abruptly.
                    accelerationSigned = -direction * a;
                    eventTime = (forwardSpeed - speed) / a;
                    eventVelocity = direction * speed;
                }
                else
                {
                    double peakSpeed = Math.Sqrt(Math.Max(0.0, a * distance + forwardSpeed * forwardSpeed / 2.0));
                    double nextSpeed = Math.Min(speed, peakSpeed);
                    if (nextSpeed > forwardSpeed + velocityEpsilon)
                    {
                        accelerationSigned = direction * a;
                        eventTime = (nextSpeed - forwardSpeed) / a;
                        eventVelocity = direction * nextSpeed;
                    }
                    else if (peakSpeed > speed + velocityEpsilon && forwardSpeed > velocityEpsilon)
                    {
                        accelerationSigned = 0.0;
                        eventTime = (distance - forwardSpeed * forwardSpeed / (2.0 * a)) / forwardSpeed;
                        eventVelocity = v;
                    }
                    else
                    {
                        accelerationSigned = -direction * a;
                        eventTime = Math.Max(0.0, forwardSpeed) / a;
                        eventVelocity = 0.0;
                    }
                }
                if (eventTime <= 0.000000001)
                {
                    if (Math.Abs(difference) < 0.00001) { x = target; v = 0.0; break; }
                    v = eventVelocity;
                    continue;
                }
                double step = Math.Min(remaining, eventTime);
                x += v * step + 0.5 * accelerationSigned * step * step;
                v += accelerationSigned * step;
                remaining -= step;
                if (step >= eventTime - 0.000000001) v = eventVelocity;
            }
            if (Math.Abs(target - x) < positionEpsilon && Math.Abs(v) < velocityEpsilon) { x = target; v = 0.0; }
            position = (float)x;
            velocity = (float)v;
        }

        /// <summary>Editor-callable numerical checks; throws on failure and requires no scene.</summary>
        public static string RunSelfTests()
        {
            float angle = 0f, velocity = 0f, peak = 0f, nearFinish = 0f;
            for (int i = 0; i < 600; i++)
            {
                float previousVelocity = velocity;
                Step(ref angle, ref velocity, 90f, 45f, 90f, 1f / 120f);
                Require(angle >= -0.0001f && angle <= 90.0001f, "A planned move overshot its target.");
                Require(Math.Abs(velocity) <= 45.0001f, "Maximum speed was exceeded.");
                Require(Math.Abs(velocity - previousVelocity) <= 0.7502f, "Acceleration bound was exceeded.");
                peak = Math.Max(peak, velocity);
                if (angle > 88f) nearFinish = Math.Max(nearFinish, velocity);
            }
            Require(Math.Abs(angle - 90f) < 0.0001f && Math.Abs(velocity) < 0.0001f, "Move did not finish.");
            Require(peak > 44.9f && nearFinish < peak, "Expected cruise and deceleration were absent.");
            float coarse = 0f, coarseVelocity = 0f, fine = 0f, fineVelocity = 0f;
            Step(ref coarse, ref coarseVelocity, 100f, 40f, 80f, 1.25f);
            for (int i = 0; i < 150; i++) Step(ref fine, ref fineVelocity, 100f, 40f, 80f, 1f / 120f);
            Require(Math.Abs(coarse - fine) < 0.001f && Math.Abs(coarseVelocity - fineVelocity) < 0.001f,
                "Different tick sizes disagree.");
            angle = 20f; velocity = 30f;
            for (int i = 0; i < 600; i++) Step(ref angle, ref velocity, -25f, 45f, 90f, 1f / 120f);
            Require(Math.Abs(angle + 25f) < 0.0001f && Math.Abs(velocity) < 0.0001f, "Reversal did not settle.");
            float unchanged = angle;
            Step(ref angle, ref velocity, 50f, 45f, 90f, 0f);
            Require(angle == unchanged, "A zero-duration step moved.");
            return "JointMotion: target arrival, speed, acceleration, braking, tick-size independence and reversal passed.";
        }

        private static void Require(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }
    }
}
