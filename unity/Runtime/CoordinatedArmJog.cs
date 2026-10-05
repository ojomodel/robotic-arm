using System;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Local reach/height jogging with base and wrist reserved for the other stick axes.
    /// Accepts bounded useful progress at workspace edges; never substitutes a remote IK branch.</summary>
    public static class CoordinatedArmJog
    {
        public sealed class Result
        {
            public float[] angles;
            public bool moved, limited;
            public Vector3 requestedWorld, achievedWorld;
        }

        public static Vector3 ReachDirection(ForwardKinematics.Snapshot snapshot, float[] pose)
        {
            snapshot.EvaluateChain(pose, out _, out Vector3[] axes);
            Vector3 up = snapshot.worldToOrigin.inverse.MultiplyVector(Vector3.up).normalized;
            // Positive shoulder rotation leans an upright arm outward. Using its current
            // axis makes reach follow the base yaw, including when yaw and reach are simultaneous.
            return Vector3.Cross(axes[1] * snapshot.directionSigns[1], up).normalized;
        }

        public static Result Step(ForwardKinematics.Snapshot snapshot, float[] seed,
            float reachMillimeters, float heightMillimeters, float maximumJointStep = 5f)
        {
            if (snapshot == null || snapshot.controlledJointCount < 3 || !JointMotion.IsFinite(reachMillimeters) ||
                !JointMotion.IsFinite(heightMillimeters) || !JointMotion.IsFinite(maximumJointStep) || maximumJointStep <= 0)
                throw new ArgumentException("Invalid coordinated arm jog.");
            float[] q = snapshot.Clamp(seed);
            Vector3 start = snapshot.Evaluate(q);
            Vector3 up = snapshot.worldToOrigin.inverse.MultiplyVector(Vector3.up).normalized;
            Vector3 requested = start + (ReachDirection(snapshot, q) * reachMillimeters + up * heightMillimeters) / snapshot.millimetersPerWorldUnit;
            Vector3 wanted = requested - start;
            var result = new Result { angles = q, requestedWorld = requested, achievedWorld = start };
            if (wanted.sqrMagnitude < 1e-16f) return result;
            float error = wanted.sqrMagnitude, initialError = error;
            float lower1 = Mathf.Max(snapshot.minimumAngles[1], q[1] - maximumJointStep);
            float upper1 = Mathf.Min(snapshot.maximumAngles[1], q[1] + maximumJointStep);
            float lower2 = Mathf.Max(snapshot.minimumAngles[2], q[2] - maximumJointStep);
            float upper2 = Mathf.Min(snapshot.maximumAngles[2], q[2] + maximumJointStep);
            float damping = Mathf.Max(.00001f, snapshot.maximumReachWorld * .003f);
            float tolerance = .015f / snapshot.millimetersPerWorldUnit;
            for (int iteration = 0; iteration < 24 && error > tolerance * tolerance; iteration++)
            {
                Vector3 tool = snapshot.EvaluateChain(q, out Vector3[] pivots, out Vector3[] axes);
                Vector3 j1 = Vector3.Cross(axes[1], tool - pivots[1]) * snapshot.directionSigns[1];
                Vector3 j2 = Vector3.Cross(axes[2], tool - pivots[2]) * snapshot.directionSigns[2];
                Vector3 remaining = requested - tool;
                double a = Vector3.Dot(j1, j1) + damping * damping;
                double b = Vector3.Dot(j1, j2);
                double d = Vector3.Dot(j2, j2) + damping * damping;
                double determinant = a * d - b * b;
                float turn1 = 0, turn2 = 0;
                if (determinant > 1e-24)
                {
                    double e1 = Vector3.Dot(j1, remaining), e2 = Vector3.Dot(j2, remaining);
                    turn1 = (float)((d * e1 - b * e2) / determinant) * Mathf.Rad2Deg;
                    turn2 = (float)((a * e2 - b * e1) / determinant) * Mathf.Rad2Deg;
                }
                bool improved = false;
                float scale = Mathf.Min(1, maximumJointStep / Mathf.Max(.0001f, Mathf.Max(Mathf.Abs(turn1), Mathf.Abs(turn2))));
                for (int attempt = 0; attempt < 8; attempt++, scale *= .5f)
                {
                    if (TryImprove(snapshot, q, requested, ref error, turn1 * scale, turn2 * scale,
                        lower1, upper1, lower2, upper2)) { improved = true; break; }
                }
                if (!improved)
                {
                    // At an exactly upright singularity, the first derivative of height is
                    // zero. Try only nearby paired bends, never an alternate posture seed.
                    // Prefer negative elbow (the usual folding direction) for equal choices.
                    float ratio = j1.sqrMagnitude > 1e-14f ? -Vector3.Dot(j1, j2) / j1.sqrMagnitude : 0;
                    foreach (float elbow in new[] { -.5f, .5f, -1.5f, 1.5f, -3f, 3f })
                        if (TryImprove(snapshot, q, requested, ref error, ratio * elbow, elbow,
                            lower1, upper1, lower2, upper2)) { improved = true; break; }
                }
                if (!improved) break;
            }
            Vector3 achieved = snapshot.Evaluate(q);
            Vector3 displacement = achieved - start;
            result.moved = error < initialError - 1e-12f && displacement.sqrMagnitude > 1e-12f && Vector3.Dot(displacement, wanted) > 0;
            result.limited = error > Mathf.Max(tolerance * tolerance, initialError * .04f);
            if (result.moved) { result.angles = q; result.achievedWorld = achieved; }
            else { result.angles = snapshot.Clamp(seed); result.limited = true; }
            return result;
        }

        private static bool TryImprove(ForwardKinematics.Snapshot snapshot, float[] q, Vector3 requested, ref float error,
            float first, float second, float lower1, float upper1, float lower2, float upper2)
        {
            float old1 = q[1], old2 = q[2];
            q[1] = Mathf.Clamp(old1 + first, lower1, upper1); q[2] = Mathf.Clamp(old2 + second, lower2, upper2);
            float candidate = (snapshot.Evaluate(q) - requested).sqrMagnitude;
            if (candidate < error - 1e-13f) { error = candidate; return true; }
            q[1] = old1; q[2] = old2; return false;
        }
    }
}
