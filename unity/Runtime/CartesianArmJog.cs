using System;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Small Cartesian steps in the fixed robot/table frame. Solves base,
    /// shoulder and elbow locally; wrist/gripper remain explicit operator controls.</summary>
    public static class CartesianArmJog
    {
        public sealed class Result
        {
            public float[] angles;
            public bool moved, limited;
            public Vector3 requestedWorld, achievedWorld;
        }

        public static Result Step(ForwardKinematics.Snapshot snapshot, float[] seed,
            Vector3 tableMillimeters, float maximumJointStep = 5f)
        {
            // If a joint is still catching up, slow the Cartesian request together.
            // Projecting only onto the other joints would make a straight stick
            // gesture wander sideways at the per-joint target-lead boundary.
            Result first = null;
            for (int attempt = 0; attempt < 7; attempt++)
            {
                Result candidate = TryStep(snapshot, seed, tableMillimeters * Mathf.Pow(.5f, attempt), maximumJointStep);
                if (first == null) first = candidate;
                Vector3 start = snapshot.Evaluate(snapshot.Clamp(seed));
                Vector3 wanted = candidate.requestedWorld - start;
                Vector3 delta = candidate.achievedWorld - start;
                float along = Vector3.Dot(delta, wanted.normalized);
                float sideways = (delta - wanted.normalized * along).magnitude;
                float tolerance = .015f / snapshot.millimetersPerWorldUnit;
                if (candidate.moved && sideways <= Mathf.Max(tolerance, along * .1f))
                {
                    candidate.limited |= attempt > 0;
                    return candidate;
                }
            }
            first.angles = snapshot.Clamp(seed); first.moved = false; first.limited = true;
            first.achievedWorld = snapshot.Evaluate(first.angles);
            return first;
        }

        private static Result TryStep(ForwardKinematics.Snapshot snapshot, float[] seed,
            Vector3 tableMillimeters, float maximumJointStep)
        {
            if (snapshot == null || snapshot.controlledJointCount < 3 ||
                !JointMotion.IsFinite(tableMillimeters.x) || !JointMotion.IsFinite(tableMillimeters.y) ||
                !JointMotion.IsFinite(tableMillimeters.z) || !JointMotion.IsFinite(maximumJointStep) || maximumJointStep <= 0)
                throw new ArgumentException("Invalid tabletop jog.");
            float[] q = snapshot.Clamp(seed);
            Vector3 start = snapshot.Evaluate(q);
            Vector3 wanted = snapshot.worldToOrigin.inverse.MultiplyVector(tableMillimeters) / snapshot.millimetersPerUnityUnit;
            Vector3 target = start + wanted;
            var result = new Result { angles = q, requestedWorld = target, achievedWorld = start };
            if (wanted.sqrMagnitude < 1e-16f) return result;
            float initialError = wanted.sqrMagnitude, error = initialError;
            var lower = new float[3]; var upper = new float[3];
            for (int j = 0; j < 3; j++)
            {
                lower[j] = Mathf.Max(snapshot.minimumAngles[j], q[j] - maximumJointStep);
                upper[j] = Mathf.Min(snapshot.maximumAngles[j], q[j] + maximumJointStep);
            }
            float damping = Mathf.Max(.00001f, snapshot.maximumReachWorld * .003f);
            float tolerance = .015f / snapshot.millimetersPerWorldUnit;
            for (int iteration = 0; iteration < 24 && error > tolerance * tolerance; iteration++)
            {
                Vector3 tool = snapshot.EvaluateChain(q, out Vector3[] pivots, out Vector3[] axes);
                var jacobian = new Vector3[3];
                for (int j = 0; j < 3; j++)
                    jacobian[j] = Vector3.Cross(axes[j], tool - pivots[j]) * snapshot.directionSigns[j];
                Vector3 turn = Solve(jacobian, target - tool, damping) * Mathf.Rad2Deg;
                float largest = Mathf.Max(Mathf.Abs(turn.x), Mathf.Max(Mathf.Abs(turn.y), Mathf.Abs(turn.z)));
                float scale = Mathf.Min(1f, maximumJointStep / Mathf.Max(.0001f, largest));
                bool improved = false;
                for (int attempt = 0; attempt < 8; attempt++, scale *= .5f)
                    if (TryImprove(snapshot, q, target, ref error, turn * scale, lower, upper))
                    { improved = true; break; }
                if (!improved)
                {
                    // Height has zero first derivative at full extension. A small
                    // nearby paired bend can leave that singularity without a jump.
                    float ratio = jacobian[1].sqrMagnitude > 1e-14f ?
                        -Vector3.Dot(jacobian[1], jacobian[2]) / jacobian[1].sqrMagnitude : 0f;
                    foreach (float elbow in new[] { -.5f, .5f, -1.5f, 1.5f, -3f, 3f })
                        if (TryImprove(snapshot, q, target, ref error, new Vector3(0, ratio * elbow, elbow), lower, upper))
                        { improved = true; break; }
                }
                if (!improved) break;
            }
            Vector3 achieved = snapshot.Evaluate(q), displacement = achieved - start;
            result.moved = error < initialError - 1e-12f && displacement.sqrMagnitude > 1e-12f && Vector3.Dot(displacement, wanted) > 0;
            result.limited = error > Mathf.Max(tolerance * tolerance, initialError * .04f);
            if (result.moved) { result.angles = q; result.achievedWorld = achieved; }
            else { result.angles = snapshot.Clamp(seed); result.limited = true; }
            return result;
        }

        private static Vector3 Solve(Vector3[] j, Vector3 error, float damping)
        {
            // Cholesky solve of (J'J + lambda^2 I) dq = J' error.
            // Positive damping keeps the system defined at straight-arm singularities.
            double a = Vector3.Dot(j[0], j[0]) + (double)damping * damping;
            double b = Vector3.Dot(j[0], j[1]), c = Vector3.Dot(j[0], j[2]);
            double d = Vector3.Dot(j[1], j[1]) + (double)damping * damping;
            double e = Vector3.Dot(j[1], j[2]);
            double f = Vector3.Dot(j[2], j[2]) + (double)damping * damping;
            double l00 = Math.Sqrt(a), l10 = b / l00, l20 = c / l00;
            double l11 = Math.Sqrt(Math.Max(1e-24, d - l10 * l10));
            double l21 = (e - l20 * l10) / l11;
            double l22 = Math.Sqrt(Math.Max(1e-24, f - l20 * l20 - l21 * l21));
            double y0 = Vector3.Dot(j[0], error) / l00;
            double y1 = (Vector3.Dot(j[1], error) - l10 * y0) / l11;
            double y2 = (Vector3.Dot(j[2], error) - l20 * y0 - l21 * y1) / l22;
            double x2 = y2 / l22, x1 = (y1 - l21 * x2) / l11, x0 = (y0 - l10 * x1 - l20 * x2) / l00;
            return new Vector3((float)x0, (float)x1, (float)x2);
        }

        private static bool TryImprove(ForwardKinematics.Snapshot snapshot, float[] q, Vector3 target,
            ref float error, Vector3 step, float[] lower, float[] upper)
        {
            Vector3 old = new Vector3(q[0], q[1], q[2]);
            for (int j = 0; j < 3; j++) q[j] = Mathf.Clamp(q[j] + step[j], lower[j], upper[j]);
            float candidate = (snapshot.Evaluate(q) - target).sqrMagnitude;
            if (candidate < error - 1e-13f) { error = candidate; return true; }
            for (int j = 0; j < 3; j++) q[j] = old[j];
            return false;
        }
    }
}
