using System;
using UnityEngine;

namespace CareerFair.Robot
{
    public enum IkStatus { Reachable, OutsideGeometricBound, NoConvergence, InvalidInput }

    [Serializable]
    public sealed class IkResult
    {
        public IkStatus status;
        public string message;
        public float[] angles;
        public float residualMillimeters;
        public int iterations;
        public Vector3 targetWorld, achievedWorld;
        public bool reachable => status == IkStatus.Reachable;
    }

    [DisallowMultipleComponent]
    public sealed class InverseKinematics : MonoBehaviour
    {
        public RobotArm arm;
        [Min(0.05f)] public float toleranceMillimeters = 0.5f;
        [Range(20, 500)] public int maximumIterations = 220;
        [Range(1, 24)] public int maximumStarts = 12;
        [Range(1f, 90f)] public float maximumStepDegrees = 35f;
        [Min(0.02f)] public float followIntervalSeconds = 0.1f;
        [SerializeField] private bool followTarget;
        public bool FollowTarget => followTarget;
        public bool RoutineStopInProgress { get; private set; }
        private static RobotArm routineStopArm;
        public static bool IsRoutineStopFor(RobotArm candidate) => candidate != null && routineStopArm == candidate;
        public IkResult LastResult { get; private set; }
        private RobotArm subscribedArm;
        private bool issuingOwnCommand;
        private float followTimer;

        private void OnEnable() { Subscribe(); }
        private void OnDisable() { Cancel(); Unsubscribe(); }
        private void Update() { Tick(Time.deltaTime); }

        public IkResult SolveTarget()
        {
            if (arm == null || arm.target == null) return LastResult = Invalid("Assign the arm and target.");
            return SolveWorld(arm.target.position);
        }

        public IkResult SolveWorld(Vector3 targetWorld, float[] seed = null)
        {
            Subscribe();
            try { return LastResult = Solve(ForwardKinematics.Capture(arm), targetWorld, seed, toleranceMillimeters, maximumIterations, maximumStarts, maximumStepDegrees); }
            catch (ArgumentException exception) { return LastResult = Invalid(exception.Message); }
        }

        public static IkResult Solve(ForwardKinematics.Snapshot snapshot, Vector3 targetWorld, float[] seed = null,
            float toleranceMillimeters = 0.5f, int maximumIterations = 220, int maximumStarts = 12, float maximumStepDegrees = 35f)
        {
            if (snapshot == null || !ForwardKinematics.IsFinite(targetWorld) || !JointMotion.IsFinite(toleranceMillimeters) || toleranceMillimeters <= 0f ||
                maximumIterations < 1 || maximumStarts < 1 || !JointMotion.IsFinite(maximumStepDegrees) || maximumStepDegrees <= 0f)
                return Invalid("Target and solver settings must be finite and valid.");
            float[] initial;
            try { initial = snapshot.Clamp(seed ?? snapshot.baseAngles); }
            catch (ArgumentException exception) { return Invalid(exception.Message); }
            float tolerance = toleranceMillimeters / snapshot.millimetersPerWorldUnit;
            float[] best = (float[])initial.Clone();
            Vector3 bestTool = snapshot.Evaluate(best);
            float bestError = Vector3.Distance(bestTool, targetWorld);
            if (snapshot.hasSerialReachBound && Vector3.Distance(targetWorld, snapshot.pivots[0]) > snapshot.maximumReachWorld + tolerance)
                return Result(IkStatus.OutsideGeometricBound, "Outside the CAD-derived maximum geometric reach bound.", best, bestTool, targetWorld, bestError, snapshot, 0);
            int iterations = 0;
            int[] primes = { 2, 3, 5, 7 };
            for (int start = 0; start < Mathf.Min(maximumStarts, 24); start++)
            {
                float[] q = (float[])initial.Clone();
                for (int i = 0; i < snapshot.controlledJointCount; i++)
                    if (start == 1) q[i] = Mathf.Clamp(snapshot.homeAngles[i], snapshot.minimumAngles[i], snapshot.maximumAngles[i]);
                    else if (start == 2) q[i] = (snapshot.minimumAngles[i] + snapshot.maximumAngles[i]) * 0.5f;
                    else if (start > 2) q[i] = Mathf.Lerp(snapshot.minimumAngles[i], snapshot.maximumAngles[i], Halton(start - 2, primes[i % primes.Length]));
                int stagnant = 0;
                float previousError = float.PositiveInfinity;
                for (int iteration = 0; iteration < Mathf.Min(maximumIterations, 500); iteration++)
                {
                    iterations++;
                    Vector3 tool = snapshot.Evaluate(q);
                    float error = Vector3.Distance(tool, targetWorld);
                    if (error < bestError) { bestError = error; best = (float[])q.Clone(); bestTool = tool; }
                    if (error <= tolerance)
                        return Result(IkStatus.Reachable, "Position solved within tolerance; orientation and collision clearance are not constrained.", q, tool, targetWorld, error, snapshot, iterations);
                    stagnant = Mathf.Abs(previousError - error) < tolerance * 0.0005f ? stagnant + 1 : 0;
                    previousError = error;
                    if (stagnant > 30) break;
                    for (int i = snapshot.controlledJointCount - 1; i >= 0; i--)
                    {
                        if (!snapshot.affectsTool[i]) continue;
                        Vector3[] pivots, axes;
                        tool = snapshot.EvaluateChain(q, out pivots, out axes);
                        Vector3 toTool = Vector3.ProjectOnPlane(tool - pivots[i], axes[i]);
                        Vector3 toTarget = Vector3.ProjectOnPlane(targetWorld - pivots[i], axes[i]);
                        if (toTool.sqrMagnitude < 1e-14f || toTarget.sqrMagnitude < 1e-14f) continue;
                        float turn = Vector3.SignedAngle(toTool, toTarget, axes[i]) / snapshot.directionSigns[i];
                        q[i] = Mathf.Clamp(q[i] + Mathf.Clamp(turn, -maximumStepDegrees, maximumStepDegrees), snapshot.minimumAngles[i], snapshot.maximumAngles[i]);
                    }
                }
            }
            return Result(IkStatus.NoConvergence, "No solution found within the bounded search. This is not proof that the target is unreachable.", best, bestTool, targetWorld, bestError, snapshot, iterations);
        }

        public bool MoveToSolution()
        {
            SetFollowTarget(false);
            if (arm == null || LastResult == null || !LastResult.reachable) return false;
            if (arm.target != null && arm.robotOrigin != null && arm.robotOrigin.InverseTransformVector(arm.target.position - LastResult.targetWorld).magnitude * arm.millimetersPerUnityUnit > toleranceMillimeters)
            { LastResult.message = "Target changed; solve again before moving."; return false; }
            // Re-capture edited axes/limits and the current gripper. A preview is not a stale command
            // that may silently reopen the gripper or apply obsolete calibration fields.
            Vector3 target = LastResult.targetWorld;
            float[] seed = LastResult.angles;
            IkResult fresh = SolveWorld(target, seed);
            if (!fresh.reachable) return false;
            CommandPose(fresh.angles);
            return true;
        }

        /// <summary>
        /// Local damped Jacobian solve for successive path waypoints. No random restarts or
        /// alternative posture seeds: inability to continue rejects the complete path.
        /// </summary>
        public static IkResult SolveContinuous(ForwardKinematics.Snapshot snapshot, Vector3 targetWorld, float[] seed,
            float toleranceMillimeters = 0.2f, int maximumIterations = 240)
        {
            if (snapshot == null || !ForwardKinematics.IsFinite(targetWorld) || !JointMotion.IsFinite(toleranceMillimeters) || toleranceMillimeters <= 0f)
                return Invalid("Invalid continuous IK input.");
            float[] q;
            try { q = snapshot.Clamp(seed); } catch (ArgumentException exception) { return Invalid(exception.Message); }
            float tolerance = toleranceMillimeters / snapshot.millimetersPerWorldUnit;
            float damping = Mathf.Max(0.00001f, snapshot.maximumReachWorld * 0.005f);
            Vector3 tool = snapshot.Evaluate(q);
            float error = Vector3.Distance(tool, targetWorld);
            int iterations = 0;
            for (; iterations < Mathf.Clamp(maximumIterations, 1, 500); iterations++)
            {
                if (error <= tolerance)
                    return Result(IkStatus.Reachable, "Local continuous waypoint solved without changing IK branch seed.", q, tool, targetWorld, error, snapshot, iterations);
                Vector3[] pivots, axes;
                tool = snapshot.EvaluateChain(q, out pivots, out axes);
                Vector3[] jacobian = new Vector3[snapshot.controlledJointCount];
                for (int j = 0; j < jacobian.Length; j++)
                    jacobian[j] = snapshot.affectsTool[j] ? Vector3.Cross(axes[j], tool - pivots[j]) * snapshot.directionSigns[j] : Vector3.zero;
                bool accepted = false;
                for (int retry = 0; retry < 10 && !accepted; retry++)
                {
                    // J is metres/radian. Solve (J J^T + lambda^2 I) w = positional error,
                    // then use the minimum-norm angular increment J^T w.
                    double aa = damping * damping, dd = aa, ff = aa, bb = 0, cc = 0, ee = 0;
                    foreach (Vector3 column in jacobian)
                    {
                        aa += column.x * column.x; bb += column.x * column.y; cc += column.x * column.z;
                        dd += column.y * column.y; ee += column.y * column.z; ff += column.z * column.z;
                    }
                    double determinant = aa * (dd * ff - ee * ee) - bb * (bb * ff - cc * ee) + cc * (bb * ee - cc * dd);
                    if (Math.Abs(determinant) < 1e-30) { damping *= 4f; continue; }
                    Vector3 delta = targetWorld - tool;
                    Vector3 w = new Vector3(
                        (float)(((dd * ff - ee * ee) * delta.x + (cc * ee - bb * ff) * delta.y + (bb * ee - cc * dd) * delta.z) / determinant),
                        (float)(((cc * ee - bb * ff) * delta.x + (aa * ff - cc * cc) * delta.y + (bb * cc - aa * ee) * delta.z) / determinant),
                        (float)(((bb * ee - cc * dd) * delta.x + (bb * cc - aa * ee) * delta.y + (aa * dd - bb * bb) * delta.z) / determinant));
                    float[] increment = new float[jacobian.Length]; float largest = 0f;
                    for (int j = 0; j < increment.Length; j++)
                    { increment[j] = Vector3.Dot(jacobian[j], w) * Mathf.Rad2Deg; largest = Mathf.Max(largest, Mathf.Abs(increment[j])); }
                    float scale = largest > 5f ? 5f / largest : 1f;
                    for (int search = 0; search < 8; search++, scale *= 0.5f)
                    {
                        float[] candidate = (float[])q.Clone();
                        for (int j = 0; j < increment.Length; j++)
                            candidate[j] = Mathf.Clamp(q[j] + increment[j] * scale, snapshot.minimumAngles[j], snapshot.maximumAngles[j]);
                        Vector3 nextTool = snapshot.Evaluate(candidate);
                        float nextError = Vector3.Distance(nextTool, targetWorld);
                        if (nextError < error - 1e-9f || nextError <= tolerance)
                        {
                            q = candidate; tool = nextTool; error = nextError; accepted = true;
                            damping = Mathf.Max(snapshot.maximumReachWorld * 0.00001f, damping * 0.7f);
                            break;
                        }
                    }
                    if (!accepted) damping *= 4f;
                }
                if (!accepted) break;
            }
            return Result(IkStatus.NoConvergence, "Local waypoint continuation could not converge; no alternative IK branch was substituted.", q, tool, targetWorld, error, snapshot, iterations);
        }

        public void SetFollowTarget(bool enabled)
        {
            Subscribe(); followTarget = enabled; followTimer = 0f;
        }

        public void Cancel() { followTarget = false; followTimer = 0f; }

        // Switching input modes or rejecting a target stops interpolation, but is
        // not the user's STOP command. Hardware keeps following the held pose.
        public void StopForTargetControl(Action stopOtherMotion = null)
        {
            bool previousRoutine = RoutineStopInProgress, previousOwn = issuingOwnCommand;
            RobotArm previousArm = routineStopArm;
            RoutineStopInProgress = issuingOwnCommand = true;
            routineStopArm = arm;
            try { stopOtherMotion?.Invoke(); if (arm != null) arm.Stop(); }
            finally { routineStopArm = previousArm; RoutineStopInProgress = previousRoutine; issuingOwnCommand = previousOwn; }
        }

        public void Tick(float deltaTime)
        {
            Subscribe();
            if (!followTarget || arm == null || arm.target == null || !JointMotion.IsFinite(deltaTime) || deltaTime < 0f) return;
            followTimer -= deltaTime;
            if (followTimer > 0f) return;
            followTimer = Mathf.Max(0.02f, followIntervalSeconds);
            var hardware = arm.GetComponent<RobotHardwareLink>();
            IkResult result = hardware != null && hardware.FollowActive ? SolveFollowWaypoint() : SolveTarget();
            if (result.reachable) CommandPose(result.angles);
            else StopForTargetControl();
        }

        private IkResult SolveFollowWaypoint()
        {
            try
            {
                var snapshot = ForwardKinematics.Capture(arm);
                float[] seed = arm.CurrentAngles();
                Vector3 start = snapshot.Evaluate(seed);
                Vector3 step = Vector3.ClampMagnitude(arm.target.position - start, 3.5f / snapshot.millimetersPerWorldUnit);
                IkResult result = null;
                for (int attempt = 0; attempt < 3; attempt++)
                {
                    result = SolveContinuous(snapshot, start + step, seed, 0.05f, 80);
                    bool continuous = result.reachable;
                    if (continuous)
                        for (int i = 0; i < snapshot.controlledJointCount; i++)
                            continuous &= Mathf.Abs(result.angles[i] - seed[i]) <= 5f;
                    if (continuous)
                    {
                        result.message = "Following a local step toward the ball; final target and collision clearance are not verified.";
                        return LastResult = result;
                    }
                    step *= .5f;
                }
                result.status = IkStatus.NoConvergence;
                result.message = "Ball movement paused at a reach or continuity limit. Drag the ball back toward the tool to resume.";
                return LastResult = result;
            }
            catch (ArgumentException exception) { return LastResult = Invalid(exception.Message); }
        }

        private void CommandPose(float[] angles)
        {
            issuingOwnCommand = true;
            try { arm.MoveToPose(angles); } finally { issuingOwnCommand = false; }
        }

        private void ExternalCommand() { if (!issuingOwnCommand) Cancel(); }
        private void Subscribe()
        {
            if (subscribedArm == arm) return;
            Unsubscribe(); subscribedArm = arm;
            if (subscribedArm == null) return;
            subscribedArm.SimulationStopped += ExternalCommand; subscribedArm.MotionCommandIssued += ExternalCommand;
        }
        private void Unsubscribe()
        {
            if (subscribedArm == null) return;
            subscribedArm.SimulationStopped -= ExternalCommand; subscribedArm.MotionCommandIssued -= ExternalCommand; subscribedArm = null;
        }
        private static float Halton(int number, int basis)
        {
            float result = 0f, fraction = 1f;
            while (number > 0) { fraction /= basis; result += fraction * (number % basis); number /= basis; }
            return result;
        }
        private static IkResult Invalid(string message) => new IkResult { status = IkStatus.InvalidInput, message = message, angles = Array.Empty<float>(), residualMillimeters = float.PositiveInfinity };
        private static IkResult Result(IkStatus status, string message, float[] angles, Vector3 tool, Vector3 target, float error, ForwardKinematics.Snapshot snapshot, int iterations)
            => new IkResult { status = status, message = message, angles = (float[])angles.Clone(), achievedWorld = tool, targetWorld = target,
                residualMillimeters = error * snapshot.millimetersPerWorldUnit, iterations = iterations };
    }
}
