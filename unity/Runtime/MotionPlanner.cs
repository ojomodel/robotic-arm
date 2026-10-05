using System;
using System.Collections.Generic;
using UnityEngine;

namespace CareerFair.Robot
{
    public enum MotionPlanMode { JointSpace, Cartesian }

    [Serializable]
    public sealed class MotionPlan
    {
        public bool valid;
        public MotionPlanMode mode;
        public string label, message;
        public Vector3[] worldPoints = Array.Empty<Vector3>();
        [NonSerialized] public float[][] jointPoses = Array.Empty<float[]>();
        public float[] sampleTimes = Array.Empty<float>();
        public float durationSeconds, maxCartesianResidualMillimeters;
        public int validationSamples;
        internal float[] speeds, accelerations;
        internal float speedMultiplier;
        internal ForwardKinematics.Snapshot snapshot;
    }

    [DefaultExecutionOrder(50)]
    [DisallowMultipleComponent]
    public sealed class MotionPlanner : MonoBehaviour
    {
        public RobotArm arm;
        public InverseKinematics inverseKinematics;
        [Min(0.1f)] public float cartesianToleranceMillimeters = 0.75f;
        public MotionPlan LastPlan { get; private set; }
        public bool IsPlaying { get; private set; }
        public string Status { get; private set; } = "No preview created.";
        private RobotArm subscribedArm;
        private bool issuingOwnCommand;
        private float playTime;
        private int playSegment;

        private void OnEnable() { Subscribe(); }
        private void OnDisable() { CancelPlayback(); Unsubscribe(); }
        private void Update() { Tick(Time.deltaTime); }

        public MotionPlan PreviewJointSpace(float[] targetAngles)
        {
            CancelPlayback(); Subscribe();
            try
            {
                RequireStopped();
                ForwardKinematics.Snapshot snapshot = ForwardKinematics.Capture(arm);
                snapshot.ValidateAngles(targetAngles);
                MotionPlan plan = NewPlan(MotionPlanMode.JointSpace, "Joint space - actual accelerated joint model");
                float[] q = arm.CurrentAngles(), velocities = new float[q.Length];
                List<float[]> poses = new List<float[]> { (float[])q.Clone() };
                List<Vector3> points = new List<Vector3> { snapshot.Evaluate(q) };
                List<float> times = new List<float> { 0f };
                const float step = 1f / 60f;
                bool done = false;
                for (int iteration = 0; iteration < 7200; iteration++)
                {
                    done = true;
                    for (int j = 0; j < q.Length; j++)
                    {
                        JointMotion.Step(ref q[j], ref velocities[j], targetAngles[j], plan.speeds[j] * plan.speedMultiplier,
                            plan.accelerations[j] * plan.speedMultiplier * plan.speedMultiplier, step);
                        q[j] = Mathf.Clamp(q[j], snapshot.minimumAngles[j], snapshot.maximumAngles[j]);
                        done &= Mathf.Abs(q[j] - targetAngles[j]) < 0.0001f && Mathf.Abs(velocities[j]) < 0.0001f;
                    }
                    poses.Add((float[])q.Clone()); points.Add(snapshot.Evaluate(q)); times.Add((iteration + 1) * step);
                    if (done) break;
                }
                if (!done) return Failure(MotionPlanMode.JointSpace, "Movement did not finish within the 120-second preview bound. Check speed settings.");
                plan.valid = true; plan.jointPoses = poses.ToArray(); plan.worldPoints = points.ToArray(); plan.sampleTimes = times.ToArray();
                plan.durationSeconds = times[times.Count - 1]; plan.validationSamples = points.Count;
                plan.message = "Preview simulates the same speed and acceleration code as manual movement. No collision-clearance guarantee.";
                LastPlan = plan; Status = plan.message; return plan;
            }
            catch (ArgumentException exception) { return Failure(MotionPlanMode.JointSpace, exception.Message); }
            catch (InvalidOperationException exception) { return Failure(MotionPlanMode.JointSpace, exception.Message); }
        }

        public MotionPlan PreviewCartesian(Vector3 targetWorld, int samples = 32)
        {
            CancelPlayback(); Subscribe();
            try
            {
                RequireStopped();
                if (!ForwardKinematics.IsFinite(targetWorld)) return Failure(MotionPlanMode.Cartesian, "Target is not finite.");
                ForwardKinematics.Snapshot snapshot = ForwardKinematics.Capture(arm);
                int count = Mathf.Clamp(samples, 2, 128);
                MotionPlan plan = NewPlan(MotionPlanMode.Cartesian, "Cartesian line - interpolated IK waypoint preview");
                List<float[]> poses = new List<float[]> { arm.CurrentAngles() };
                List<float> times = new List<float> { 0f };
                List<Vector3> points = new List<Vector3> { snapshot.toolPosition };
                float[] seed = (float[])poses[0].Clone();
                float tolerance = Mathf.Max(0.1f, cartesianToleranceMillimeters);
                float solveTolerance = Mathf.Min(tolerance * 0.3f, inverseKinematics == null ? 0.2f : inverseKinematics.toleranceMillimeters);
                float maximumResidual = 0f;
                for (int i = 1; i <= count; i++)
                {
                    Vector3 waypoint = Vector3.Lerp(snapshot.toolPosition, targetWorld, i / (float)count);
                    IkResult result = InverseKinematics.SolveContinuous(snapshot, waypoint, seed, solveTolerance,
                        inverseKinematics == null ? 240 : Mathf.Max(240, inverseKinematics.maximumIterations));
                    if (!result.reachable) return Failure(MotionPlanMode.Cartesian, "Entire path rejected at waypoint " + i + ": " + result.message);
                    float[] previous = poses[poses.Count - 1];
                    float segmentDuration = 0.02f;
                    for (int j = 0; j < previous.Length; j++)
                    {
                        float distance = Mathf.Abs(result.angles[j] - previous[j]);
                        if (distance > 30f) return Failure(MotionPlanMode.Cartesian, "Entire path rejected: IK branch changes by more than 30 degrees between waypoints.");
                        // Cubic smoothstep has max derivative1.5 and max second derivative6.
                        segmentDuration = Mathf.Max(segmentDuration, 1.5f * distance / (plan.speeds[j] * plan.speedMultiplier),
                            Mathf.Sqrt(6f * distance / (plan.accelerations[j] * plan.speedMultiplier * plan.speedMultiplier)));
                    }
                    // Validate intermediate joint interpolation as well as solved XYZ waypoints.
                    // This samples the whole planned line; it is not an analytic continuum or collision proof.
                    for (int sub = 1; sub <= 8; sub++)
                    {
                        float fraction = sub / 8f;
                        float[] candidate = Interpolate(previous, result.angles, fraction);
                        Vector3 point = snapshot.Evaluate(candidate);
                        Vector3 expected = Vector3.Lerp(snapshot.toolPosition, targetWorld, ((i - 1) + fraction) / count);
                        float residual = Vector3.Distance(point, expected) * snapshot.millimetersPerWorldUnit;
                        maximumResidual = Mathf.Max(maximumResidual, residual);
                        if (residual > tolerance) return Failure(MotionPlanMode.Cartesian, "Entire path rejected: intermediate FK deviation exceeds " + tolerance.ToString("0.00") + " mm.");
                        points.Add(point);
                    }
                    poses.Add((float[])result.angles.Clone()); times.Add(times[times.Count - 1] + segmentDuration); seed = result.angles;
                }
                plan.valid = true; plan.jointPoses = poses.ToArray(); plan.worldPoints = points.ToArray(); plan.sampleTimes = times.ToArray();
                plan.durationSeconds = times[times.Count - 1]; plan.maxCartesianResidualMillimeters = maximumResidual;
                plan.validationSamples = points.Count;
                plan.message = "Every XYZ waypoint solved; joint interpolation sampled throughout the path (max deviation " + maximumResidual.ToString("0.000") +
                    " mm). Playback is a smooth interpolated preview that stops at waypoints; it is not the manual joint dynamics or a collision guarantee.";
                LastPlan = plan; Status = plan.message; return plan;
            }
            catch (ArgumentException exception) { return Failure(MotionPlanMode.Cartesian, exception.Message); }
            catch (InvalidOperationException exception) { return Failure(MotionPlanMode.Cartesian, exception.Message); }
        }

        public bool Play()
        {
            Subscribe();
            if (arm == null || LastPlan == null || !LastPlan.valid || LastPlan.jointPoses.Length == 0) { Status = "Create a valid preview first."; return false; }
            if (!arm.IsAtTarget) { Status = "Stop movement and preview again before playback."; return false; }
            float[] current = arm.CurrentAngles();
            for (int i = 0; i < current.Length; i++)
            {
                if (arm.joints[i].instantMode || Mathf.Abs(current[i] - LastPlan.jointPoses[0][i]) > 0.01f ||
                    !Mathf.Approximately(arm.joints[i].maximumSpeed, LastPlan.speeds[i]) || !Mathf.Approximately(arm.joints[i].acceleration, LastPlan.accelerations[i]))
                { Status = "Start pose or motion settings changed; create a fresh preview."; return false; }
            }
            if (!Mathf.Approximately(arm.speedMultiplier, LastPlan.speedMultiplier) || arm.instantMode)
            { Status = "Speed multiplier or instant mode changed; disable instant mode and preview again."; return false; }
            try
            {
                if (!SameConfiguration(LastPlan.snapshot, ForwardKinematics.Capture(arm)))
                { Status = "Joint configuration, origin or geometry changed; create a fresh preview."; return false; }
            }
            catch (ArgumentException exception) { Status = exception.Message; return false; }
            if (inverseKinematics != null) inverseKinematics.Cancel();
            playTime = 0f; playSegment = 0; IsPlaying = true;
            if (LastPlan.mode == MotionPlanMode.JointSpace) Command(LastPlan.jointPoses[LastPlan.jointPoses.Length - 1], false);
            else Command(LastPlan.jointPoses[0], true);
            Status = "Playing: " + LastPlan.label; return true;
        }

        public void Tick(float deltaTime)
        {
            Subscribe();
            if (!IsPlaying || arm == null || !JointMotion.IsFinite(deltaTime) || deltaTime < 0f) return;
            if (LastPlan.mode == MotionPlanMode.JointSpace)
            {
                if (arm.IsAtTarget) { CancelPlayback(); Status = "Joint-space playback complete."; }
                return;
            }
            playTime += deltaTime;
            if (playTime >= LastPlan.durationSeconds)
            {
                Command(LastPlan.jointPoses[LastPlan.jointPoses.Length - 1], true); CancelPlayback(); Status = "Cartesian preview playback complete."; return;
            }
            while (playSegment + 1 < LastPlan.sampleTimes.Length - 1 && playTime > LastPlan.sampleTimes[playSegment + 1]) playSegment++;
            float span = LastPlan.sampleTimes[playSegment + 1] - LastPlan.sampleTimes[playSegment];
            float t = Mathf.Clamp01((playTime - LastPlan.sampleTimes[playSegment]) / span);
            t = t * t * (3f - 2f * t);
            Command(Interpolate(LastPlan.jointPoses[playSegment], LastPlan.jointPoses[playSegment + 1], t), true);
        }

        public void Stop() { CancelPlayback(); if (arm != null) arm.Stop(); Status = "Plan playback stopped."; }
        public void ClearPreview() { Stop(); LastPlan = null; Status = "Preview cleared."; }

        private MotionPlan NewPlan(MotionPlanMode mode, string label)
        {
            MotionPlan plan = new MotionPlan { mode = mode, label = label, speedMultiplier = arm.speedMultiplier,
                speeds = new float[arm.joints.Length], accelerations = new float[arm.joints.Length], snapshot = ForwardKinematics.Capture(arm) };
            for (int i = 0; i < arm.joints.Length; i++)
            {
                plan.speeds[i] = arm.joints[i].maximumSpeed; plan.accelerations[i] = arm.joints[i].acceleration;
                if (!JointMotion.IsFinite(plan.speeds[i]) || !JointMotion.IsFinite(plan.accelerations[i]) || plan.speeds[i] <= 0f || plan.accelerations[i] <= 0f || arm.joints[i].instantMode) throw new InvalidOperationException("Use finite positive speed/acceleration and disable per-joint instant mode.");
            }
            if (!JointMotion.IsFinite(plan.speedMultiplier) || plan.speedMultiplier <= 0f || arm.instantMode) throw new InvalidOperationException("Use a finite positive speed multiplier and disable instant mode before preview.");
            return plan;
        }

        private static bool SameConfiguration(ForwardKinematics.Snapshot a, ForwardKinematics.Snapshot b)
        {
            if (a == null || a.baseAngles.Length != b.baseAngles.Length || a.controlledJointCount != b.controlledJointCount ||
                Vector3.Distance(a.toolPosition, b.toolPosition) > 0.00001f || !Mathf.Approximately(a.millimetersPerUnityUnit, b.millimetersPerUnityUnit)) return false;
            for (int k = 0; k < 16; k++) if (Mathf.Abs(a.worldToOrigin[k] - b.worldToOrigin[k]) > 0.00001f) return false;
            for (int i = 0; i < a.baseAngles.Length; i++)
            {
                if (!Mathf.Approximately(a.minimumAngles[i], b.minimumAngles[i]) || !Mathf.Approximately(a.maximumAngles[i], b.maximumAngles[i]) ||
                    a.directionSigns[i] != b.directionSigns[i] || !Mathf.Approximately(a.angleOffsets[i], b.angleOffsets[i]) ||
                    Vector3.Distance(a.configuredLocalAxes[i], b.configuredLocalAxes[i]) > 0.000001f ||
                    Quaternion.Angle(a.baselineRotations[i], b.baselineRotations[i]) > 0.001f) return false;
                if (i < a.controlledJointCount && (Vector3.Distance(a.pivots[i], b.pivots[i]) > 0.00001f || Vector3.Distance(a.axes[i], b.axes[i]) > 0.00001f)) return false;
            }
            return true;
        }

        private void RequireStopped()
        {
            if (arm == null || !arm.IsAtTarget) throw new InvalidOperationException("Stop current movement before creating a preview.");
        }
        private MotionPlan Failure(MotionPlanMode mode, string message)
        { LastPlan = new MotionPlan { valid = false, mode = mode, label = "Invalid preview", message = message }; Status = message; return LastPlan; }
        private static float[] Interpolate(float[] a, float[] b, float t)
        { float[] result = new float[a.Length]; for (int i = 0; i < result.Length; i++) result[i] = Mathf.Lerp(a[i], b[i], t); return result; }
        private void Command(float[] angles, bool immediate)
        {
            issuingOwnCommand = true;
            try { if (immediate) arm.SetAnglesImmediate(angles); else arm.MoveToPose(angles); }
            finally { issuingOwnCommand = false; }
        }
        private void CancelPlayback() { IsPlaying = false; playTime = 0f; playSegment = 0; }
        private void ExternalCommand() { if (!issuingOwnCommand) { CancelPlayback(); Status = "Playback cancelled by a manual command or STOP."; } }
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
    }
}
