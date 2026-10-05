using System;
using UnityEngine;

namespace CareerFair.Robot
{
    [DefaultExecutionOrder(-50)]
    [DisallowMultipleComponent]
    public sealed class RobotArm : MonoBehaviour
    {
        public RobotJoint[] joints = Array.Empty<RobotJoint>();
        public Transform robotOrigin;
        public Transform endEffector;
        public Transform target;
        public bool automaticUpdate = true;
        public bool instantMode;
        [Min(0f)] public float speedMultiplier = 1f;
        [Tooltip("Unity length units are metres; origin coordinates are displayed in millimetres.")]
        public float millimetersPerUnityUnit = 1000f;
        [Tooltip("This project contains simulation only. No hardware connection is implemented.")]
        public bool simulatedOnly = true;
        public RobotNeutralReference neutralReference;
        [SerializeReference, Tooltip("Optional saved target for Home and Reset. Does not change startup or hardware calibration.")]
        public RobotPose operationalHome;
        [NonSerialized] public float externalSpeedMultiplierLimit = float.PositiveInfinity;
        [TextArea] public string sourceProvenance = "CAD-derived simulated geometry. No physical hardware control or calibration.";

        public event Action PoseChanged;
        public event Action PoseUpdated;
        public event Action SimulationStopped;
        public event Action MotionCommandIssued;
        public Vector3 EndEffectorMillimeters => RelativeMillimeters(endEffector);
        public Vector3 TargetMillimeters => RelativeMillimeters(target);
        public bool IsAtTarget
        {
            get
            {
                if (joints == null) return true;
                foreach (RobotJoint joint in joints) if (joint != null && !joint.IsAtTarget) return false;
                return true;
            }
        }

        private void Update() { if (automaticUpdate) Tick(Time.deltaTime); }

        public void SetJointTarget(string jointName, float angle)
        {
            RobotJoint joint = FindJoint(jointName);
            if (joint == null) throw new ArgumentException("Unknown simulated joint: " + jointName, nameof(jointName));
            if (!JointMotion.IsFinite(angle)) throw new ArgumentOutOfRangeException(nameof(angle));
            MotionCommandIssued?.Invoke();
            joint.SetTarget(angle);
            if (instantMode) { joint.SetAngleImmediate(angle); NotifyPoseUpdated(); }
        }

        public RobotJoint FindJoint(string jointName)
        {
            if (joints == null) return null;
            foreach (RobotJoint joint in joints)
                if (joint != null && string.Equals(joint.jointName, jointName, StringComparison.OrdinalIgnoreCase)) return joint;
            return null;
        }

        public void MoveToPose(float[] angles)
        {
            ValidatePose(angles);
            MotionCommandIssued?.Invoke();
            for (int i = 0; i < joints.Length; i++) joints[i].SetTarget(angles[i]);
            if (instantMode) SetAnglesImmediateInternal(angles);
        }

        public void Home()
        {
            MoveToPose(ValidatedHomePose());
        }

        public void ResetHome()
        {
            ValidatedHomePose();
            Stop();
            // HOLD can restore narrower bounds. Never let Reset silently clip the saved pose.
            SetAnglesImmediate(ValidatedHomePose());
        }

        public void ValidateHomePose() { ValidatedHomePose(); }

        private float[] ValidatedHomePose()
        {
            if (joints == null || joints.Length == 0)
                throw new InvalidOperationException("No robot joints are configured.");
            foreach (var joint in joints)
                if (joint == null) throw new InvalidOperationException("A robot joint is missing.");
            float[] home;
            if (operationalHome != null) home = operationalHome.AnglesFor(this);
            else
            {
                home = new float[joints.Length];
                for (int i = 0; i < home.Length; i++) home[i] = joints[i].homeAngle;
            }
            ValidatePose(home);
            const float roundoffTolerance = .0001f;
            for (int i = 0; i < home.Length; i++)
            {
                if (!JointMotion.IsFinite(joints[i].minimumAngle) || !JointMotion.IsFinite(joints[i].maximumAngle))
                    throw new InvalidOperationException("Home travel limits are unavailable.");
                float bounded = joints[i].Clamp(home[i]);
                if (Mathf.Abs(home[i] - bounded) > roundoffTolerance)
                    throw new InvalidOperationException("Saved Home is outside the current travel range. Enable Direct arm control to use this pose.");
                home[i] = bounded;
            }
            var link = GetComponent<RobotHardwareLink>();
            if (link != null && link.FollowActive && !link.TryValidatePlaybackPose(home, out string reason))
                throw new InvalidOperationException("Home is unavailable: " + reason);
            return home;
        }

        public void Stop()
        {
            SimulationStopped?.Invoke();
            if (joints == null) return;
            foreach (RobotJoint joint in joints) if (joint != null) joint.Stop();
        }

        public void Tick(float deltaTime)
        {
            if (!JointMotion.IsFinite(deltaTime) || deltaTime < 0f) return;
            if (joints == null) return;
            float requestedSpeed = JointMotion.IsFinite(speedMultiplier) ? Mathf.Max(0f, speedMultiplier) : 0f;
            bool externallyLimited = !float.IsPositiveInfinity(externalSpeedMultiplierLimit);
            float externalLimit = JointMotion.IsFinite(externalSpeedMultiplierLimit)
                ? Mathf.Max(0f, externalSpeedMultiplierLimit) : 0f;
            float effectiveSpeed = externallyLimited ? Mathf.Min(requestedSpeed, externalLimit) : requestedSpeed;
            // Physical follow already limits velocity. Scaling acceleration once
            // avoids a second slowdown while retaining the same velocity ceiling.
            // Unconnected simulation keeps its established squared time scaling.
            float accelerationScale = externallyLimited ? effectiveSpeed : effectiveSpeed * effectiveSpeed;
            foreach (RobotJoint joint in joints)
                if (joint != null) joint.Tick(deltaTime, effectiveSpeed, instantMode, accelerationScale);
            NotifyPoseUpdated();
        }

        public float[] CurrentAngles()
        {
            float[] result = new float[joints == null ? 0 : joints.Length];
            for (int i = 0; i < result.Length; i++) result[i] = joints[i] == null ? 0f : joints[i].currentAngle;
            return result;
        }

        public void SetAnglesImmediate(float[] angles)
        {
            ValidatePose(angles);
            MotionCommandIssued?.Invoke();
            SetAnglesImmediateInternal(angles);
        }

        private void SetAnglesImmediateInternal(float[] angles)
        {
            for (int i = 0; i < joints.Length; i++) joints[i].SetAngleImmediate(angles[i]);
            NotifyPoseUpdated();
        }

        private void NotifyPoseUpdated()
        {
            PoseChanged?.Invoke();
            PoseUpdated?.Invoke();
        }

        private void ValidatePose(float[] angles)
        {
            if (joints == null || angles == null || angles.Length != joints.Length)
                throw new ArgumentException("A pose must contain one angle for every configured joint.", nameof(angles));
            for (int i = 0; i < angles.Length; i++)
                if (joints[i] == null || !JointMotion.IsFinite(angles[i]))
                    throw new ArgumentException("Pose/joint entry " + i + " is invalid.", nameof(angles));
        }

        private Vector3 RelativeMillimeters(Transform marker)
        {
            if (marker == null || robotOrigin == null) return Vector3.zero;
            return robotOrigin.InverseTransformPoint(marker.position) * millimetersPerUnityUnit;
        }

        private void OnValidate()
        {
            simulatedOnly = true;
            if (!JointMotion.IsFinite(speedMultiplier)) speedMultiplier = 1f;
            speedMultiplier = Mathf.Max(0f, speedMultiplier);
            if (!JointMotion.IsFinite(millimetersPerUnityUnit) || millimetersPerUnityUnit <= 0f) millimetersPerUnityUnit = 1000f;
        }
    }
}
