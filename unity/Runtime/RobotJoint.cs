using System;
using UnityEngine;

namespace CareerFair.Robot
{
    [DisallowMultipleComponent]
    public sealed class RobotJoint : MonoBehaviour
    {
        public string jointName = "Joint";
        [Tooltip("Axis in the neutral pivot's local coordinate frame.")]
        public Vector3 rotationAxis = Vector3.up;
        public float minimumAngle = -90f;
        public float maximumAngle = 90f;
        public float homeAngle;
        public float currentAngle;
        public float targetAngle;
        [Min(0f)] public float maximumSpeed = 45f;
        [Min(0.01f)] public float acceleration = 90f;
        public bool reverseDirection;
        public float angleOffset;
        public bool instantMode;
        [Tooltip("Neutral (zero command) pivot orientation, before offset and direction are applied.")]
        public Quaternion baselineLocalRotation = Quaternion.identity;
        [SerializeField] private bool baselineCaptured;
        [SerializeField] private float currentVelocity;

        public float CurrentVelocity => currentVelocity;
        public bool IsAtTarget => Mathf.Abs(currentAngle - targetAngle) <= 0.01f && Mathf.Abs(currentVelocity) <= 0.01f;
        public Vector3 NormalizedAxis => rotationAxis.sqrMagnitude > 0.0000001f ? rotationAxis.normalized : Vector3.up;
        public float AppliedAngle => (reverseDirection ? -1f : 1f) * (currentAngle + angleOffset);

        private void Awake() { EnsureBaseline(); ApplyRotation(); }

        public void CaptureBaseline()
        {
            baselineLocalRotation = transform.localRotation;
            baselineCaptured = true;
            ApplyRotation();
        }

        public void SetBaseline(Quaternion rotation)
        {
            baselineLocalRotation = rotation.normalized;
            baselineCaptured = true;
            ApplyRotation();
        }

        public void SetTarget(float degrees)
        {
            ValidateAngle(degrees);
            targetAngle = Clamp(degrees);
        }

        public void SetAngleImmediate(float degrees)
        {
            ValidateAngle(degrees);
            currentAngle = targetAngle = Clamp(degrees);
            currentVelocity = 0f;
            ApplyRotation();
        }

        public void Stop()
        {
            targetAngle = currentAngle;
            currentVelocity = 0f;
        }

        public void Tick(float deltaTime, float speedMultiplier = 1f, bool forceInstant = false,
            float? accelerationScale = null)
        {
            EnsureBaseline();
            targetAngle = Clamp(targetAngle);
            if (forceInstant || instantMode)
            {
                SetAngleImmediate(targetAngle);
                return;
            }
            if (!JointMotion.IsFinite(deltaTime) || deltaTime <= 0f) return;
            float multiplier = JointMotion.IsFinite(speedMultiplier) ? Mathf.Max(0f, speedMultiplier) : 0f;
            float scaledAcceleration = accelerationScale.HasValue
                ? (JointMotion.IsFinite(accelerationScale.Value) ? Mathf.Max(0f, accelerationScale.Value) : 0f)
                : multiplier * multiplier;
            JointMotion.Step(ref currentAngle, ref currentVelocity, targetAngle,
                maximumSpeed * multiplier, acceleration * scaledAcceleration, deltaTime);
            float limited = Clamp(currentAngle);
            if (limited != currentAngle) { currentAngle = limited; currentVelocity = 0f; }
            ApplyRotation();
        }

        public void ApplyRotation()
        {
            EnsureBaseline();
            transform.localRotation = baselineLocalRotation * Quaternion.AngleAxis(AppliedAngle, NormalizedAxis);
        }

        public float Clamp(float degrees) => Mathf.Clamp(degrees, Mathf.Min(minimumAngle, maximumAngle), Mathf.Max(minimumAngle, maximumAngle));

        private void EnsureBaseline()
        {
            if (baselineCaptured) return;
            baselineLocalRotation = transform.localRotation;
            baselineCaptured = true;
        }

        private static void ValidateAngle(float degrees)
        {
            if (!JointMotion.IsFinite(degrees)) throw new ArgumentOutOfRangeException(nameof(degrees), "Joint angle must be finite.");
        }

        private void OnValidate()
        {
            if (!JointMotion.IsFinite(minimumAngle)) minimumAngle = -90f;
            if (!JointMotion.IsFinite(maximumAngle)) maximumAngle = 90f;
            if (minimumAngle > maximumAngle) { float swap = minimumAngle; minimumAngle = maximumAngle; maximumAngle = swap; }
            if (rotationAxis.sqrMagnitude < 0.0000001f) rotationAxis = Vector3.up;
            maximumSpeed = JointMotion.IsFinite(maximumSpeed) ? Mathf.Max(0f, maximumSpeed) : 45f;
            acceleration = JointMotion.IsFinite(acceleration) ? Mathf.Max(0.01f, acceleration) : 90f;
            homeAngle = Clamp(JointMotion.IsFinite(homeAngle) ? homeAngle : 0f);
            currentAngle = Clamp(JointMotion.IsFinite(currentAngle) ? currentAngle : homeAngle);
            targetAngle = Clamp(JointMotion.IsFinite(targetAngle) ? targetAngle : homeAngle);
            if (!JointMotion.IsFinite(angleOffset)) angleOffset = 0f;
        }
    }
}
