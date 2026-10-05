using UnityEngine;

namespace CareerFair.Robot.Hardware
{
    /// <summary>Temporary model bounds used by both IK and joint interpolation during physical follow.</summary>
    public sealed class VerifiedJointTravel
    {
        private RobotJoint[] joints;
        private float[] originalMinimum, originalMaximum;
        public bool Active => joints != null;

        public bool TryApply(RobotArm arm, RobotHardwareMapping mapping, float[] currentModelPose, out string reason)
        {
            reason = null;
            if (Active) { reason = "Stop the current follow session before changing travel limits."; return false; }
            if (mapping == null || !mapping.HasVerifiedCalibration(out reason)) return false;
            if (arm == null || arm.joints == null || arm.joints.Length != RobotHardwareMapping.ChannelCount ||
                currentModelPose == null || currentModelPose.Length != RobotHardwareMapping.ChannelCount)
            { reason = "Five joints and a current mapped pose are required."; return false; }
            if (!mapping.TryMapPose(currentModelPose, out _, out reason)) return false;

            var lower = new float[5]; var upper = new float[5];
            // Validate every intersection before touching any joint or moving the model.
            for (int i = 0; i < 5; i++)
            {
                var joint = arm.joints[i];
                if (joint == null || !RobotHardwareMapping.Finite(joint.minimumAngle) || !RobotHardwareMapping.Finite(joint.maximumAngle))
                { reason = "Invalid model travel on channel " + i + "."; return false; }
                float a = mapping.modelHome[i] + mapping.directions[i] * (mapping.minimumCommand[i] - mapping.servoNeutral[i]);
                float b = mapping.modelHome[i] + mapping.directions[i] * (mapping.maximumCommand[i] - mapping.servoNeutral[i]);
                lower[i] = Mathf.Max(Mathf.Min(a, b), Mathf.Min(joint.minimumAngle, joint.maximumAngle));
                upper[i] = Mathf.Min(Mathf.Max(a, b), Mathf.Max(joint.minimumAngle, joint.maximumAngle));
                if (lower[i] > upper[i] || currentModelPose[i] < lower[i] - .0001f || currentModelPose[i] > upper[i] + .0001f)
                { reason = "Current command is outside the verified model travel on channel " + i + "."; return false; }
            }

            joints = (RobotJoint[])arm.joints.Clone();
            originalMinimum = new float[5]; originalMaximum = new float[5];
            for (int i = 0; i < 5; i++)
            {
                originalMinimum[i] = joints[i].minimumAngle; originalMaximum[i] = joints[i].maximumAngle;
                joints[i].minimumAngle = lower[i]; joints[i].maximumAngle = upper[i];
            }
            return true;
        }

        public void Restore()
        {
            if (!Active) return;
            for (int i = 0; i < joints.Length; i++)
                if (joints[i] != null)
                { joints[i].minimumAngle = originalMinimum[i]; joints[i].maximumAngle = originalMaximum[i]; }
            joints = null; originalMinimum = originalMaximum = null;
        }
    }
}
