using System;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>CAD hierarchy snapshots. Candidate evaluation never writes to scene transforms.</summary>
    public static class ForwardKinematics
    {
        public sealed class Snapshot
        {
            public readonly int controlledJointCount;
            public readonly float[] baseAngles, minimumAngles, maximumAngles, homeAngles, directionSigns;
            public readonly float[] angleOffsets;
            public readonly Vector3[] configuredLocalAxes;
            public readonly Quaternion[] baselineRotations;
            public readonly Vector3[] pivots, axes;
            public readonly bool[,] affectsJoint;
            public readonly bool[] affectsTool;
            public readonly Vector3 toolPosition;
            public readonly Matrix4x4 worldToOrigin;
            public readonly float millimetersPerUnityUnit, millimetersPerWorldUnit;
            public readonly bool hasSerialReachBound;
            public readonly float maximumReachWorld;

            internal Snapshot(RobotArm arm, int controlled)
            {
                if (arm == null || arm.endEffector == null || arm.robotOrigin == null || arm.joints == null)
                    throw new ArgumentException("Assign the arm, origin, tool and joints before capturing kinematics.");
                if (controlled < 1 || controlled > arm.joints.Length) throw new ArgumentOutOfRangeException(nameof(controlled));
                controlledJointCount = controlled;
                int count = arm.joints.Length;
                baseAngles = arm.CurrentAngles(); minimumAngles = new float[count]; maximumAngles = new float[count];
                homeAngles = new float[count]; directionSigns = new float[count];
                angleOffsets = new float[count]; configuredLocalAxes = new Vector3[count]; baselineRotations = new Quaternion[count];
                pivots = new Vector3[controlled]; axes = new Vector3[controlled];
                affectsJoint = new bool[controlled, controlled]; affectsTool = new bool[controlled];
                toolPosition = arm.endEffector.position; worldToOrigin = arm.robotOrigin.worldToLocalMatrix;
                millimetersPerUnityUnit = arm.millimetersPerUnityUnit;
                float originScale = UniformScale(arm.robotOrigin);
                millimetersPerWorldUnit = millimetersPerUnityUnit / originScale;
                if (!IsFinite(toolPosition) || !JointMotion.IsFinite(millimetersPerWorldUnit) || millimetersPerWorldUnit <= 0f)
                    throw new ArgumentException("Invalid tool position or origin scale.");
                bool serial = true;
                float reach = 0f;
                for (int i = 0; i < count; i++)
                {
                    RobotJoint joint = arm.joints[i];
                    if (joint == null || !JointMotion.IsFinite(baseAngles[i]) || !IsFinite(joint.rotationAxis))
                        throw new ArgumentException("Invalid joint configuration at index " + i + ".");
                    minimumAngles[i] = Mathf.Min(joint.minimumAngle, joint.maximumAngle);
                    maximumAngles[i] = Mathf.Max(joint.minimumAngle, joint.maximumAngle);
                    homeAngles[i] = joint.homeAngle; directionSigns[i] = joint.reverseDirection ? -1f : 1f;
                    angleOffsets[i] = joint.angleOffset; configuredLocalAxes[i] = joint.NormalizedAxis;
                    baselineRotations[i] = joint.baselineLocalRotation;
                    if (!JointMotion.IsFinite(minimumAngles[i]) || !JointMotion.IsFinite(maximumAngles[i]))
                        throw new ArgumentException("Joint limits must be finite.");
                    if (i >= controlled) continue;
                    UniformScale(joint.transform);
                    pivots[i] = joint.transform.position;
                    // A rotation about its own axis does not change that axis. Existing baseline,
                    // offset and parent rotations are already present in this captured world axis.
                    axes[i] = joint.transform.TransformDirection(joint.NormalizedAxis).normalized;
                    affectsTool[i] = arm.endEffector.IsChildOf(joint.transform);
                    serial &= affectsTool[i];
                    for (int j = i + 1; j < controlled; j++)
                    {
                        if (arm.joints[j] == null) throw new ArgumentException("Missing joint.");
                        affectsJoint[i, j] = arm.joints[j].transform.IsChildOf(joint.transform);
                        serial &= affectsJoint[i, j];
                    }
                    if (i > 0) reach += Vector3.Distance(pivots[i - 1], pivots[i]);
                }
                reach += Vector3.Distance(pivots[controlled - 1], toolPosition);
                maximumReachWorld = reach; hasSerialReachBound = serial;
            }

            public Vector3 Evaluate(float[] angles)
            {
                Vector3[] unusedPivots, unusedAxes;
                return EvaluateChain(angles, out unusedPivots, out unusedAxes);
            }

            public Vector3 EvaluateMillimeters(float[] angles) => worldToOrigin.MultiplyPoint3x4(Evaluate(angles)) * millimetersPerUnityUnit;

            public Vector3 EvaluateChain(float[] angles, out Vector3[] candidatePivots, out Vector3[] candidateAxes)
            {
                ValidateAngles(angles);
                candidatePivots = (Vector3[])pivots.Clone(); candidateAxes = (Vector3[])axes.Clone();
                Vector3 tool = toolPosition;
                for (int i = 0; i < controlledJointCount; i++)
                {
                    Quaternion turn = Quaternion.AngleAxis((angles[i] - baseAngles[i]) * directionSigns[i], candidateAxes[i]);
                    Vector3 center = candidatePivots[i];
                    if (affectsTool[i]) tool = center + turn * (tool - center);
                    for (int j = i + 1; j < controlledJointCount; j++) if (affectsJoint[i, j])
                    {
                        candidatePivots[j] = center + turn * (candidatePivots[j] - center);
                        candidateAxes[j] = turn * candidateAxes[j];
                    }
                }
                return tool;
            }

            public float[] Clamp(float[] angles)
            {
                if (angles == null || angles.Length != baseAngles.Length) throw new ArgumentException("Pose length differs from captured joints.");
                float[] result = (float[])angles.Clone();
                for (int i = 0; i < result.Length; i++)
                {
                    if (!JointMotion.IsFinite(result[i])) throw new ArgumentException("Pose contains a nonfinite angle.");
                    result[i] = i < controlledJointCount ? Mathf.Clamp(result[i], minimumAngles[i], maximumAngles[i]) : baseAngles[i];
                }
                return result;
            }

            public void ValidateAngles(float[] angles)
            {
                if (angles == null || angles.Length != baseAngles.Length) throw new ArgumentException("Pose length differs from captured joints.");
                for (int i = 0; i < angles.Length; i++)
                {
                    if (!JointMotion.IsFinite(angles[i]) || angles[i] < minimumAngles[i] - 0.0001f || angles[i] > maximumAngles[i] + 0.0001f)
                        throw new ArgumentException("Pose exceeds a joint limit or is nonfinite.");
                    if (i >= controlledJointCount && Mathf.Abs(angles[i] - baseAngles[i]) > 0.0001f)
                        throw new ArgumentException("This positional snapshot holds the gripper fixed. Use a pose sequence for gripper moves.");
                }
            }
        }

        public static Snapshot Capture(RobotArm arm, int controlledJointCount = 4) => new Snapshot(arm, controlledJointCount);
        public static bool IsFinite(Vector3 value) => JointMotion.IsFinite(value.x) && JointMotion.IsFinite(value.y) && JointMotion.IsFinite(value.z);

        private static float UniformScale(Transform transform)
        {
            Vector3 s = transform.lossyScale;
            float largest = Mathf.Max(s.x, s.y, s.z);
            if (!IsFinite(s) || Mathf.Min(s.x, s.y, s.z) <= 0f || Mathf.Abs(s.x - s.y) > largest * 0.0001f || Mathf.Abs(s.x - s.z) > largest * 0.0001f)
                throw new ArgumentException("Kinematics requires positive uniform scale; imported robot geometry must not be distorted.");
            return s.x;
        }
    }
}
