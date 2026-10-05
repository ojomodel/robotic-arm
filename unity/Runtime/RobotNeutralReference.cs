using System;
using UnityEngine;

namespace CareerFair.Robot
{
    [Serializable]
    public sealed class NeutralJointReference
    {
        public string jointName;
        public int channel;
        public float servoCommandDegrees;
        public float modelHomeDegrees;
        public string observedState;
    }

    /// <summary>A single observed pose correspondence, not a calibrated hardware command mapping.</summary>
    public sealed class RobotNeutralReference : ScriptableObject
    {
        public string referenceDate;
        [TextArea] public string poseDescription;
        [TextArea] public string limitations;
        public string sourceMonitorReport;
        public string photoEmailSubject;
        public string photoEmailReceived;
        public string[] photoUrls;
        public NeutralJointReference[] joints;

        public NeutralJointReference Find(string jointName)
        {
            if (joints == null) return null;
            return Array.Find(joints, j => j != null && string.Equals(j.jointName, jointName, StringComparison.OrdinalIgnoreCase));
        }
    }
}
