using System;
using System.IO;
using System.Linq;
using UnityEngine;

namespace CareerFair.Robot
{
    [Serializable]
    public sealed class RobotPose
    {
        public int formatVersion = 1;
        public string name = "Pose";
        public string[] jointNames = Array.Empty<string>();
        public float[] angles = Array.Empty<float>();
        public string savedUtc;
        public bool simulatedOnly = true;
        public static string StorageDirectory => Path.Combine(Application.persistentDataPath, "RobotPoses");

        public static RobotPose Capture(RobotArm arm, string poseName)
        {
            if (arm == null) throw new ArgumentNullException(nameof(arm));
            RobotPose pose = new RobotPose { name = poseName, angles = arm.CurrentAngles(), savedUtc = DateTime.UtcNow.ToString("o") };
            pose.jointNames = arm.joints.Select(j => j.jointName).ToArray();
            pose.Validate();
            return pose;
        }

        public float[] AnglesFor(RobotArm arm)
        {
            if (arm == null) throw new ArgumentNullException(nameof(arm));
            Validate();
            if (jointNames.Length != arm.joints.Length) throw new InvalidDataException("Saved pose joint count differs from this arm.");
            float[] result = new float[arm.joints.Length];
            for (int i = 0; i < result.Length; i++)
            {
                string wanted = arm.joints[i].jointName;
                int index = Array.FindIndex(jointNames, n => string.Equals(n, wanted, StringComparison.OrdinalIgnoreCase));
                if (index < 0) throw new InvalidDataException("Saved pose is missing joint " + wanted + ".");
                result[i] = angles[index];
            }
            return result;
        }

        public void Apply(RobotArm arm, bool immediate = false)
        {
            float[] values = AnglesFor(arm);
            if (immediate) arm.SetAnglesImmediate(values); else arm.MoveToPose(values);
        }

        public string Save()
        {
            Validate();
            Directory.CreateDirectory(StorageDirectory);
            savedUtc = DateTime.UtcNow.ToString("o");
            string path = FilePath(name);
            File.WriteAllText(path, JsonUtility.ToJson(this, true));
            return path;
        }

        public static RobotPose Load(string poseName)
        {
            string path = FilePath(poseName);
            RobotPose pose = JsonUtility.FromJson<RobotPose>(File.ReadAllText(path));
            if (pose == null) throw new InvalidDataException("Invalid saved pose JSON.");
            pose.Validate();
            return pose;
        }

        public static string[] SavedPoseNames()
        {
            if (!Directory.Exists(StorageDirectory)) return Array.Empty<string>();
            return Directory.GetFiles(StorageDirectory, "*.json").Select(Path.GetFileNameWithoutExtension)
                .OrderBy(n => n, StringComparer.OrdinalIgnoreCase).ToArray();
        }

        private static string FilePath(string poseName)
        {
            if (string.IsNullOrWhiteSpace(poseName)) throw new ArgumentException("Enter a pose name.", nameof(poseName));
            if (poseName.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0 || poseName.Contains("/") || poseName.Contains("\\") || poseName == "." || poseName == "..")
                throw new ArgumentException("Pose names cannot contain path separators or invalid filename characters.", nameof(poseName));
            return Path.Combine(StorageDirectory, poseName.Trim() + ".json");
        }

        public void Validate()
        {
            if (formatVersion != 1 || !simulatedOnly) throw new InvalidDataException("Unsupported simulation pose format.");
            if (jointNames == null || angles == null || jointNames.Length == 0 || jointNames.Length != angles.Length)
                throw new InvalidDataException("Pose names and angles must have the same nonzero length.");
            if (jointNames.Any(string.IsNullOrWhiteSpace) || jointNames.Distinct(StringComparer.OrdinalIgnoreCase).Count() != jointNames.Length)
                throw new InvalidDataException("Joint names must be present and unique.");
            if (angles.Any(a => !JointMotion.IsFinite(a))) throw new InvalidDataException("Pose angles must be finite.");
        }
    }
}
