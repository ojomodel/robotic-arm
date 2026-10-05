using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Passive, timed capture of current model commands. Never commands or arms the robot.</summary>
    [DisallowMultipleComponent]
    public sealed class MotionRecordingCapture : MonoBehaviour
    {
        public RobotArm arm;
        public bool automaticUpdate = true;
        public string recordingDirectory;
        public bool IsRecording { get; private set; }
        public bool HasUnsavedRecording => buffer != null && !IsRecording;
        public float ElapsedSeconds => (float)elapsed;
        public int SampleCount => buffer == null ? 0 : buffer.samples.Count;
        public string Status { get; private set; } = "Start recording, control the arm, then Stop & Save.";
        public string LastSavedPath { get; private set; }
        public string LastSavedId { get; private set; }
        public string LastSavedID => LastSavedId;
        public string DirectoryPath => string.IsNullOrWhiteSpace(recordingDirectory)
            ? Path.Combine(Application.persistentDataPath, "RobotRecordings") : recordingDirectory;
        public RobotMotionRecording RecordingSnapshot => RobotMotionRecordingStore.DeepClone(buffer);
        private RobotMotionRecording buffer;
        private double elapsed, lastRealtime;
        private const double SamplePeriod = .05;

        private void Start() { if (arm == null) arm = GetComponent<RobotArm>(); }
        private void LateUpdate() { if (automaticUpdate && IsRecording) AdvanceRealtime(); }
        private void OnApplicationFocus(bool focused) { if (!focused && IsRecording) FinishForLifecycle("Focus lost"); }
        private void OnApplicationQuit() { if (IsRecording || HasUnsavedRecording) FinishForLifecycle("Application closing"); }
        private void OnDisable() { if (IsRecording || HasUnsavedRecording) FinishForLifecycle("Recorder disabled"); }

        public bool StartRecording(string name, out string reason)
        {
            if (IsRecording) return Fail("A recording is already running.", out reason);
            if (HasUnsavedRecording) return Fail("Save the pending recording before starting another.", out reason);
            foreach (var player in FindObjectsByType<MotionRecordingPlayer>(FindObjectsSortMode.None))
                if (player.IsPlaying) return Fail("Stop playback before starting a recording.", out reason);
            if (arm == null) arm = GetComponent<RobotArm>();
            if (!TryCurrentAngles(out float[] angles, out reason)) { Status = reason; return false; }
            string label = string.IsNullOrWhiteSpace(name)
                ? "Recording " + DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss", CultureInfo.InvariantCulture) : name.Trim();
            if (label.Length > 64) return Fail("Use a recording name of at most 64 characters.", out reason);
            buffer = new RobotMotionRecording { name = label };
            buffer.samples.Add(new RobotMotionSample { timeSeconds = 0f, angles = angles });
            elapsed = 0; lastRealtime = Time.realtimeSinceStartupAsDouble;
            IsRecording = true; Status = "Recording: " + label; reason = null; return true;
        }

        private void AdvanceRealtime()
        {
            double now = Time.realtimeSinceStartupAsDouble;
            double dt = Math.Max(0, now - lastRealtime); lastRealtime = now;
            Tick((float)dt);
        }

        public void Tick(float deltaTime)
        {
            if (!IsRecording || !JointMotion.IsFinite(deltaTime) || deltaTime <= 0f) return;
            double next = elapsed + deltaTime;
            if (next > RobotMotionRecordingStore.MaximumDurationSeconds)
            {
                // Keep the last genuinely observed frame. Do not label a pose
                // sampled after the limit with a fictitious earlier timestamp.
                elapsed = buffer.samples[buffer.samples.Count - 1].timeSeconds;
                FinishBuffer("Maximum recording duration reached", out _); return;
            }
            elapsed = next;
            float sampleTime = (float)elapsed;
            float previousTime = buffer.samples[buffer.samples.Count - 1].timeSeconds;
            if (elapsed - previousTime + 1e-7 < SamplePeriod && elapsed < RobotMotionRecordingStore.MaximumDurationSeconds) return;
            if (!TryCurrentAngles(out float[] angles, out string reason))
            { FinishBuffer("Recording stopped: " + reason, out _); return; }
            AddObservedFrame(sampleTime, angles);
            if (buffer.samples.Count >= RobotMotionRecordingStore.MaximumSamples || elapsed >= RobotMotionRecordingStore.MaximumDurationSeconds)
                FinishBuffer("Recording limit reached", out _);
        }

        public bool StopAndSave(out string reason)
        {
            if (!IsRecording)
                return HasUnsavedRecording ? SavePending(out reason) : Fail("There is no pending recording to save.", out reason);
            if (automaticUpdate && Application.isPlaying)
            {
                AdvanceRealtime();
                // The final clock update may already have reached and saved the limit.
                if (!IsRecording)
                { if (HasUnsavedRecording) return SavePending(out reason); reason = null; return true; }
            }
            if (TryCurrentAngles(out float[] angles, out string captureError)) AddObservedFrame((float)elapsed, angles);
            bool saved = FinishBuffer(captureError == null ? null : "Stopped at last valid frame: " + captureError, out reason);
            return saved;
        }

        private void AddObservedFrame(float time, float[] angles)
        {
            var last = buffer.samples[buffer.samples.Count - 1];
            if (time == last.timeSeconds)
            { last.angles = angles; return; }
            if (time <= last.timeSeconds || buffer.samples.Count >= RobotMotionRecordingStore.MaximumSamples) return;
            buffer.samples.Add(new RobotMotionSample { timeSeconds = time, angles = angles });
        }

        private bool FinishBuffer(string prefix, out string reason)
        {
            IsRecording = false;
            bool saved = SavePending(out reason);
            if (!string.IsNullOrEmpty(prefix)) Status = prefix + ". " + Status;
            return saved;
        }

        private bool SavePending(out string reason)
        {
            if (buffer == null) return Fail("There is no pending recording to save.", out reason);
            buffer.savedUtc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture);
            if (!RobotMotionRecordingStore.Save(DirectoryPath, buffer, out reason))
            {
                Status = "Not saved; recording retained for Retry Save. " + reason;
                return false;
            }
            LastSavedId = buffer.id;
            LastSavedPath = Path.Combine(Path.GetFullPath(DirectoryPath), Guid.Parse(buffer.id).ToString("N") + ".json");
            elapsed = buffer.samples[buffer.samples.Count - 1].timeSeconds;
            Status = "Saved: " + buffer.name + " (" + elapsed.ToString("0.00", CultureInfo.InvariantCulture) + " s).";
            buffer = null; reason = null; return true;
        }

        // Explicit discard is available to the UI; no lifecycle event calls it.
        public bool DiscardUnsavedRecording(out string reason)
        {
            if (IsRecording) return Fail("Stop recording before discarding it.", out reason);
            buffer = null; elapsed = 0; Status = "Unsaved recording discarded."; reason = null; return true;
        }

        private void FinishForLifecycle(string cause)
        {
            if (!StopAndSave(out string reason)) Debug.LogWarning("MOTION_RECORDING_NOT_SAVED " + cause + ": " + reason);
            else Status = cause + ". " + Status;
        }

        private bool TryCurrentAngles(out float[] angles, out string reason)
        {
            angles = null;
            if (arm == null || arm.joints == null || arm.joints.Length != 5)
            { reason = "Five configured joints are required."; return false; }
            foreach (var joint in arm.joints)
                if (joint == null || !JointMotion.IsFinite(joint.currentAngle))
                { reason = "Every current joint command must be finite."; return false; }
            angles = arm.CurrentAngles(); reason = null; return true;
        }
        private bool Fail(string message, out string reason) { Status = reason = message; return false; }
    }
}
