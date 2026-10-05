using System;
using System.Linq;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Explicit once-only replay of one continuous recording; never arms or sends transport commands.</summary>
    [DisallowMultipleComponent]
    public sealed class MotionRecordingPlayer : MonoBehaviour
    {
        public RobotArm arm;
        public bool automaticUpdate = true;
        public bool IsPlaying { get; private set; }
        public bool IssuingReplayCommand => ownCommand && IsPlaying;
        public bool PlayingOnHardware { get; private set; }
        public bool MovingToStart { get; private set; }
        public float ElapsedSeconds => (float)timeline;
        public float DurationSeconds { get; private set; }
        public string Status { get; private set; } = "Select a motion recording to play once.";
        public string RecordingName { get; private set; }
        private RobotMotionRecording playback;
        private RobotArm subscribedArm;
        private RobotHardwareLink link;
        private bool ownCommand, controllerSeen;
        private int segment;
        private double timeline;
        private float[] bounds, commanded;
        private static RobotArm routineStopArm;
        private static RobotArm playbackCommandArm;
        public static bool IsRoutineStopFor(RobotArm candidate) => candidate != null && routineStopArm == candidate;
        public static bool IsPlaybackCommandFor(RobotArm candidate) => candidate != null && playbackCommandArm == candidate;
        private void Start() { if (arm == null) arm = GetComponent<RobotArm>(); Bind(); }
        private void OnEnable() { Bind(); }
        private void OnDisable() { if (IsPlaying) Stop(); Unbind(); }
        private void OnApplicationFocus(bool focused) { if (!focused && IsPlaying) Stop("Playback stopped when the window lost focus."); }
        private void LateUpdate() { if (automaticUpdate) Tick(Time.deltaTime); }
        public void Bind()
        {
            if (subscribedArm == arm) return;
            if (IsPlaying) { Cancel("The recording's arm changed."); if (subscribedArm != null) subscribedArm.Stop(); }
            Unbind(); subscribedArm = arm;
            if (subscribedArm != null)
            { subscribedArm.MotionCommandIssued += OnManualCommand; subscribedArm.SimulationStopped += OnStopped; }
        }
        private void Unbind()
        {
            if (subscribedArm != null)
            { subscribedArm.MotionCommandIssued -= OnManualCommand; subscribedArm.SimulationStopped -= OnStopped; }
            subscribedArm = null;
        }

        public bool PlayOnce(RobotMotionRecording recording, bool hardware, out string reason)
        {
            Bind();
            if (IsPlaying) return Fail("Stop the current recording before playing another.", out reason);
            if (!RobotMotionRecordingStore.Validate(recording, out reason)) { Status = reason; return false; }
            if (arm == null || arm.joints == null || arm.joints.Length != 5 || arm.joints.Any(j => j == null))
                return Fail("Five configured joints are required.", out reason);
            var capture = arm.GetComponent<MotionRecordingCapture>();
            if (capture != null && capture.IsRecording) return Fail("Stop recording before playing a clip.", out reason);
            link = arm.GetComponent<RobotHardwareLink>();
            if (!hardware && link != null && link.FollowActive)
                return Fail("Hold Robot link before simulation-only playback.", out reason);
            foreach (var sample in recording.samples)
                if (!ValidatePose(sample.angles, hardware, out reason)) { Status = reason; return false; }
            var immutable = RobotMotionRecordingStore.DeepClone(recording);
            RoutineStop(() =>
            {
                foreach (var ik in FindObjectsByType<InverseKinematics>(FindObjectsSortMode.None)) if (ik.arm == arm) ik.Cancel();
                foreach (var planner in FindObjectsByType<MotionPlanner>(FindObjectsSortMode.None)) if (planner.arm == arm && planner.IsPlaying) planner.Stop();
                foreach (var sequence in FindObjectsByType<PoseSequence>(FindObjectsSortMode.None)) if (sequence.arm == arm && sequence.IsPlaying) sequence.Stop();
                foreach (var old in FindObjectsByType<PickAndPlaceRoutine>(FindObjectsSortMode.None)) if (old.arm == arm && old.IsPlaying) old.CancelForManualControl();
                arm.Stop();
            });
            arm.instantMode = false; foreach (var joint in arm.joints) joint.instantMode = false;
            playback = immutable; PlayingOnHardware = hardware; IsPlaying = MovingToStart = true;
            timeline = 0; DurationSeconds = playback.samples[playback.samples.Count - 1].timeSeconds;
            RecordingName = playback.name; segment = 0; SaveBounds();
            var controller = arm.GetComponent<XboxRobotControl>(); controllerSeen = controller != null && controller.Connected;
            Command(playback.samples[0].angles);
            Status = "Moving smoothly to the start of " + RecordingName + ".";
            reason = null; return true;
        }

        public void Tick(float deltaTime)
        {
            Bind(); if (!IsPlaying || !JointMotion.IsFinite(deltaTime) || deltaTime <= 0) return;
            float dt = Mathf.Min(deltaTime, .1f); // A stalled frame cannot seek ahead through the recording.
            var controller = arm.GetComponent<XboxRobotControl>();
            if (controllerSeen && (controller == null || !controller.Connected)) { Stop("Controller disconnected; playback stopped."); return; }
            controllerSeen |= controller != null && controller.Connected;
            string reason = null;
            if ((!PlayingOnHardware && link != null && link.FollowActive) ||
                !ValidatePose(commanded, PlayingOnHardware, out reason))
            { Stop("Playback stopped: " + (reason ?? "Robot link changed mode.")); return; }
            if (BoundsChanged())
            {
                foreach (var sample in playback.samples)
                    if (!ValidatePose(sample.angles, PlayingOnHardware, out reason)) { Stop("Playback stopped: " + reason); return; }
                SaveBounds();
            }
            if (MovingToStart)
            {
                if (!Arrived(playback.samples[0].angles)) return;
                MovingToStart = false; Status = PlaybackLabel(); return;
            }
            if (timeline >= DurationSeconds)
            {
                if (!Arrived(playback.samples[playback.samples.Count - 1].angles))
                { Status = "Finishing " + RecordingName + "; waiting for final commands to arrive."; return; }
                bool hardware = PlayingOnHardware;
                Cancel("Recording complete; holding final position.");
                if (hardware && link != null) link.Hold("Motion recording complete");
                return;
            }
            while (segment + 1 < playback.samples.Count - 1 && timeline >= playback.samples[segment + 1].timeSeconds) segment++;
            var a = playback.samples[segment]; var b = playback.samples[segment + 1];
            double span = (double)b.timeSeconds - a.timeSeconds;
            bool pause = Enumerable.Range(0, 5).All(i => Mathf.Abs(b.angles[i] - a.angles[i]) <= .0001f);
            // A recorded pause starts after acquisition, including command telemetry in hardware mode.
            if (pause && !Arrived(a.angles)) { Status = "Waiting at a recorded pause in " + RecordingName + "."; return; }
            double rate = 1;
            for (int i = 0; i < 5; i++)
            {
                double slope = Math.Abs((double)b.angles[i] - a.angles[i]) / span;
                if (slope > .00001) rate = Math.Min(rate, EffectiveJointSpeed(i) / slope);
            }
            double remaining = b.timeSeconds - timeline;
            double step = Math.Min(dt * Math.Max(0, Math.Min(1, rate)), remaining);
            double nextTime = timeline;
            float[] next = commanded;
            bool accepted = false;
            for (int attempt = 0; attempt < 10 && step > 0; attempt++, step *= .5)
            {
                // Double precision allows even very short valid sample intervals to be
                // stretched without skipping their endpoint or freezing below a float epsilon.
                nextTime = step >= remaining ? b.timeSeconds : timeline + step;
                float fraction = Mathf.Clamp01((float)((nextTime - a.timeSeconds) / span));
                next = new float[5]; bool leads = false;
                for (int i = 0; i < 5; i++)
                {
                    next[i] = Mathf.Lerp(a.angles[i], b.angles[i], fraction);
                    leads |= Mathf.Abs(next[i] - arm.joints[i].currentAngle) > XboxRobotControl.MaximumTargetLead(arm.joints[i], arm);
                }
                if (!leads) { accepted = true; break; }
            }
            if (!accepted) { Status = "Following " + RecordingName + " at the configured joint speed."; return; }
            if (!ValidatePose(next, PlayingOnHardware, out reason)) { Stop("Playback stopped: " + reason); return; }
            timeline = nextTime; Command(next); Status = PlaybackLabel();
        }

        private float EffectiveJointSpeed(int channel)
        {
            float multiplier = Mathf.Max(0, arm.speedMultiplier);
            if (!float.IsPositiveInfinity(arm.externalSpeedMultiplierLimit)) multiplier = Mathf.Min(multiplier, Mathf.Max(0, arm.externalSpeedMultiplierLimit));
            return arm.joints[channel].maximumSpeed * multiplier;
        }
        private bool Arrived(float[] pose) => arm.IsAtTarget && Enumerable.Range(0, 5).All(i => Mathf.Abs(arm.joints[i].currentAngle - pose[i]) <= .02f) &&
            (!PlayingOnHardware || (link != null && link.IsPlaybackPoseReported(pose)));
        private bool ValidatePose(float[] pose, bool hardware, out string reason)
        {
            if (pose == null || pose.Length != 5) { reason = "Recording requires five joint tracks."; return false; }
            for (int i = 0; i < 5; i++)
                if (!JointMotion.IsFinite(pose[i]) || pose[i] < arm.joints[i].minimumAngle || pose[i] > arm.joints[i].maximumAngle)
                { reason = "Recorded motion exceeds current bounds for " + arm.joints[i].jointName + "; no clamping was applied."; return false; }
            if (hardware)
            {
                if (link == null) { reason = "Enable Robot link before real-arm playback."; return false; }
                return link.TryValidatePlaybackPose(pose, out reason);
            }
            reason = null; return true;
        }
        private void Command(float[] pose)
        {
            commanded = (float[])pose.Clone(); ownCommand = true;
            var previous = playbackCommandArm; playbackCommandArm = arm;
            try { arm.MoveToPose(commanded); } finally { ownCommand = false; playbackCommandArm = previous; }
        }
        private string PlaybackLabel() => (PlayingOnHardware ? "REAL ARM" : "SIMULATION ONLY") + " — " + RecordingName +
            " " + ElapsedSeconds.ToString("0.0") + " / " + DurationSeconds.ToString("0.0") + " s";
        private void SaveBounds() { bounds = arm.joints.SelectMany(j => new[] {j.minimumAngle, j.maximumAngle}).ToArray(); }
        private bool BoundsChanged() => bounds == null || Enumerable.Range(0, 5).Any(i => bounds[i * 2] != arm.joints[i].minimumAngle || bounds[i * 2 + 1] != arm.joints[i].maximumAngle);
        public void Stop() => Stop("Playback stopped; select Play once to restart.");
        private void Stop(string message) { Cancel(message); if (arm != null) arm.Stop(); }
        public void CancelForManualControl()
        { if (!IsPlaying) return; Cancel("Manual control cancelled playback."); RoutineStop(() => arm.Stop()); }
        private void OnManualCommand() { if (!ownCommand && IsPlaying) CancelForManualControl(); }
        private void OnStopped() { if (IsPlaying && !IsRoutineStopFor(arm)) Cancel("Playback stopped; select Play once to restart."); }
        private void Cancel(string message)
        { IsPlaying = PlayingOnHardware = MovingToStart = false; playback = null; commanded = null; bounds = null; Status = message; }
        private void RoutineStop(Action action)
        { var previous = routineStopArm; routineStopArm = arm; try { action(); } finally { routineStopArm = previous; } }
        private bool Fail(string message, out string reason) { Status = reason = message; return false; }
    }
}
