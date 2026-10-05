using System;
using System.Collections.Generic;
using UnityEngine;

namespace CareerFair.Robot
{
    [DisallowMultipleComponent]
    public sealed class PoseSequence : MonoBehaviour
    {
        [Serializable]
        public sealed class Step
        {
            public RobotPose pose = new RobotPose();
            [Range(0.05f, 4f)] public float speedMultiplier = 1f;
            [Min(0f)] public float pauseSeconds;
        }

        public RobotArm arm;
        public List<Step> steps = new List<Step>();
        public bool automaticUpdate = true;
        public bool loop;
        public bool IsPlaying { get; private set; }
        public int CurrentStepIndex { get; private set; } = -1;
        public string LastError { get; private set; } = "";
        private RobotArm subscribedArm;
        private float originalSpeed = 1f;
        private float pauseRemaining;
        private bool pausing;
        private bool issuingOwnCommand;

        private void OnEnable() { Subscribe(); }
        private void OnDisable() { CancelPlayback(); Unsubscribe(); }
        private void LateUpdate() { if (automaticUpdate) Tick(Time.deltaTime); }

        public void AddCurrentPose(string name)
        {
            if (arm == null) throw new InvalidOperationException("Assign the simulated arm first.");
            steps.Add(new Step { pose = RobotPose.Capture(arm, name) });
        }

        public void Clear() { Stop(); steps.Clear(); }

        public void Play()
        {
            if (arm == null) throw new InvalidOperationException("Assign the simulated arm first.");
            Stop(); Subscribe(); LastError = "";
            if (steps == null || steps.Count == 0) return;
            // Validate every step before moving, so a bad later pose cannot start a partial sequence.
            foreach (Step step in steps)
            {
                if (step == null || step.pose == null) throw new ArgumentException("A sequence step has no pose.");
                step.pose.AnglesFor(arm);
                if (!JointMotion.IsFinite(step.speedMultiplier) || step.speedMultiplier <= 0f ||
                    !JointMotion.IsFinite(step.pauseSeconds) || step.pauseSeconds < 0f)
                    throw new ArgumentException("Sequence speed and pause must be finite and nonnegative; speed must be positive.");
            }
            originalSpeed = arm.speedMultiplier;
            IsPlaying = true;
            CurrentStepIndex = 0;
            BeginCurrentStep();
        }

        public void Stop()
        {
            CancelPlayback();
            if (arm != null) arm.Stop();
        }

        public void Tick(float deltaTime)
        {
            if (!IsPlaying || !JointMotion.IsFinite(deltaTime) || deltaTime < 0f) return;
            if (arm == null || steps == null || CurrentStepIndex < 0 || CurrentStepIndex >= steps.Count)
            {
                LastError = "Sequence configuration changed during playback.";
                Stop(); return;
            }
            if (!pausing)
            {
                if (!arm.IsAtTarget) return;
                pausing = true;
                pauseRemaining = Mathf.Max(0f, steps[CurrentStepIndex].pauseSeconds);
            }
            // A pause starts after the arrival tick, so it is never shorter than requested.
            else pauseRemaining -= deltaTime;
            if (pauseRemaining > 0f) return;
            CurrentStepIndex++;
            if (CurrentStepIndex >= steps.Count)
            {
                if (loop) CurrentStepIndex = 0;
                else { CancelPlayback(); return; }
            }
            try { BeginCurrentStep(); }
            catch (Exception exception) { LastError = exception.Message; Stop(); }
        }

        private void BeginCurrentStep()
        {
            Step step = steps[CurrentStepIndex];
            arm.speedMultiplier = originalSpeed * step.speedMultiplier;
            pausing = false; pauseRemaining = 0f;
            issuingOwnCommand = true;
            try { step.pose.Apply(arm); }
            finally { issuingOwnCommand = false; }
        }

        private void ExternalCommand()
        {
            if (!issuingOwnCommand) CancelPlayback();
        }

        private void CancelPlayback()
        {
            if (IsPlaying && arm != null) arm.speedMultiplier = originalSpeed;
            IsPlaying = false; CurrentStepIndex = -1; pausing = false; pauseRemaining = 0f;
        }

        private void Subscribe()
        {
            if (subscribedArm == arm) return;
            Unsubscribe(); subscribedArm = arm;
            if (subscribedArm == null) return;
            subscribedArm.SimulationStopped += CancelPlayback;
            subscribedArm.MotionCommandIssued += ExternalCommand;
        }

        private void Unsubscribe()
        {
            if (subscribedArm == null) return;
            subscribedArm.SimulationStopped -= CancelPlayback;
            subscribedArm.MotionCommandIssued -= ExternalCommand;
            subscribedArm = null;
        }
    }
}
