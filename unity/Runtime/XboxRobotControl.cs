using System;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Tabletop Cartesian controller jogging with separate height/wrist controls and no hardware transport.</summary>
    [DisallowMultipleComponent]
    public sealed class XboxRobotControl : MonoBehaviour
    {
        public RobotArm arm;
        public bool readController = true, controllerEnabled = true;
        public float translationSpeedMillimeters = 35f, jointSpeedDegrees = 35f;
        [Range(0.5f, 5f)] public float controllerSpeedMultiplier = 3f;
        public bool Connected { get; private set; }
        public bool NeedsNeutral => needsNeutral;
        public bool RoutineStopInProgress { get; private set; }
        public string Status { get; private set; } = "Release sticks and triggers to begin";
        public int AcceptedMoves { get; private set; }
        private bool needsNeutral = true, ownsMotion, ownCommand, brakingOnRelease;
        private bool translating, rotatingBase, tilting, gripping;
        private int controllerIndex = -1;
        private float timer;
        private float[] commandPose;
        private RobotArm subscribedArm;
        private RobotArm speedArm;
        private float savedSpeedMultiplier, appliedSpeedMultiplier;
        private RobotControlPanel panel;
        private Func<Vector2, bool> previousHitTest, installedHitTest;
        private XboxInputFrame lastFrame;
        private GUIStyle wrapStyle;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            RobotArm robot = FindAnyObjectByType<RobotArm>();
            if (robot == null || robot.GetComponent<XboxRobotControl>() != null) return;
            robot.gameObject.AddComponent<XboxRobotControl>().arm = robot;
        }

        private void Start()
        {
            if (arm == null) arm = GetComponent<RobotArm>();
            Subscribe();
            panel = FindAnyObjectByType<RobotControlPanel>();
            if (panel != null)
            {
                previousHitTest = panel.AdditionalUiHitTest;
                installedHitTest = point => (!panel.PortfolioDashboardActive && HudRect.Contains(new Vector2(point.x, Screen.height - point.y))) ||
                    (previousHitTest != null && previousHitTest(point));
                panel.AdditionalUiHitTest = installedHitTest;
            }
        }

        private void Update()
        {
            if (!readController) return;
            XboxInputFrame frame = WindowsXboxInput.Read(controllerIndex);
            controllerIndex = frame.connected ? frame.controllerIndex : -1;
            bool focused = Application.isFocused;
#if UNITY_EDITOR
            focused &= UnityEditor.EditorWindow.focusedWindow != null &&
                UnityEditor.EditorWindow.focusedWindow.GetType().Name == "GameView";
#endif
            // Poll every frame, solve at most 30 times per second. STOP/disconnect bypass the timer.
            timer += Time.unscaledDeltaTime;
            if (!frame.connected || !focused || frame.Down(WindowsXboxInput.B) || frame.Neutral || timer >= 1f / 30f)
            {
                ProcessFrame(frame, Mathf.Min(timer, 0.05f), focused);
                timer = 0f;
            }
        }

        public void ProcessFrame(XboxInputFrame frame, float deltaTime, bool focused)
        {
            Subscribe(); lastFrame = frame; Connected = frame.connected;
            if (arm == null || arm.joints == null || arm.joints.Length != 5 || arm.endEffector == null || arm.robotOrigin == null)
            { ReleaseMotion(); needsNeutral = true; Status = "Assign the five-joint robot before using Xbox input"; return; }
            if (!frame.connected || !focused || !controllerEnabled || !JointMotion.IsFinite(deltaTime) || deltaTime < 0f)
            {
                ReleaseMotion(); needsNeutral = true;
                Status = !frame.connected ? "Connect your Xbox controller" : !controllerEnabled ? "Xbox control disabled" : "Click the simulation to control it";
                return;
            }
            if (frame.Down(WindowsXboxInput.B))
            {
                ReleaseMotion(); Own(() => arm.Stop());
                needsNeutral = true;
                Status = "STOP — release the controls to resume"; return;
            }
            if (needsNeutral)
            {
                if (frame.Neutral) { needsNeutral = false; Status = "Ready — move either stick"; }
                else Status = "Release sticks, triggers and buttons to begin";
                return;
            }
            Vector2 planar = ShapeStick(frame.leftStick);
            float baseYaw = -ShapeAxis(frame.rightStick.x);
            float height = ShapeAxis(frame.rightStick.y);
            Vector3 translation = Vector3.ClampMagnitude(new Vector3(planar.x, height, planar.y), 1);
            // Sideways right-stick input directly turns the base; it never requests IK.
            float wrist = Mathf.Clamp((frame.Down(WindowsXboxInput.RB) ? 1f : 0f) -
                (frame.Down(WindowsXboxInput.LB) ? 1f : 0f), -1, 1);
            // The CAD linkage closes with increasing driver angle.
            float grip = frame.rightTrigger - frame.leftTrigger;
            bool hasInput = translation.sqrMagnitude > 0f || baseYaw != 0f || wrist != 0f || grip != 0f;
            if (!hasInput) { BrakeAfterRelease(); return; }
            brakingOnRelease = false;
            if (!ownsMotion)
            {
                StopForController(); // Cancel existing sequence/follow without disarming a verified hardware session.
                commandPose = arm.CurrentAngles(); ownsMotion = true;
                // Sequence cancellation can restore its speed during Stop, so capture afterwards.
                speedArm = arm; savedSpeedMultiplier = arm.speedMultiplier;
            }
            float speed = JointMotion.IsFinite(controllerSpeedMultiplier) ? Mathf.Clamp(controllerSpeedMultiplier, 0.5f, 5f) : 3f;
            // Preserve existing feel through100; above100, requests must keep up
            // with the selected hardware cap instead of retaining the old105deg/s feed.
            if (!float.IsPositiveInfinity(arm.externalSpeedMultiplierLimit))
            {
                float maximumJointSpeed = 0f;
                foreach (var joint in arm.joints) maximumJointSpeed = Mathf.Max(maximumJointSpeed, joint.maximumSpeed);
                speed *= Mathf.Max(1f, arm.externalSpeedMultiplierLimit * maximumJointSpeed / 100f);
            }
            appliedSpeedMultiplier = savedSpeedMultiplier * speed;
            speedArm.speedMultiplier = appliedSpeedMultiplier;
            // Each control group releases independently while another stays held.
            // Resumed input extends the braking destination, never the discarded
            // old lead or a behind-velocity target that makes a joint reverse back.
            ChangeInputGroup(0, 1, translating || rotatingBase, translation.sqrMagnitude > 0 || baseYaw != 0);
            ChangeInputGroup(1, 2, translating, translation.sqrMagnitude > 0);
            ChangeInputGroup(3, 1, tilting, wrist != 0);
            ChangeInputGroup(4, 1, gripping, grip != 0);
            translating = translation.sqrMagnitude > 0; rotatingBase = baseYaw != 0; tilting = wrist != 0; gripping = grip != 0;
            float dt = Mathf.Clamp(deltaTime, 0f, 0.05f);
            try
            {
                ForwardKinematics.Snapshot snapshot = ForwardKinematics.Capture(arm);
                float gripperCommand = commandPose[4];
                float[] seed = snapshot.Clamp(commandPose);
                float[] next = (float[])seed.Clone();
                // Limit target lead independently: a slower joint must not freeze the others.
                for (int i = 0; i < 4; i++)
                {
                    float maximumLead = MaximumTargetLead(arm.joints[i], arm);
                    snapshot.minimumAngles[i] = Mathf.Max(snapshot.minimumAngles[i], arm.joints[i].currentAngle - maximumLead);
                    snapshot.maximumAngles[i] = Mathf.Min(snapshot.maximumAngles[i], arm.joints[i].currentAngle + maximumLead);
                    next[i] = Mathf.Clamp(next[i], snapshot.minimumAngles[i], snapshot.maximumAngles[i]);
                }
                next[3] = Mathf.Clamp(next[3] + wrist * jointSpeedDegrees * speed * dt, snapshot.minimumAngles[3], snapshot.maximumAngles[3]);
                bool limited = false;
                if (translation.sqrMagnitude > 0)
                {
                    var result = CartesianArmJog.Step(snapshot, next, translation * translationSpeedMillimeters * speed * dt);
                    next = result.angles; limited = result.limited;
                }
                // Compose yaw after any independently requested table/height move, so
                // IK cannot compensate for yaw by changing the shoulder or elbow.
                next[0] = Mathf.Clamp(next[0] + baseYaw * jointSpeedDegrees * speed * dt,
                    snapshot.minimumAngles[0], snapshot.maximumAngles[0]);
                next[4] = arm.joints[4].Clamp(gripperCommand + grip * jointSpeedDegrees * speed * dt);
                bool accepted = false;
                for (int i = 0; i < 5; i++) accepted |= Mathf.Abs(next[i] - commandPose[i]) > .00001f;
                Status = !accepted ? "Reach limit — try forward/back or lower the gripper" : limited ?
                    "Near reach limit — try forward/back or lower" : rotatingBase && !translating && !tilting && !gripping ?
                    "Rotating base only — release to stop" : "Moving gripper — release to stop";
                commandPose = next;
                Own(() => arm.MoveToPose(commandPose));
                // Marker follows the achievable target, never an accumulating unreachable request.
                if (arm.target != null) arm.target.position = snapshot.Evaluate(snapshot.Clamp(commandPose));
                if (accepted) AcceptedMoves++;
            }
            catch (ArgumentException exception)
            {
                ReleaseMotion(); needsNeutral = true; Status = "Movement paused: " + exception.Message;
            }
        }

        // WindowsXboxInput already applies a radial deadzone. Square the remaining
        // magnitude for fine positioning without reducing the full-stick speed.
        public static Vector2 ShapeStick(Vector2 value)
        {
            value = Vector2.ClampMagnitude(value, 1f);
            return value * value.magnitude;
        }
        public static float ShapeAxis(float value)
        { value = Mathf.Clamp(value, -1f, 1f); return value * Mathf.Abs(value); }

        // Enough target lead to attain the configured velocity without spending every
        // update braking toward a nearby target. Still bounded and cleared on release.
        public static float MaximumTargetLead(RobotJoint joint, RobotArm robot)
        {
            bool external = !float.IsPositiveInfinity(robot.externalSpeedMultiplierLimit);
            float speed = Mathf.Max(0, robot.speedMultiplier);
            if (external) speed = Mathf.Min(speed, Mathf.Max(0, robot.externalSpeedMultiplierLimit));
            float accelerationScale = external ? speed : speed * speed;
            float velocity = joint.maximumSpeed * speed;
            float brakingDistance = velocity * velocity / (2f * Mathf.Max(.01f, joint.acceleration * accelerationScale));
            // Above100 allow one maximum50ms controller interval beyond braking
            // distance, so inter-update travel does not trigger premature braking.
            // Actual targets and release destinations retain the joint bounds.
            bool highSpeed = external && velocity > 100f;
            float leadCap = highSpeed ? 110f : external && velocity > 60f ? 24f : 16f;
            float lead = Mathf.Clamp(brakingDistance + 2f + (highSpeed ? velocity * .05f : 0f), 6f, leadCap);
            return Mathf.Min(lead, Mathf.Max(0f, joint.maximumAngle - joint.minimumAngle));
        }

        // Normal stick/trigger release brakes with the same acceleration used by
        // RobotArm.Tick. Emergency stops still use ReleaseMotion/RobotArm.Stop.
        public static float ReleasedTarget(RobotJoint joint, RobotArm robot)
        {
            bool external = !float.IsPositiveInfinity(robot.externalSpeedMultiplierLimit);
            float speed = Mathf.Max(0, robot.speedMultiplier);
            if (external) speed = Mathf.Min(speed, Mathf.Max(0, robot.externalSpeedMultiplierLimit));
            float scale = external ? speed : speed * speed;
            float acceleration = joint.acceleration * scale;
            float velocity = joint.CurrentVelocity;
            if (!JointMotion.IsFinite(acceleration) || acceleration <= 0 || !JointMotion.IsFinite(velocity))
                return joint.Clamp(joint.currentAngle);
            return joint.Clamp(joint.currentAngle + velocity * Mathf.Abs(velocity) / (2f * acceleration));
        }

        private void ChangeInputGroup(int first, int count, bool wasActive, bool active)
        {
            if (!wasActive || active) return;
            for (int i = first; i < first + count; i++)
                commandPose[i] = ReleasedTarget(arm.joints[i], arm);
        }

        private void BrakeAfterRelease()
        {
            if (ownsMotion && speedArm != null)
            {
                if (!brakingOnRelease)
                {
                    brakingOnRelease = true;
                    translating = rotatingBase = tilting = gripping = false;
                    var stopPose = new float[arm.joints.Length];
                    for (int i = 0; i < stopPose.Length; i++) stopPose[i] = ReleasedTarget(arm.joints[i], arm);
                    commandPose = stopPose;
                    Own(() => arm.MoveToPose(stopPose));
                }
                if (!arm.IsAtTarget) { Status = "Controls released — slowing to a stop (B stops immediately)"; return; }
                RestoreSpeed();
                ownsMotion = brakingOnRelease = false; commandPose = null;
            }
            Status = "Ready — controls released, arm stopped";
        }

        private void Own(Action action) { ownCommand = true; try { action(); } finally { ownCommand = false; } }
        private void StopForController(RobotArm stoppedArm = null)
        {
            RoutineStopInProgress = true;
            try { Own(() => (stoppedArm != null ? stoppedArm : arm).Stop()); } finally { RoutineStopInProgress = false; }
        }
        private void ReleaseMotion()
        {
            if (ownsMotion && speedArm != null) StopForController(speedArm);
            RestoreSpeed();
            ownsMotion = brakingOnRelease = false; commandPose = null;
            translating = rotatingBase = tilting = gripping = false;
        }
        private void RestoreSpeed()
        {
            // Preserve a newer speed assigned by another control or a sequence.
            if (speedArm != null && Mathf.Approximately(speedArm.speedMultiplier, appliedSpeedMultiplier))
                speedArm.speedMultiplier = savedSpeedMultiplier;
            speedArm = null;
        }
        private void ExternalCommand()
        {
            if (ownCommand) return;
            // A streaming replay updates its target every frame. Requiring a new
            // release for each sample would erase a neutral frame before the next
            // controller input can take over. Initial playback's routine Stop and
            // every other external command still require an actual neutral release.
            bool replaySample=MotionRecordingPlayer.IsPlaybackCommandFor(arm);
            RestoreSpeed();
            ownsMotion = brakingOnRelease = false; commandPose = null;
            translating = rotatingBase = tilting = gripping = false;
            if(!replaySample)needsNeutral=true;
            Status = replaySample&&!needsNeutral ? "Recording playing — move the controller to take over" :
                "Mouse/automatic controls active — release controller before taking over";
        }
        private void Subscribe()
        {
            if (subscribedArm == arm) return;
            ReleaseMotion(); needsNeutral = true;
            Unsubscribe(); subscribedArm = arm;
            if (subscribedArm == null) return;
            subscribedArm.MotionCommandIssued += ExternalCommand;
            subscribedArm.SimulationStopped += ExternalCommand;
        }
        private void Unsubscribe()
        {
            if (subscribedArm == null) return;
            subscribedArm.MotionCommandIssued -= ExternalCommand;
            subscribedArm.SimulationStopped -= ExternalCommand;
            subscribedArm = null;
        }
        private void OnDisable() { ReleaseMotion(); needsNeutral = true; Unsubscribe(); }
        private void OnDestroy()
        {
            ReleaseMotion(); Unsubscribe();
            if (panel != null && panel.AdditionalUiHitTest == installedHitTest) panel.AdditionalUiHitTest = previousHitTest;
        }
        private Rect HudRect
        {
            get
            {
                float left = 352f, right = Screen.width >= 1280 ? 302f : 12f;
                float width = Mathf.Min(600f, Mathf.Max(200f, Screen.width - left - right));
                return new Rect(left + Mathf.Max(0f, (Screen.width - left - right - width) * 0.5f), Screen.height - 188f, width, 176f);
            }
        }
        public void SetEnabledFromUi(bool value)
        {
            if (controllerEnabled == value) return;
            controllerEnabled = value; ReleaseMotion(); needsNeutral = true;
        }
        private void OnGUI()
        {
            if (panel != null && panel.PortfolioDashboardActive) return;
            if (wrapStyle == null) wrapStyle = new GUIStyle(GUI.skin.label) { wordWrap = true };
            GUILayout.BeginArea(HudRect, GUI.skin.box);
            GUILayout.BeginHorizontal();
            var hardwareLink = arm != null ? arm.GetComponent<RobotHardwareLink>() : null;
            GUILayout.Label(hardwareLink != null && hardwareLink.FollowActive
                ? "XBOX CONNECTED — REAL ARM FOLLOWING" : Connected ? "XBOX CONNECTED — UNITY CONTROLS" : "XBOX — UNITY CONTROLS");
            bool enabledNow = GUILayout.Toggle(controllerEnabled, "Enabled", GUILayout.Width(80));
            SetEnabledFromUi(enabledNow);
            GUILayout.EndHorizontal();
            GUILayout.Label("LEFT STICK: move across table (X/Z)  |  RIGHT STICK: sideways = base, up/down = height\nLB/RB: tilt wrist  |  LT: open  |  RT: close  |  B: STOP", wrapStyle);
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Speed {controllerSpeedMultiplier:0.0}x", GUILayout.Width(90));
            controllerSpeedMultiplier = GUILayout.HorizontalSlider(controllerSpeedMultiplier, 0.5f, 5f);
            GUILayout.Label($"{translationSpeedMillimeters * controllerSpeedMultiplier:0} mm/s", GUILayout.Width(80));
            GUILayout.EndHorizontal();
            GUILayout.Label(Status, wrapStyle);
            GUILayout.Label("Small stick movements = fine control. Release = stop. Right-stick sideways rotates only the base.", wrapStyle);
            GUILayout.EndArea();
        }
    }
}
