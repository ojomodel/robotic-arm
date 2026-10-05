using System;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;
using CareerFair.Robot.Hardware;

namespace CareerFair.Robot
{
    /// <summary>Explicit, disarmed-by-default USB link. Never infers physical direction from CAD.</summary>
    [DefaultExecutionOrder(80)]
    public sealed class RobotHardwareLink : MonoBehaviour
    {
        public RobotArm arm;
        public string port = "COM4";
        public float physicalSpeed = RobotSpeedSettings.Default;
        public bool FollowActive => (mode == "Follow" || mode == "DirectFollow") && deviceArmed;
        public bool DirectControlActive => mode == "DirectFollow" && deviceArmed;
        public string Status { get; private set; } = "Robot disconnected";
        public string LastConnectionError { get; private set; }
        public string LastStopReason => holdReason;
        public bool Connected => transport != null && transport.IsConnected && handshake;
        public float EffectivePhysicalSpeed => Mathf.Min(RobotSpeedSettings.Clamp(physicalSpeed), firmwareMaximumSpeed);
        public float FirmwareMaximumSpeed => firmwareMaximumSpeed;
        public bool TimestampedMotionAvailable => trajectorySupported;
        public bool CanEditSpeed => !deviceArmed && !waitingArm && !waitingLimits && !holdPending && !releasePending;

        public bool SetRequestedSpeed(float value)
        {
            if (!CanEditSpeed || !RobotHardwareMapping.Finite(value)) return false;
            float normalized = RobotSpeedSettings.Clamp(value);
            if (normalized != physicalSpeed)
            {
                physicalSpeed = normalized;
                speedDirty = true; lastSpeedEditTime = Time.realtimeSinceStartup;
            }
            return true;
        }
        private IRobotUsbTransport transport;
        private RobotHardwareMapping mapping = new RobotHardwareMapping();
        private readonly VerifiedJointTravel verifiedTravel = new VerifiedJointTravel();
        private DirectOperatorTravel directTravel;
        private RobotJoint[] directJoints;
        private float[] directOriginalMinimum, directOriginalMaximum;
        private RobotArm followSpeedArm;
        private float originalFollowSpeed, appliedFollowSpeed;
        private bool speedDirty;
        private float lastSpeedEditTime;
        private string speedSettingsNotice;
        private const float LegacyMaximumSpeed = 30f, MotionQueryTimeout = 2f;
        private float firmwareMaximumSpeed = LegacyMaximumSpeed, motionQueryTime;
        private bool motionQueryPending;
        private bool trajectoryQueryPending, trajectorySupported;
        private float trajectoryQueryTime;
        private readonly MotionSampleClock sampleClock = new MotionSampleClock();
        private readonly PoseSendCadence poseCadence = new PoseSendCadence();
        private string[] ports = Array.Empty<string>();
        private readonly string[] names = { "Base", "Shoulder", "Elbow", "Wrist", "Gripper" };
        private float[] commands;
        private uint boot, sequence;
        private bool handshake, deviceArmed, outputsEnabled, physicalReady, handlingStop;
        private bool releasePending, releaseAcknowledged, supportedForRelease;
        private bool waitingArm, waitingLimits;
        private bool holdPending, holdAcknowledged, holdConfirmed, telemetryTimeoutReported;
        private int holdAttempts;
        private uint holdBoot;
        private float holdStartedTime, holdLastSentTime;
        private string holdReason;
        private const float HoldRetryInterval = .25f, HoldConfirmationTimeout = 1f;
        private const int MaximumHoldAttempts = 3;
        private string mode = "Hold", requestedMode;
        private float lastStateTime, lastHelloTime, lastPingTime, requestTime;
        private int selected;
        private bool inputWasEnabled, heldInput;
        private string[] lowFields = new string[5], highFields = new string[5];
        private string notice = "Connect only reads the ESP32. Movement requires Arm.";
        private GUIStyle wrap, toggleWrap;
        private bool disconnectionReported;
        private const float StateDiagnosticInterval = .25f;
        private float lastStateDiagnosticTime = float.NegativeInfinity;
        private string pendingStateDiagnostic, pendingStateFingerprint, lastStateFingerprint;
        private string MappingPath => Path.Combine(Application.persistentDataPath, "robot-hardware-mapping.json");
        private string SpeedSettingsPath => Path.Combine(Application.persistentDataPath, "robot-speed-settings.json");

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            var robot = FindAnyObjectByType<RobotArm>();
            if (robot != null && robot.GetComponent<RobotHardwareLink>() == null)
                robot.gameObject.AddComponent<RobotHardwareLink>().arm = robot;
        }
        private void Start()
        {
            if (!arm) arm = GetComponent<RobotArm>();
            if (arm) arm.SimulationStopped += OnSimulationStopped;
            RefreshPorts();
            try
            {
                if (File.Exists(MappingPath))
                {
                    var loaded = JsonUtility.FromJson<RobotHardwareMapping>(File.ReadAllText(MappingPath));
                    if (loaded != null && loaded.ValidateStructure(out _) &&
                        loaded.servoNeutral.SequenceEqual(mapping.servoNeutral) && loaded.modelHome.SequenceEqual(mapping.modelHome)) mapping = loaded;
                    else notice = "Saved mapping does not match this calibration reference; verify calibration again.";
                }
            }
            catch (Exception e) { notice = "Could not load saved calibration: " + e.Message; }
            LoadSpeedPreference();
            SyncFields();
        }
        private void LoadSpeedPreference()
        {
            var result = RobotSpeedSettings.Load(SpeedSettingsPath);
            physicalSpeed = result.Speed;
            speedSettingsNotice = result.Success ? null : result.Message;
            if (result.Success && (result.UsedDefault || result.WasClamped))
            { speedDirty = true; SaveSpeedPreference(); }
        }
        private void SaveSpeedPreference()
        {
            if (!speedDirty) return;
            var result = RobotSpeedSettings.Save(SpeedSettingsPath, physicalSpeed);
            speedDirty = false;
            speedSettingsNotice = result.Success ? null : result.Message;
            if (result.Success) physicalSpeed = result.Speed;
        }
        private void ApplyFollowSpeedLimit()
        {
            if (!arm || arm.joints == null || arm.joints.Any(j => j == null)) return;
            float maximum = arm.joints.Max(j => j.maximumSpeed);
            float ratio = maximum > 0 ? EffectivePhysicalSpeed / maximum : 1f;
            if (followSpeedArm != arm)
            {
                RestoreFollowSpeed();
                followSpeedArm = arm;
                originalFollowSpeed = arm.speedMultiplier;
            }
            // The degrees/s control applies to target and joint moves too. The
            // external bound still caps faster controller/replay requests.
            appliedFollowSpeed = Mathf.Max(JointMotion.IsFinite(originalFollowSpeed) ? originalFollowSpeed : 1f, ratio);
            arm.speedMultiplier = appliedFollowSpeed;
            arm.externalSpeedMultiplierLimit = ratio;
        }
        private void RestoreFollowSpeed()
        {
            if (followSpeedArm && Mathf.Approximately(followSpeedArm.speedMultiplier, appliedFollowSpeed))
                followSpeedArm.speedMultiplier = originalFollowSpeed;
            followSpeedArm = null;
        }
        private void RefreshPorts() { ports = UsbRobotTransport.GetPortNames(); }
        private void SyncFields()
        {
            for (int i = 0; i < 5; i++)
            {
                lowFields[i] = CF1Protocol.Number(mapping.minimumCommand[i]);
                highFields[i] = CF1Protocol.Number(mapping.maximumCommand[i]);
            }
        }
        private bool Send(string line)
        {
            if (transport == null || !transport.SendLine(line)) { notice = transport?.LastError ?? "No connection"; return false; }
            return true;
        }
        public void Connect()
        {
            Disconnect();
            LastConnectionError = null; disconnectionReported = false;
            transport = new UsbRobotTransport();
            if (!transport.Connect(port))
            {
                ReportTransportDisconnection(); Status = "Connection failed"; return;
            }
            lastHelloTime = Time.realtimeSinceStartup; lastStateTime = lastHelloTime;
            Status = "Waiting for CF1 firmware";
            Send("HELLO"); Send("STATUS");
        }
        public void Disconnect()
        {
            Hold("Disconnected"); transport?.Disconnect(); transport = null;
            ResetMotionCapability();
            SaveSpeedPreference();
            handshake = false; physicalReady = false; commands = null; boot = sequence = 0;
            releasePending = supportedForRelease = false;
            ResetHoldRequest();
            pendingStateDiagnostic = pendingStateFingerprint = lastStateFingerprint = null;
            lastStateDiagnosticTime = float.NegativeInfinity;
            Status = "Robot disconnected";
        }
        private void ReportTransportDisconnection()
        {
            if (disconnectionReported) return;
            disconnectionReported = true;
            LastConnectionError = string.IsNullOrWhiteSpace(transport?.LastError)
                ? "Port closed without a transport error description." : transport.LastError;
            string context = "port=" + port + " boot=" + boot + " seq=" + sequence +
                " focused=" + Application.isFocused + " runInBackground=" + Application.runInBackground;
            Hold("USB connection lost: " + LastConnectionError);
            ResetMotionCapability();
            handshake = false; Status = "USB disconnected";
            Debug.LogWarning("CF1_USB_DISCONNECTED utc=" + DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture) +
                " " + context + " reason=" + LastConnectionError);
        }
        private void QueueStateDiagnostic(CF1Message message)
        {
            string values = string.Join(" ", message.Commands.Select(a => a.ToString("F3", CultureInfo.InvariantCulture)));
            string state = "boot=" + message.Boot + " armed=" + (message.Armed ? 1 : 0) +
                " limits=" + (message.LimitsSet ? 1 : 0) + " outputs=" + (message.OutputsEnabled ? 1 : 0);
            string fingerprint = state + " commands=" + values;
            if (fingerprint == lastStateFingerprint)
            { pendingStateDiagnostic = pendingStateFingerprint = null; return; }
            pendingStateFingerprint = fingerprint;
            pendingStateDiagnostic = "CF1_STATE " + state + " seq=" + message.LastSequence + " commands=" + values;
        }
        private void FlushStateDiagnostic(float now)
        {
            if (pendingStateDiagnostic == null || now - lastStateDiagnosticTime < StateDiagnosticInterval) return;
            Debug.Log(pendingStateDiagnostic + " utc=" + DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture) +
                " (command telemetry, not measured position)");
            lastStateFingerprint = pendingStateFingerprint; lastStateDiagnosticTime = now;
            pendingStateDiagnostic = pendingStateFingerprint = null;
        }
        private void Update()
        {
            if (speedDirty && Time.realtimeSinceStartup - lastSpeedEditTime >= .35f)
                SaveSpeedPreference();
            if (directTravel == null && directJoints != null && !verifiedTravel.Active)
                RestoreDirectTravel();
            if (transport == null) return;
            for (int i = 0; i < 64 && transport.TryReadLine(out string line); i++) Receive(line);
            float now = Time.realtimeSinceStartup;
            TickMotionCapability(now);
            FlushStateDiagnostic(now);
            if (!transport.IsConnected) { ReportTransportDisconnection(); return; }
            TickHoldRequest(now);
            if (!transport.IsConnected || holdPending) return;
            if (!handshake)
            {
                if (now - lastHelloTime > 2f) { Send("HELLO"); Send("STATUS"); lastHelloTime = now; }
                return;
            }
            if (now - lastStateTime > .75f)
            {
                if (!telemetryTimeoutReported)
                {
                    telemetryTimeoutReported = true;
                    Debug.LogWarning("CF1_TELEMETRY_TIMEOUT boot=" + boot + " seq=" + sequence +
                        " age=" + (now - lastStateTime).ToString("F3", CultureInfo.InvariantCulture) +
                        " mode=" + mode + " focused=" + Application.isFocused +
                        " runInBackground=" + Application.runInBackground);
                }
                Hold("Command telemetry timed out"); handshake = false; return;
            }
            if ((waitingArm || waitingLimits) && now - requestTime > 2f) Hold("Arming timed out");
            if (!deviceArmed && !waitingArm && !waitingLimits) return;
            XboxInputFrame pad = WindowsXboxInput.Read();
            if (!Application.isFocused || !pad.connected || pad.Down(WindowsXboxInput.B))
            { Hold(!pad.connected ? "Controller disconnected" : "Held: focus lost or B pressed"); return; }
            if ((deviceArmed || waitingArm) && now - lastPingTime >= .1f) { Send("PING " + boot); lastPingTime = now; }
            if (FollowActive)
            {
                arm.instantMode = false;
                foreach (var joint in arm.joints) joint.instantMode = false;
                SendFollowPose(Time.realtimeSinceStartupAsDouble);
            }
        }
        private void SendFollowPose(double now)
        {
            if (!FollowActive || holdPending || !poseCadence.TryTake(now)) return;
            float[] target = null; string reason = "Direct execution bounds are unavailable";
            bool mapped = mode == "DirectFollow"
                ? directTravel != null && directTravel.TryMapPose(arm.CurrentAngles(), out target, out reason)
                : mapping.TryMapPose(arm.CurrentAngles(), out target, out reason);
            if (!mapped) { Hold(mode == "DirectFollow" && directTravel == null ? "Direct execution bounds are unavailable" : reason); return; }
            string verb = "POSE "; string timestamp = "";
            if (trajectorySupported)
            {
                if (!sampleClock.TryStamp(now, out ulong stamp)) { Hold("Motion sample clock stalled or reset; re-arm required"); return; }
                verb = "TPOSE "; timestamp = stamp.ToString(CultureInfo.InvariantCulture) + " ";
            }
            if (!NextSequence()) return;
            if (!Send(verb + boot + " " + sequence + " " + timestamp + string.Join(" ", target.Select(CF1Protocol.Number)))) Hold("Pose send failed");
        }
        private void Receive(string line)
        {
            if (!CF1Protocol.TryParse(line, out CF1Message message, out _)) return;
            string kind = message.Kind.ToString();
            if (kind == "Hello")
            {
                ResetMotionCapability();
                bool newBoot = boot != message.Boot;
                if (newBoot) { sequence = 0; ResetHoldRequest(); }
                boot = message.Boot; handshake = true; physicalReady = false; commands = null;
                releasePending = releaseAcknowledged = false; telemetryTimeoutReported = false;
                if (!mapping.NeutralMatches(message.Commands, out string reason))
                { Hold(reason); handshake = false; Status = "Calibration reference mismatch"; notice = reason; return; }
                Hold(newBoot ? "ESP32 started; re-arm is required" : "Handshake refreshed; re-arm is required");
                motionQueryPending = true; motionQueryTime = Time.realtimeSinceStartup;
                if (!Send("MOTION")) motionQueryPending = false;
                Send("STATUS");
            }
            else if (kind == "Motion")
            {
                TickMotionCapability(Time.realtimeSinceStartup);
                if (Connected && motionQueryPending && mode == "Hold" && !deviceArmed && !waitingArm && !waitingLimits)
                {
                    firmwareMaximumSpeed = message.MaximumMotionSpeed;
                    motionQueryPending = false;
                    if (firmwareMaximumSpeed == 100 || firmwareMaximumSpeed == 400)
                    {
                        trajectoryQueryPending = true; trajectoryQueryTime = Time.realtimeSinceStartup;
                        if (!Send("TRAJECTORY " + boot)) trajectoryQueryPending = false;
                    }
                }
            }
            else if (kind == "Trajectory")
            {
                TickMotionCapability(Time.realtimeSinceStartup);
                if (Connected && trajectoryQueryPending && message.Boot == boot && message.MaximumMotionSpeed == firmwareMaximumSpeed && mode == "Hold" &&
                    !deviceArmed && !waitingArm && !waitingLimits)
                { trajectorySupported = true; trajectoryQueryPending = false; }
            }
            else if (kind == "State")
            {
                // After a telemetry timeout the movement handshake is invalid, but
                // matching telemetry may still finish the already-requested stop.
                if ((!handshake && !holdPending) || message.Boot != boot) return;
                lastStateTime = Time.realtimeSinceStartup;
                telemetryTimeoutReported = false;
                commands = message.Commands;
                QueueStateDiagnostic(message);
                outputsEnabled = message.OutputsEnabled;
                bool releaseComplete = releasePending && releaseAcknowledged && !outputsEnabled && !message.Armed;
                if (releaseComplete)
                { releasePending = false; notice = "Servo signals disabled. Power can be switched with the arm supported. Recheck its pose before arming."; }
                if (message.LastSequence > sequence) sequence = message.LastSequence;
                // STATE lines queued before our HOLD may still report armed. They are not
                // permission to resume, nor a reason to enqueue another HOLD per line.
                if (holdPending)
                {
                    deviceArmed = false;
                    if (message.Boot == holdBoot && !message.Armed && (holdAcknowledged || releaseComplete))
                        ConfirmHold();
                    return;
                }
                deviceArmed = message.Armed;
                if (deviceArmed && waitingArm)
                {
                    if (requestedMode == "DirectFollow")
                    {
                        // No stale visual target is sent after ARM. The confirming
                        // telemetry sets the model and clears any pending Xbox gesture.
                        if (directTravel == null || !directTravel.TryMapTelemetry(commands, out float[] pose, out string reason))
                        { Hold("Direct start rejected: current command outside execution bounds"); return; }
                        StopSimulation(); arm.SetAnglesImmediate(pose); RestoreController();
                        var controller = GetComponent<XboxRobotControl>();
                        if (controller != null) controller.controllerEnabled = true;
                    }
                    waitingArm = false; mode = requestedMode; poseCadence.Reset(); sampleClock.Reset();
                    Status = mode == "DirectFollow" ? "DIRECT ARM CONTROL — UNCALIBRATED TRAVEL"
                        : mode == "Follow" ? "REAL ARM FOLLOWING UNITY" : "Calibration armed";
                }
                else if (deviceArmed && mode == "Hold")
                {
                    // A new armed report after confirmed HOLD is a separate stop episode.
                    ResetHoldRequest(); Hold("Unexpected armed state; held"); return;
                }
                else if (!deviceArmed && !waitingArm && mode != "Hold") Hold("ESP32 stopped; re-arm required");
            }
            else if (kind == "Ack" && message.Verb == "HOLD" && holdPending)
            { holdAcknowledged = true; }
            else if (kind == "Ack" && message.Verb == "RELEASE" && releasePending)
            { releaseAcknowledged = true; Send("STATUS"); }
            else if (kind == "Ack" && message.Verb == "LIMITS" && waitingLimits)
            {
                waitingLimits = false; BeginArm(requestedMode);
            }
            else if (kind == "Error")
            {
                TickMotionCapability(Time.realtimeSinceStartup);
                // Older firmware has no MOTION query. Only that expected response
                // during the disarmed query window is a capability fallback.
                if (message.ErrorCode == "COMMAND" && motionQueryPending && mode == "Hold" && !deviceArmed && !waitingArm && !waitingLimits)
                { motionQueryPending = false; return; }
                if (message.ErrorCode == "COMMAND" && trajectoryQueryPending && mode == "Hold" && !deviceArmed && !waitingArm && !waitingLimits)
                { trajectoryQueryPending = false; trajectorySupported = false; return; }
                Hold("ESP32: " + message.ErrorCode);
            }
        }
        private void ResetMotionCapability()
        {
            firmwareMaximumSpeed = LegacyMaximumSpeed; motionQueryPending = false; motionQueryTime = 0;
            trajectorySupported = trajectoryQueryPending = false; trajectoryQueryTime = 0;
            poseCadence.Reset(); sampleClock.Reset();
        }
        private void TickMotionCapability(float now)
        {
            if (motionQueryPending && now - motionQueryTime >= MotionQueryTimeout) motionQueryPending = false;
            if (trajectoryQueryPending && now - trajectoryQueryTime >= MotionQueryTimeout) trajectoryQueryPending = false;
        }
        private bool NextSequence()
        {
            if (sequence == uint.MaxValue) { Hold("Sequence exhausted; restart ESP32 with servo power off"); return false; }
            sequence++; return true;
        }
        private bool CanArm(out string reason)
        {
            TickMotionCapability(Time.realtimeSinceStartup);
            reason = "";
            if (!Connected || commands == null || Time.realtimeSinceStartup - lastStateTime > .5f) reason = "Wait for fresh ESP32 command telemetry.";
            else if (holdPending) reason = "Wait for the ESP32 to confirm HOLD before arming.";
            else if (motionQueryPending || trajectoryQueryPending) reason = "Checking firmware motion support; try again in a moment.";
            else if (!physicalReady) reason = "Confirm the physical arm matches the displayed commands before arming.";
            else if (releasePending) reason = "Wait for confirmation that servo signals are disabled.";
            else if (!Application.isFocused || !WindowsXboxInput.Read().connected) reason = "Focus the simulator and connect the Xbox controller to the PC.";
            return reason.Length == 0;
        }
        private void PauseController()
        {
            var controller = GetComponent<XboxRobotControl>();
            if (controller && !heldInput) { inputWasEnabled = controller.controllerEnabled; controller.controllerEnabled = false; heldInput = true; }
        }
        private void RestoreController()
        {
            var controller = GetComponent<XboxRobotControl>();
            if (controller && heldInput) controller.controllerEnabled = inputWasEnabled;
            heldInput = false;
        }
        private void BeginArm(string requested)
        {
            if (!CanArm(out string reason)) { Hold(reason); return; }
            ResetHoldRequest();
            requestedMode = requested; waitingArm = true; requestTime = Time.realtimeSinceStartup;
            if (!Send("ARM " + boot)) Hold("Arm command not sent");
        }
        private void BeginCalibration()
        {
            if (!CanArm(out string reason)) { notice = reason; return; }
            StopSimulation(); PauseController(); BeginArm("Calibration");
        }
        private void BeginFollow()
        {
            if (!CanArm(out string reason) || !mapping.HasVerifiedCalibration(out reason)) { notice = reason; return; }
            if (!mapping.TryMapTelemetry(commands, out float[] pose, out reason)) { notice = reason; return; }
            for (int i = 0; i < 5; i++)
                if (Mathf.Abs(arm.joints[i].Clamp(pose[i]) - pose[i]) > .01f) { notice = "Current robot command maps outside model travel: " + names[i]; return; }
            if (!verifiedTravel.TryApply(arm, mapping, pose, out reason)) { notice = reason; return; }
            ResetHoldRequest();
            StopSimulation(); arm.SetAnglesImmediate(pose); arm.instantMode = false;
            foreach (var joint in arm.joints) joint.instantMode = false;
            RestoreController();
            physicalSpeed = RobotSpeedSettings.Clamp(physicalSpeed);
            SaveSpeedPreference();
            ApplyFollowSpeedLimit();
            string limits = string.Join(" ", Enumerable.Range(0,5).Select(i => CF1Protocol.Number(mapping.minimumCommand[i]) + " " + CF1Protocol.Number(mapping.maximumCommand[i])));
            requestedMode = "Follow"; waitingLimits = true; requestTime = Time.realtimeSinceStartup;
            if (!Send("LIMITS " + boot + " " + limits + " " + CF1Protocol.Number(EffectivePhysicalSpeed))) Hold("Limits send failed");
        }
        private void BeginDirectControl()
        {
            if (!CanArm(out string reason)) { notice = reason; return; }
            if (deviceArmed || waitingArm || waitingLimits || verifiedTravel.Active || directTravel != null)
            { notice = "HOLD the current session before starting direct control."; return; }
            if (arm == null || arm.joints == null || arm.joints.Length != 5 || arm.joints.Any(j => j == null))
            { notice = "Five model joints are required."; return; }
            if (directJoints != null && !directJoints.SequenceEqual(arm.joints))
            { notice = "Model joints changed while a held pose was outside the original bounds; reopen the simulator."; return; }
            float[] originalMinimum = directOriginalMinimum ?? arm.joints.Select(j => j.minimumAngle).ToArray();
            float[] originalMaximum = directOriginalMaximum ?? arm.joints.Select(j => j.maximumAngle).ToArray();
            if (!DirectOperatorTravel.TryCreate(mapping, originalMinimum,
                originalMaximum, commands, out var execution, out float[] pose, out reason))
            { notice = reason; return; }

            // This is a temporary execution policy. The saved verified mapping and
            // all of its boundsVerified flags remain untouched.
            ResetHoldRequest(); StopSimulation(); PauseController();
            ApplyDirectTravel(execution);
            arm.SetAnglesImmediate(pose); arm.instantMode = false;
            foreach (var joint in arm.joints) joint.instantMode = false;
            // Keep the saved speed across reconnects and application restarts.
            // Older or unrecognized firmware remains capped at 30 degrees/s without
            // overwriting the saved operator preference.
            physicalSpeed = RobotSpeedSettings.Clamp(physicalSpeed);
            SaveSpeedPreference();
            ApplyFollowSpeedLimit();
            string limits = string.Join(" ", Enumerable.Range(0,5).Select(i => CF1Protocol.Number(execution.MinimumCommand(i)) + " " + CF1Protocol.Number(execution.MaximumCommand(i))));
            requestedMode = "DirectFollow"; waitingLimits = true; requestTime = Time.realtimeSinceStartup;
            notice = "Direct arm control allows up to 90 degrees each way from the calibration reference, clipped to 0..180 without wrapping. Use Xbox or target-ball follow; physical travel is uncalibrated.";
            if (!Send("LIMITS " + boot + " " + limits + " " + CF1Protocol.Number(EffectivePhysicalSpeed))) Hold("Direct limits send failed");
        }
        private void ApplyDirectTravel(DirectOperatorTravel execution)
        {
            directTravel = execution;
            if (directJoints == null)
            {
                directJoints = (RobotJoint[])arm.joints.Clone();
                directOriginalMinimum = directJoints.Select(j => j.minimumAngle).ToArray();
                directOriginalMaximum = directJoints.Select(j => j.maximumAngle).ToArray();
            }
            for (int i = 0; i < 5; i++)
            {
                directJoints[i].minimumAngle = execution.MinimumModelAngle(i);
                directJoints[i].maximumAngle = execution.MaximumModelAngle(i);
            }
        }
        private void RestoreDirectTravel()
        {
            bool pendingRestore = false;
            if (directJoints != null)
                for (int i = 0; i < directJoints.Length; i++)
                    if (directJoints[i] != null)
                    {
                        // Expanded operator travel can put the held pose outside
                        // the original CAD bounds. Restoring those bounds verbatim
                        // would make the next Tick snap the model away from it.
                        float heldLow = Mathf.Min(directJoints[i].currentAngle, directJoints[i].targetAngle);
                        float heldHigh = Mathf.Max(directJoints[i].currentAngle, directJoints[i].targetAngle);
                        directJoints[i].minimumAngle = Mathf.Min(directOriginalMinimum[i], heldLow);
                        directJoints[i].maximumAngle = Mathf.Max(directOriginalMaximum[i], heldHigh);
                        pendingRestore |= heldLow < directOriginalMinimum[i] || heldHigh > directOriginalMaximum[i];
                    }
            // Keep the original snapshot until the held model has returned inside
            // it. A subsequent Direct session must not recapture expanded bounds.
            if (!pendingRestore) { directJoints = null; directOriginalMinimum = directOriginalMaximum = null; }
            directTravel = null;
        }
        private void Jog(float delta)
        {
            if (!deviceArmed || mode != "Calibration" || !NextSequence()) return;
            Send("JOG " + boot + " " + sequence + " " + selected + " " + CF1Protocol.Number(delta));
        }
        private void StopSimulation()
        {
            if (!arm) return;
            handlingStop = true;
            try { arm.Stop(); } finally { handlingStop = false; }
        }
        private void OnSimulationStopped()
        {
            var controller = GetComponent<XboxRobotControl>();
            if (!handlingStop && !(controller && controller.RoutineStopInProgress) && !InverseKinematics.IsRoutineStopFor(arm)
                && !PickAndPlaceRoutine.IsRoutineStopFor(arm) && !MotionRecordingPlayer.IsRoutineStopFor(arm))
                Hold("Stopped from simulation controls");
        }
        public bool TryValidatePlaybackPose(float[] pose, out string reason)
        {
            reason = "Start Direct arm control or checked follow before playing a real-arm program.";
            if (!Connected || !FollowActive || holdPending || waitingArm || waitingLimits) return false;
            if (mode == "DirectFollow")
            {
                if (directTravel == null) { reason = "Direct execution bounds are unavailable."; return false; }
                return directTravel.TryMapPose(pose, out _, out reason);
            }
            return mapping.TryMapPose(pose, out _, out reason);
        }
        public bool IsPlaybackPoseReported(float[] pose)
        {
            if (!TryValidatePlaybackPose(pose, out _) || commands == null || commands.Length != 5 ||
                Time.realtimeSinceStartup - lastStateTime > .5f) return false;
            float[] expected;
            if (mode == "DirectFollow")
            {
                if (!directTravel.TryMapPose(pose, out expected, out _)) return false;
            }
            else if (!mapping.TryMapPose(pose, out expected, out _)) return false;
            for (int i = 0; i < 5; i++)
                if (!JointMotion.IsFinite(commands[i]) || Mathf.Abs(expected[i] - commands[i]) > .1f) return false;
            return true;
        }
        public void Hold(string reason)
        {
            poseCadence.Reset(); sampleClock.Reset();
            mode = "Hold"; deviceArmed = waitingArm = waitingLimits = false;
            physicalReady = false;
            if (arm) arm.externalSpeedMultiplierLimit = float.PositiveInfinity;
            StopSimulation(); RestoreFollowSpeed(); RestoreController();
            verifiedTravel.Restore();
            RestoreDirectTravel();
            bool firstRequest = string.IsNullOrEmpty(holdReason);
            if (firstRequest)
            {
                holdReason = string.IsNullOrEmpty(reason) ? "Held" : reason;
                Debug.LogWarning("CF1_HOLD boot=" + boot + " seq=" + sequence + " reason=" + holdReason +
                    " utc=" + DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture));
            }
            if (!holdPending && !holdConfirmed && transport != null && transport.IsConnected && handshake)
            {
                holdPending = true; holdAcknowledged = false; holdAttempts = 0; holdBoot = boot;
                holdStartedTime = Time.realtimeSinceStartup;
                SendHoldAttempt(holdStartedTime);
            }
            notice = holdReason;
            Status = handshake ? (holdPending ? "Connected / confirming HOLD" : "Connected / held") : "Robot disconnected";
        }
        private void ResetHoldRequest()
        {
            holdPending = holdAcknowledged = holdConfirmed = false;
            holdAttempts = 0; holdBoot = 0; holdReason = null;
            holdStartedTime = holdLastSentTime = 0;
        }
        private void SendHoldAttempt(float now)
        {
            holdAttempts++; holdLastSentTime = now;
            Send("HOLD " + holdBoot);
            // A failed write must not replace the initiating stop reason.
            notice = holdReason;
        }
        private void ConfirmHold()
        {
            holdPending = false; holdConfirmed = true; deviceArmed = false;
            Status = handshake ? "Connected / held" : "Robot disconnected";
            Debug.Log("CF1_HOLD_CONFIRMED boot=" + holdBoot + " seq=" + sequence +
                " attempts=" + holdAttempts + " reason=" + holdReason);
        }
        private void TickHoldRequest(float now)
        {
            if (!holdPending || transport == null || !transport.IsConnected) return;
            if (now - holdStartedTime >= HoldConfirmationTimeout)
            {
                LastConnectionError = "HOLD was not confirmed within 1 second; connection closed. Reconnect explicitly.";
                Debug.LogWarning("CF1_HOLD_TIMEOUT boot=" + holdBoot + " seq=" + sequence +
                    " attempts=" + holdAttempts + " reason=" + holdReason);
                transport.Disconnect();
                ResetMotionCapability();
                disconnectionReported = true; handshake = false; holdPending = false;
                deviceArmed = waitingArm = waitingLimits = physicalReady = false;
                commands = null; Status = "USB disconnected";
                notice = holdReason + " " + LastConnectionError;
                return;
            }
            if (holdAttempts < MaximumHoldAttempts && now - holdLastSentTime >= HoldRetryInterval)
                SendHoldAttempt(now);
        }
        public void ReleaseSignals()
        {
            if (!Connected || !supportedForRelease) { notice = "Support the arm before disabling signals: holding torque may disappear."; return; }
            Hold("Disabling servo signals...");
            releasePending = true; releaseAcknowledged = false; supportedForRelease = false;
            if (!Send("RELEASE " + boot)) { releasePending = false; notice = "Could not disable signals; do not assume they are off."; }
            else Send("STATUS");
        }
        private void OnApplicationFocus(bool focus) { if (!focus && (deviceArmed || waitingArm || waitingLimits)) Hold("Window lost focus"); }
        private void OnDisable() { Disconnect(); }
        private void OnDestroy() { if (arm) arm.SimulationStopped -= OnSimulationStopped; Disconnect(); }
        public void DrawControls()
        {
            if (wrap == null) wrap = new GUIStyle(GUI.skin.label) { wordWrap = true };
            if (toggleWrap == null) toggleWrap = new GUIStyle(GUI.skin.toggle) { wordWrap = true };
            GUILayout.Label("ESP32 USB robot link");
            GUILayout.Label(Status, wrap);
            if (!Connected && !string.IsNullOrEmpty(LastConnectionError))
                GUILayout.Label("USB detail: " + LastConnectionError, wrap);
            if (!Connected)
            {
                GUILayout.Label("Xbox connects to this PC. ESP32 uses its own USB cable.", wrap);
                GUILayout.BeginHorizontal(); port = GUILayout.TextField(port); if (GUILayout.Button("Ports")) RefreshPorts(); GUILayout.EndHorizontal();
                foreach (string available in ports) if (GUILayout.Button(available)) port = available;
                if (GUILayout.Button("Connect (no movement)")) Connect();
            }
            else
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("HOLD / STOP")) Hold("Held by user");
                if (GUILayout.Button("Disconnect (holds)")) Disconnect();
                GUILayout.EndHorizontal();
                bool signalsKnown = commands != null && Time.realtimeSinceStartup - lastStateTime < .5f && !waitingArm && !releasePending;
                GUILayout.Label(!signalsKnown ? "Servo signal state unconfirmed: wait for current telemetry." : outputsEnabled ? "Signals on / off unconfirmed: switching servo power may move the arm." : "Firmware reports servo signals OFF.", wrap);
                supportedForRelease = GUILayout.Toggle(supportedForRelease, "Arm supported for switching servo power", toggleWrap);
                if (GUILayout.Button("Disable servo signals")) ReleaseSignals();
                GUILayout.Label("Reported commands (not position sensors):", wrap);
                if (commands != null) for (int i = 0; i < 5; i++) GUILayout.Label("CH" + i + " " + names[i] + ": " + commands[i].ToString("0.0", CultureInfo.InvariantCulture) + " deg");
                if (!deviceArmed && !waitingArm && !waitingLimits)
                {
                    physicalReady = GUILayout.Toggle(physicalReady, "Arm supported; physical pose matches these commands", toggleWrap);
                    if (GUILayout.Button("Direct arm control")) BeginDirectControl();
                    GUILayout.Label("Uncalibrated travel", wrap);
                    GUILayout.Label("Calibration reference +/-90 degrees, bounded by servo 0..180. Starts from current commands; physical travel is uncalibrated.", wrap);
                    if (GUILayout.Button("Arm for 1-degree checks")) BeginCalibration();
                    if (GUILayout.Button("Arm and follow Unity — checked travel")) BeginFollow();
                }
                GUILayout.Space(8);
                selected = GUILayout.SelectionGrid(selected, names, 2);
                if (arm != null && arm.joints != null && arm.joints.Length == 5 && arm.joints.All(j => j != null) &&
                    DirectOperatorTravel.TryCreate(mapping, arm.joints.Select(j => j.minimumAngle).ToArray(),
                        arm.joints.Select(j => j.maximumAngle).ToArray(), mapping.servoNeutral,
                        out var displayedDirectRange, out _, out _))
                    GUILayout.Label("Direct " + names[selected] + ": " + CF1Protocol.Number(displayedDirectRange.MinimumCommand(selected)) +
                        " to " + CF1Protocol.Number(displayedDirectRange.MaximumCommand(selected)) +
                        " deg; calibration reference " + CF1Protocol.Number(mapping.servoNeutral[selected]), wrap);
                if (deviceArmed && mode == "Calibration")
                {
                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button("-1 servo degree")) Jog(-1f);
                    if (GUILayout.Button("+1 servo degree")) Jog(1f);
                    GUILayout.EndHorizontal();
                    GUILayout.Label("Calibration steps stay within 10 degrees of the calibration reference. Stop before binding.", wrap);
                }
                bool editable = !deviceArmed && !waitingArm && !waitingLimits;
                bool prior = GUI.enabled; GUI.enabled = prior && editable;
                GUILayout.Label("When model angle increases, servo command:", wrap);
                GUILayout.BeginHorizontal();
                if (GUILayout.Toggle(mapping.directions[selected] == 1, "Increases") && mapping.directions[selected] != 1) { mapping.directions[selected] = 1; mapping.directionVerified[selected] = false; }
                if (GUILayout.Toggle(mapping.directions[selected] == -1, "Decreases") && mapping.directions[selected] != -1) { mapping.directions[selected] = -1; mapping.directionVerified[selected] = false; }
                GUILayout.EndHorizontal();
                mapping.directionVerified[selected] = GUILayout.Toggle(mapping.directionVerified[selected], "I checked this joint's direction", toggleWrap);
                GUILayout.Label("Checked-travel limits (separate from Direct):", wrap);
                GUILayout.BeginHorizontal(); GUILayout.Label("Low", GUILayout.Width(30)); lowFields[selected] = GUILayout.TextField(lowFields[selected]); GUILayout.Label("High", GUILayout.Width(32)); highFields[selected] = GUILayout.TextField(highFields[selected]); GUILayout.EndHorizontal();
                float lo = 0, hi = 0;
                bool validFields = float.TryParse(lowFields[selected], NumberStyles.Float, CultureInfo.InvariantCulture, out lo) && float.TryParse(highFields[selected], NumberStyles.Float, CultureInfo.InvariantCulture, out hi);
                if (validFields)
                {
                    if (mapping.minimumCommand[selected] != lo || mapping.maximumCommand[selected] != hi) mapping.boundsVerified[selected] = false;
                    mapping.minimumCommand[selected] = lo; mapping.maximumCommand[selected] = hi;
                }
                else mapping.boundsVerified[selected] = false;
                mapping.boundsVerified[selected] = GUILayout.Toggle(mapping.boundsVerified[selected], "I checked this command range physically", toggleWrap) && validFields;
                GUILayout.Label("Start with a small tested range, then widen after checking clearance.", wrap);
                GUILayout.Label("Robot speed: " + physicalSpeed.ToString("0.0", CultureInfo.InvariantCulture) + " degrees/s");
                float selectedSpeed = GUILayout.HorizontalSlider(physicalSpeed, RobotSpeedSettings.Minimum, RobotSpeedSettings.Maximum);
                if (editable && selectedSpeed != physicalSpeed) SetRequestedSpeed(selectedSpeed);
                GUILayout.Label(speedDirty ? "Saving speed..." : string.IsNullOrEmpty(speedSettingsNotice)
                    ? "Speed is remembered when you reopen or reconnect." : speedSettingsNotice, wrap);
                if (EffectivePhysicalSpeed < physicalSpeed)
                    GUILayout.Label("Connected firmware limit: " + EffectivePhysicalSpeed.ToString("0.0", CultureInfo.InvariantCulture) + " degrees/s. Saved speed is unchanged.", wrap);
                if (GUILayout.Button("Save checked mapping"))
                {
                    if (mapping.HasVerifiedCalibration(out string why)) { File.WriteAllText(MappingPath, JsonUtility.ToJson(mapping, true)); notice = "Saved local direction and travel mapping."; }
                    else notice = why;
                }
                GUI.enabled = prior;
                mapping.HasVerifiedCalibration(out string remaining);
                if (!string.IsNullOrEmpty(remaining)) GUILayout.Label(remaining, wrap);
            }
            GUILayout.Space(6); GUILayout.Label(notice, wrap);
            GUILayout.Label("B, STOP or loss of focus holds the last command. USB loss triggers hold within 0.5 seconds. HOLD keeps servo signals on; use Disable signals before switching servo power. Re-arming still requires a matching physical pose.", wrap);
        }
    }
}
