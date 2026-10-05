using System;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Runtime controls only. Poses are local simulation files, never hardware commands.</summary>
    [DisallowMultipleComponent]
    public sealed partial class RobotControlPanel : MonoBehaviour
    {
        public RobotArm arm;
        public RobotOrbitCamera orbitCamera;
        public PoseSequence sequence;
        public RobotDebugVisualizer debugVisualizer;
        public GripperLinkage gripperLinkage;
        public InverseKinematics inverseKinematics;
        public MotionPlanner motionPlanner;
        public RobotCollisionMonitor collisionMonitor;
        public MotionRecordingCapture motionRecorder;
        public MotionRecordingPlayer motionPlayer;
        public bool showPanels = true;

        public int SelectedJointIndex { get; private set; }
        public RobotJoint SelectedJoint => arm != null && arm.joints != null
            && SelectedJointIndex >= 0 && SelectedJointIndex < arm.joints.Length
            ? arm.joints[SelectedJointIndex] : null;
        public event Action<RobotJoint> JointSelected;
        public bool TargetMarkerVisible => debugVisualizer == null || debugVisualizer.showTarget;
        // Other optional runtime panels can participate in the same camera input exclusion.
        public Func<Vector2, bool> AdditionalUiHitTest;

        private readonly string[] compactTabs = { "Joints", "Target", "Poses", "Display", "Robot link", "Demo" };
        private readonly string[] rightTabs = { "Target", "Poses", "Display", "Robot link", "Demo" };
        private readonly string[] pathModes = { "Joint space", "Cartesian" };
        private readonly string[] axisNames = { "X", "Y", "Z" };
        private readonly string[] targetFields = new string[3];
        private string[] angleFields = Array.Empty<string>();
        private bool[] angleDirty = Array.Empty<bool>();
        private bool targetDirty;
        private Vector3 lastTargetPosition;
        private bool targetInitialized;
        private int compactTab;
        private int rightTab, pathMode;
        private float cartesianSamples = 32f;
        private bool previewTargetRecorded;
        private Vector3 previewTargetWorld;
        private int previewPathMode = -1, previewSampleCount;
        private Renderer[] targetMarkerRenderers = Array.Empty<Renderer>();
        private Renderer[] toolMarkerRenderers = Array.Empty<Renderer>();
        private MaterialPropertyBlock targetTint;
        private IkResult observedIkResult;
        private Vector3 invalidResultTarget;
        private Vector2 leftScroll, rightScroll, poseScroll;
        private string poseName = "My pose";
        private string[] savedPoseNames = Array.Empty<string>();
        private int savedPoseIndex = -1;
        private RobotPose loadedPose;
        private float stepSpeed = 1f;
        private float stepPause = 0.5f;
        private string taskRecordingName = "";
        private RobotMotionRecordingInfo[] taskRecordings = Array.Empty<RobotMotionRecordingInfo>();
        private int selectedRecording = -1;
        private string observedSavedRecordingId;
        private Vector2 recordingsScroll;
        private string status = "Simulation controls. Use Robot link for the ESP32.";
        private GUIStyle titleStyle, headingStyle, smallStyle, wrapStyle, selectedStyle, calibrationStyle;

        private bool Wide => Screen.width >= 1280;
        private Rect LeftRect => new Rect(12, 12, Mathf.Min(330, Screen.width - 24), Mathf.Max(120, Screen.height - 24));
        private Rect RightRect => new Rect(Screen.width - 292, 12, 280, Mathf.Max(120, Screen.height - 24));
        public static Rect ClassicCollapsedControlsBounds => new Rect(12, 12, 360, 30);

        private void Start()
        {
            if (arm == null) arm = FindAnyObjectByType<RobotArm>();
            if (orbitCamera == null) orbitCamera = FindAnyObjectByType<RobotOrbitCamera>();
            if (sequence == null) sequence = FindAnyObjectByType<PoseSequence>();
            if (debugVisualizer == null) debugVisualizer = FindAnyObjectByType<RobotDebugVisualizer>();
            if (inverseKinematics == null) inverseKinematics = FindAnyObjectByType<InverseKinematics>();
            if (motionPlanner == null) motionPlanner = FindAnyObjectByType<MotionPlanner>();
            if (collisionMonitor == null) collisionMonitor = FindAnyObjectByType<RobotCollisionMonitor>();
            if (gripperLinkage == null) gripperLinkage = arm != null ? arm.GetComponentInChildren<GripperLinkage>() : null;
            if (gripperLinkage == null) gripperLinkage = FindAnyObjectByType<GripperLinkage>();
            if (sequence == null && arm != null) sequence = arm.gameObject.AddComponent<PoseSequence>();
            if (sequence != null && sequence.arm == null) sequence.arm = arm;
            if(arm!=null)
            {
                if(motionRecorder==null)motionRecorder=arm.GetComponent<MotionRecordingCapture>()??arm.gameObject.AddComponent<MotionRecordingCapture>();
                if(motionPlayer==null)motionPlayer=arm.GetComponent<MotionRecordingPlayer>()??arm.gameObject.AddComponent<MotionRecordingPlayer>();
                motionRecorder.arm=arm;motionPlayer.arm=arm;motionPlayer.Bind();
                RefreshTaskRecordings();
            }
            if (arm != null && arm.target != null) targetMarkerRenderers = arm.target.GetComponentsInChildren<Renderer>(true);
            if (arm != null && arm.endEffector != null) toolMarkerRenderers = arm.endEffector.GetComponentsInChildren<Renderer>(true);
            if (orbitCamera != null)
            {
                orbitCamera.controlPanel = this;
                if (orbitCamera.arm == null) orbitCamera.arm = arm;
                orbitCamera.TargetDragged += OnTargetDragged;
            }
            RefreshSavedPoses();
            SyncTargetFields();
        }

        private void OnDestroy()
        {
            if (orbitCamera != null) orbitCamera.TargetDragged -= OnTargetDragged;
            ReleasePortfolioResources();
        }

        public bool IsPointerOverPanel(Vector2 screenPoint)
        {
            Vector2 guiPoint = new Vector2(screenPoint.x, Screen.height - screenPoint.y);
            if (PortfolioDashboardActive)
                return PortfolioHitTest(guiPoint) || (AdditionalUiHitTest != null && AdditionalUiHitTest(screenPoint));
            bool ownPanel = showPanels ? LeftRect.Contains(guiPoint) || (Wide && RightRect.Contains(guiPoint))
                : ClassicCollapsedControlsBounds.Contains(guiPoint);
            return ownPanel || (AdditionalUiHitTest != null && AdditionalUiHitTest(screenPoint));
        }

        private void Update()
        {
            UpdatePortfolioPresentation();
            if(motionRecorder!=null&&motionRecorder.LastSavedId!=observedSavedRecordingId)
            {
                observedSavedRecordingId=motionRecorder.LastSavedId;
                RefreshTaskRecordings(observedSavedRecordingId);
            }
            if (debugVisualizer != null)
            {
                foreach (Renderer marker in targetMarkerRenderers) if (marker != null) marker.enabled = debugVisualizer.showTarget;
                foreach (Renderer marker in toolMarkerRenderers) if (marker != null) marker.enabled = debugVisualizer.showEndEffector;
            }
            Color targetColor = IkIndicator(out _);
            if (targetTint == null) targetTint = new MaterialPropertyBlock();
            foreach (Renderer marker in targetMarkerRenderers)
            {
                if (marker == null) continue;
                marker.GetPropertyBlock(targetTint);
                targetTint.SetColor("_Color", targetColor);
                targetTint.SetColor("_BaseColor", targetColor);
                marker.SetPropertyBlock(targetTint);
            }
            if (arm != null && arm.target != null && !targetDirty
                && (!targetInitialized || (lastTargetPosition - arm.target.position).sqrMagnitude > 0.000000001f))
                SyncTargetFields();
        }

        private void OnGUI()
        {
            PrepareStyles();
            if (PortfolioDashboardActive) { DrawPortfolioDashboard(); return; }
            if (!showPanels)
            {
                if (GUI.Button(new Rect(12, 12, 138, 30), "Show controls")) showPanels = true;
                if (GUI.Button(new Rect(158, 12, 90, 30), "Studio UI")) SetDashboardMode(true);
                if (GUI.Button(new Rect(256, 12, 116, 30), "HOLD / STOP")) PortfolioStopAll();
                return;
            }
            GUILayout.BeginArea(LeftRect, GUI.skin.box);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Career fair robot", titleStyle);
            if (GUILayout.Button("Hide", GUILayout.Width(46))) showPanels = false;
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Classic UI", smallStyle);
            if (GUILayout.Button("Studio UI", GUILayout.Width(90))) SetDashboardMode(true);
            GUILayout.EndHorizontal();
            GUILayout.Label("Five-joint simulation", smallStyle);
            GUILayout.Label("Drag empty space: 360° view | Wheel: zoom",smallStyle);
            var hardwareLink = arm != null ? arm.GetComponent<RobotHardwareLink>() : null;
            if (hardwareLink != null) GUILayout.Label(hardwareLink.Status, smallStyle);
            GUILayout.Label(arm != null && arm.neutralReference != null
                ? "Saved physical neutral. Travel limits unverified."
                : "CAD starting pose and limits - editable in Unity.", calibrationStyle);
            DrawGripperStatus();
            DrawCollisionSummary();
            if (arm != null) DrawTransportControls();
            if (!Wide)
            {
                int tab = DrawTabRows(compactTab, compactTabs);
                if (tab != compactTab) { compactTab = tab; leftScroll = Vector2.zero; GUI.FocusControl(null); }
            }
            leftScroll = GUILayout.BeginScrollView(leftScroll);
            if (arm == null) GUILayout.Label("Assign a RobotArm to this panel to begin.", wrapStyle);
            else if (Wide || compactTab == 0) DrawJointControls();
            else if (compactTab == 1) DrawTargetControls();
            else if (compactTab == 2) DrawPoseControls();
            else if (compactTab == 3) DrawDebugControls();
            else if(compactTab==4) DrawHardwareControls();
            else DrawPickAndPlaceControls();
            GUILayout.EndScrollView();
            GUILayout.Space(6);
            GUILayout.Label(status, wrapStyle, GUILayout.MaxHeight(78));
            GUILayout.EndArea();

            if (Wide)
            {
                GUILayout.BeginArea(RightRect, GUI.skin.box);
                int tab = DrawTabRows(rightTab, rightTabs);
                if (tab != rightTab) { rightTab = tab; rightScroll = Vector2.zero; GUI.FocusControl(null); }
                rightScroll = GUILayout.BeginScrollView(rightScroll);
                if (arm != null)
                {
                    if (rightTab == 0) DrawTargetControls();
                    else if (rightTab == 1) DrawPoseControls();
                    else if (rightTab == 2) DrawDebugControls();
                    else if(rightTab==3) DrawHardwareControls();
                    else DrawPickAndPlaceControls();
                }
                GUILayout.EndScrollView();
                GUILayout.EndArea();
            }
        }

        private void PrepareStyles()
        {
            if (titleStyle != null) return;
            titleStyle = new GUIStyle(GUI.skin.label) { fontSize = 19, fontStyle = FontStyle.Bold };
            headingStyle = new GUIStyle(GUI.skin.label) { fontSize = 15, fontStyle = FontStyle.Bold };
            smallStyle = new GUIStyle(GUI.skin.label) { fontSize = 11, wordWrap = true };
            wrapStyle = new GUIStyle(GUI.skin.label) { wordWrap = true, fontSize = 12 };
            selectedStyle = new GUIStyle(GUI.skin.button) { fontStyle = FontStyle.Bold };
            calibrationStyle = new GUIStyle(GUI.skin.label) { fontSize = 10, wordWrap = false };
        }

        private static int DrawTabRows(int selected,string[] labels)
        {
            int upperBefore=selected<3?selected:-1,lowerBefore=selected>=3?selected-3:-1;
            int upper=GUILayout.Toolbar(upperBefore,labels.Take(3).ToArray());
            int lower=GUILayout.Toolbar(lowerBefore,labels.Skip(3).ToArray());
            if(upper>=0&&upper!=upperBefore)return upper;
            if(lower>=0&&lower!=lowerBefore)return lower+3;
            return selected;
        }

        private void DrawHardwareControls()
        {
            var hardwareLink = arm != null ? arm.GetComponent<RobotHardwareLink>() : null;
            if (hardwareLink != null) hardwareLink.DrawControls();
            else GUILayout.Label("Robot link is available while the simulator is running.", wrapStyle);
        }

        private void DrawGripperStatus()
        {
            if (gripperLinkage == null) return;
            if (gripperLinkage.valid)
                GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                    "Gripper: {0:0.0} mm opening | linkage valid", gripperLinkage.jawOpeningMillimeters), smallStyle);
            else
                GUILayout.Label("Gripper: invalid pose - last valid geometry shown", smallStyle);
            if (!gripperLinkage.valid && !string.IsNullOrEmpty(gripperLinkage.status))
                GUILayout.Label(gripperLinkage.status, wrapStyle);
        }

        private void DrawDebugControls()
        {
            GUILayout.Label("Display", headingStyle);
            if(orbitCamera!=null)
            {
                GUILayout.Label("Camera view",smallStyle);
                GUILayout.BeginHorizontal();
                if(GUILayout.Button("Front"))orbitCamera.SetView(180,15);
                if(GUILayout.Button("Side"))orbitCamera.SetView(90,15);
                if(GUILayout.Button("Top"))orbitCamera.SetView(180,85);
                GUILayout.EndHorizontal();
                if(GUILayout.Button("Frame robot"))orbitCamera.FrameRobot();
                GUILayout.Label("Camera controls change only your view.",smallStyle);
            }
            if (debugVisualizer == null)
            {
                GUILayout.Label("Assign a RobotDebugVisualizer to show scene guides.", wrapStyle);
                return;
            }
            GUILayout.BeginHorizontal();
            debugVisualizer.showAxes = GUILayout.Toggle(debugVisualizer.showAxes, "Joint XYZ axes", GUILayout.Width(128));
            debugVisualizer.showPivots = GUILayout.Toggle(debugVisualizer.showPivots, "Pivot points");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            debugVisualizer.showOrigin = GUILayout.Toggle(debugVisualizer.showOrigin, "Origin frame", GUILayout.Width(128));
            debugVisualizer.showTarget = GUILayout.Toggle(debugVisualizer.showTarget, "Target marker");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            debugVisualizer.showTrajectory = GUILayout.Toggle(debugVisualizer.showTrajectory, "Trajectory", GUILayout.Width(128));
            debugVisualizer.showColliderBounds = GUILayout.Toggle(debugVisualizer.showColliderBounds, "Collider bounds");
            GUILayout.EndHorizontal();
            debugVisualizer.showJointLimits = GUILayout.Toggle(debugVisualizer.showJointLimits, "Joint limit arcs");
            debugVisualizer.showEndEffector = GUILayout.Toggle(debugVisualizer.showEndEffector, "Tool center marker");
            GUILayout.Label("XYZ frames: X red, Y green, Z blue.", smallStyle);
            if (collisionMonitor != null)
            {
                GUILayout.Space(10);
                collisionMonitor.monitoringEnabled = GUILayout.Toggle(collisionMonitor.monitoringEnabled, "Collision advisory");
                GUILayout.Label("Bounding-box overlap is a warning, not an exact mesh or hardware safety check.", smallStyle);
                GUILayout.Label(collisionMonitor.Status, wrapStyle);
                string[] warnings = collisionMonitor.CurrentWarnings;
                if (warnings != null)
                {
                    int visibleCount = Mathf.Min(5, warnings.Length);
                    for (int i = 0; i < visibleCount; i++) GUILayout.Label("- " + warnings[i], smallStyle);
                    if (warnings.Length > visibleCount) GUILayout.Label("+ " + (warnings.Length - visibleCount) + " more potential overlaps", smallStyle);
                }
            }
        }

        private void DrawCollisionSummary()
        {
            if (collisionMonitor == null) return;
            string text = !collisionMonitor.monitoringEnabled ? "Collision advisory: off"
                : collisionMonitor.HasInvalidProxies ? "Collision advisory incomplete - see Display"
                : collisionMonitor.HasPotentialCollision ? "Collision advisory: potential overlap - see Display"
                : collisionMonitor.Status;
            Color previous = GUI.contentColor;
            if (collisionMonitor.monitoringEnabled && collisionMonitor.HasPotentialCollision)
                GUI.contentColor = new Color(1f, 0.73f, 0.4f);
            GUILayout.Label(text, smallStyle);
            GUI.contentColor = previous;
        }

        private void DrawTransportControls()
        {
            if (arm.operationalHome != null) GUILayout.Label("Home: " + arm.operationalHome.name, smallStyle);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(new GUIContent("Home", "Move to the saved operational Home, or the configured home angles.")))
                MoveHomeFromUi(false);
            if (GUILayout.Button(new GUIContent("Reset", "Reset the arm immediately to its configured home pose.")))
                MoveHomeFromUi(true);
            Color oldColor = GUI.backgroundColor;
            GUI.backgroundColor = new Color(1f, 0.58f, 0.48f);
            if (GUILayout.Button(new GUIContent("STOP", "Stop follow, path and sequence playback at the current joint angles.")))
            { StopAutomaticMotion(); arm.Stop(); ClearAngleEdits(); status = "Stopped. Target follow and all playback are off."; }
            GUI.backgroundColor = oldColor;
            GUILayout.EndHorizontal();
        }

        private void MoveHomeFromUi(bool immediate)
        {
            try
            {
                arm.ValidateHomePose();
                StopAutomaticMotion();
                if (immediate) arm.ResetHome(); else arm.Home();
                ClearAngleEdits();
                status = immediate ? "Home pose reset." : "Moving to home.";
            }
            catch (Exception e) { status = e.Message; }
        }

        private void DrawJointControls()
        {
            arm.instantMode = GUILayout.Toggle(arm.instantMode, "Instant changes (skip smooth movement)");
            GUILayout.Label("Model angles in degrees, not servo commands. Type a target then press Set.", smallStyle);
            if (arm.joints == null || arm.joints.Length == 0) { GUILayout.Label("No joints configured."); return; }
            EnsureAngleFields();
            for (int i = 0; i < arm.joints.Length; i++)
            {
                RobotJoint joint = arm.joints[i];
                if (joint == null) continue;
                GUILayout.Space(7);
                GUILayout.BeginHorizontal();
                if (GUILayout.Button((SelectedJointIndex == i ? "> " : "") + joint.jointName,
                    SelectedJointIndex == i ? selectedStyle : GUI.skin.button, GUILayout.MinWidth(0)))
                { SelectedJointIndex = i; JointSelected?.Invoke(joint); }
                if (GUILayout.Button("Focus", GUILayout.Width(50)))
                {
                    SelectedJointIndex = i;
                    JointSelected?.Invoke(joint);
                    if (orbitCamera != null) orbitCamera.Focus(joint.transform);
                }
                GUILayout.EndHorizontal();
                GUILayout.Label(string.Format(CultureInfo.InvariantCulture, "Now {0:0.0}  Target {1:0.0} deg  [{2:0.#}, {3:0.#}]",
                    joint.currentAngle, joint.targetAngle, joint.minimumAngle, joint.maximumAngle), smallStyle);
                var reference = arm.neutralReference != null ? arm.neutralReference.Find(joint.jointName) : null;
                if (reference != null)
                    GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                        "Startup reference: CH{0} servo command {1:0.0} deg{2}", reference.channel,
                        reference.servoCommandDegrees, reference.channel == 4 ? " (OPEN)" : ""), smallStyle);
                float next = GUILayout.HorizontalSlider(joint.targetAngle, joint.minimumAngle, joint.maximumAngle);
                if (!Mathf.Approximately(next, joint.targetAngle)) SetJoint(i, next);
                GUILayout.BeginHorizontal();
                string fieldName = "robot-angle-" + i;
                if (!angleDirty[i] && GUI.GetNameOfFocusedControl() != fieldName)
                    angleFields[i] = Format(joint.targetAngle);
                GUI.SetNextControlName(fieldName);
                string text = GUILayout.TextField(angleFields[i], GUILayout.MinWidth(65));
                if (text != angleFields[i]) { angleFields[i] = text; angleDirty[i] = true; }
                GUILayout.Label("deg", GUILayout.Width(26));
                bool enter = IsEnter(fieldName);
                if (GUILayout.Button("Set", GUILayout.Width(50)) || enter)
                {
                    if (TryNumber(angleFields[i], out float number))
                    {
                        float clamped = Mathf.Clamp(number, joint.minimumAngle, joint.maximumAngle);
                        SetJoint(i, clamped);
                        status = Mathf.Approximately(number, clamped) ? joint.jointName + " target updated."
                            : joint.jointName + " clamped to its configured limit.";
                    }
                    else status = "Enter a finite angle, for example 25 or -12.5.";
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.Space(10);
            if (GUILayout.Button("Frame whole robot") && orbitCamera != null) orbitCamera.FrameRobot();
            GUILayout.Label("Outside these panels: right-drag to orbit, middle-drag to pan, wheel to zoom. Left-drag the target to move it in the camera plane.", wrapStyle);
        }

        private void DrawTargetControls()
        {
            GUILayout.Label("Tool position", headingStyle);
            GUILayout.Label("Relative to robot origin - millimetres", smallStyle);
            Vector3 ee = arm.EndEffectorMillimeters;
            GUILayout.Label(string.Format(CultureInfo.InvariantCulture, "X {0:0.0}   Y {1:0.0}   Z {2:0.0}", ee.x, ee.y, ee.z), wrapStyle);
            GUILayout.Space(8);
            GUILayout.Label("Target position", headingStyle);
            if (arm.target == null || arm.robotOrigin == null)
            { GUILayout.Label("Assign the target and robot origin to edit XYZ.", wrapStyle); return; }
            if (!targetInitialized) SyncTargetFields();
            for (int i = 0; i < 3; i++)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(axisNames[i], GUILayout.Width(18));
                GUI.SetNextControlName("robot-target-" + i);
                string text = GUILayout.TextField(targetFields[i] ?? "0");
                if (text != targetFields[i]) { targetFields[i] = text; targetDirty = true; }
                GUILayout.Label("mm", GUILayout.Width(28));
                GUILayout.EndHorizontal();
            }
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Apply XYZ")) ApplyTargetFields();
            if (GUILayout.Button("Read target")) { SyncTargetFields(); status = "Target fields refreshed."; }
            GUILayout.EndHorizontal();
            if (GUILayout.Button("Place target at tool") && arm.endEffector != null)
            { arm.target.position = arm.endEffector.position; OnTargetChanged(); status = "Target placed at the tool once."; }
            if (GUILayout.Button("Use target ball")) UseTargetBall();
            GUILayout.Label(inverseKinematics != null && inverseKinematics.FollowTarget
                ? "Follow is ON: the arm moves toward target edits and drags."
                : "Target stays fixed. Follow is off until you enable it.", smallStyle);
            DrawInverseKinematicsControls();
            DrawPathControls();
        }

        private void DrawInverseKinematicsControls()
        {
            GUILayout.Space(10);
            GUILayout.Label("Reach target (IK)", headingStyle);
            if (inverseKinematics == null)
            { GUILayout.Label("IK component is not configured.", smallStyle); return; }
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(new GUIContent("Compute IK", "Stop current motion and compute angles without moving the robot.")))
                ComputeTargetSolution();
            bool oldEnabled = GUI.enabled;
            IkIndicator(out bool staleSolution);
            GUI.enabled = oldEnabled && !staleSolution && inverseKinematics.LastResult != null && inverseKinematics.LastResult.reachable;
            if (GUILayout.Button("Move to solution"))
                MoveToComputedSolution();
            GUI.enabled = oldEnabled;
            GUILayout.EndHorizontal();
            bool follow = GUILayout.Toggle(inverseKinematics.FollowTarget, "Follow target continuously");
            if (follow != inverseKinematics.FollowTarget)
            {
                if (follow)
                {
                    if (!targetDirty || ApplyTargetFields())
                    {
                        inverseKinematics.StopForTargetControl(StopAutomaticMotion);
                        inverseKinematics.SetFollowTarget(true);
                        status = "Following target. Manual joint commands or STOP turn follow off.";
                    }
                }
                else { inverseKinematics.SetFollowTarget(false); inverseKinematics.StopForTargetControl(); status = "Target follow off."; }
            }
            var result = inverseKinematics.LastResult;
            if (result == null) { GUILayout.Label("Compute IK to preview joint angles.", smallStyle); return; }
            Color previousContentColor = GUI.contentColor;
            GUI.contentColor = IkIndicator(out bool stale);
            string resultLabel = result.status == IkStatus.NoConvergence ? "No solution found"
                : result.status == IkStatus.OutsideGeometricBound ? "Outside geometric bound"
                : result.status == IkStatus.InvalidInput ? "Invalid input" : "Reachable";
            GUILayout.Label(stale ? "Target changed - compute IK again"
                : resultLabel + " | error " + Metric(result.residualMillimeters) + " mm", wrapStyle);
            GUI.contentColor = previousContentColor;
            GUILayout.Label(stale ? "Previous solution angles are shown below." : result.message, smallStyle);
            if (result.angles != null && arm.joints != null)
            {
                int count = Mathf.Min(result.angles.Length, arm.joints.Length);
                for (int i = 0; i < count; i++)
                {
                    string name = arm.joints[i] != null ? arm.joints[i].jointName : "Joint " + (i + 1);
                    GUILayout.Label(name + ": " + Format(result.angles[i]) + " deg", smallStyle);
                }
            }
        }

        private void DrawPathControls()
        {
            GUILayout.Space(10);
            GUILayout.Label("Path preview", headingStyle);
            if (motionPlanner == null)
            { GUILayout.Label("Motion planner is not configured.", smallStyle); return; }
            pathMode = GUILayout.Toolbar(pathMode, pathModes);
            if (pathMode == 1)
            {
                GUILayout.Label("Cartesian samples: " + Mathf.RoundToInt(cartesianSamples), smallStyle);
                cartesianSamples = Mathf.Round(GUILayout.HorizontalSlider(cartesianSamples, 8, 64));
            }
            if (GUILayout.Button("Preview path"))
            {
                if (!targetDirty || ApplyTargetFields())
                {
                    if (inverseKinematics != null) inverseKinematics.StopForTargetControl(StopAutomaticMotion);
                    else { StopAutomaticMotion(); arm.Stop(); }
                    try
                    {
                        if (pathMode == 0)
                        {
                            var result = inverseKinematics != null ? inverseKinematics.SolveTarget() : null;
                            if (result != null && result.reachable) motionPlanner.PreviewJointSpace(result.angles);
                            else
                            {
                                inverseKinematics.StopForTargetControl(motionPlanner.ClearPreview);
                                status = "No reachable IK solution for a joint-space path.";
                            }
                        }
                        else motionPlanner.PreviewCartesian(arm.target.position, Mathf.RoundToInt(cartesianSamples));
                        previewTargetWorld = arm.target.position;
                        previewTargetRecorded = true;
                        previewPathMode = pathMode;
                        previewSampleCount = Mathf.RoundToInt(cartesianSamples);
                        RefreshPathVisual();
                        if (motionPlanner.LastPlan != null) status = motionPlanner.LastPlan.message;
                    }
                    catch (Exception e) { status = "Could not preview path: " + e.Message; }
                }
            }
            var plan = motionPlanner.LastPlan;
            bool targetChanged = previewTargetRecorded && arm.target != null
                && (previewTargetWorld - arm.target.position).sqrMagnitude > 0.00000001f;
            bool settingsChanged = previewPathMode >= 0 && (previewPathMode != pathMode
                || (pathMode == 1 && previewSampleCount != Mathf.RoundToInt(cartesianSamples)));
            if (plan != null)
            {
                GUILayout.Label(plan.label, wrapStyle);
                GUILayout.Label(plan.message, smallStyle);
                if (plan.valid)
                    GUILayout.Label("Duration " + Metric(plan.durationSeconds) + " s"
                        + (previewPathMode == 1 ? " | max error " + Metric(plan.maxCartesianResidualMillimeters) + " mm" : ""), smallStyle);
            }
            if (targetChanged || targetDirty) GUILayout.Label("Target changed. Preview again before playing.", smallStyle);
            else if (settingsChanged) GUILayout.Label("Path settings changed. Preview again before playing.", smallStyle);
            GUILayout.BeginHorizontal();
            bool oldEnabled = GUI.enabled;
            GUI.enabled = oldEnabled && plan != null && plan.valid && !targetChanged && !settingsChanged && !targetDirty && !motionPlanner.IsPlaying;
            if (GUILayout.Button("Play path"))
            {
                if (inverseKinematics != null) inverseKinematics.StopForTargetControl(StopSequence);
                else StopSequence();
                if (inverseKinematics != null) inverseKinematics.SetFollowTarget(false);
                try { status = motionPlanner.Play() ? "Playing planned path." : motionPlanner.Status; }
                catch (Exception e) { status = "Could not play path: " + e.Message; }
            }
            GUI.enabled = oldEnabled;
            if (GUILayout.Button("Cancel")) { motionPlanner.Stop(); status = "Path playback cancelled."; }
            GUILayout.EndHorizontal();
            if (GUILayout.Button("Clear preview"))
            { motionPlanner.ClearPreview(); previewTargetRecorded = false; previewPathMode = -1; RefreshPathVisual(); }
            GUILayout.Label(motionPlanner.Status, smallStyle);
            GUILayout.Label("Previewed motion is not a collision clearance or hardware safety guarantee.", smallStyle);
        }

        private void RefreshPathVisual()
        {
            if (debugVisualizer == null || motionPlanner == null) return;
            var plan = motionPlanner.LastPlan;
            debugVisualizer.trajectory = plan != null && plan.worldPoints != null ? plan.worldPoints : Array.Empty<Vector3>();
            debugVisualizer.trajectoryColor = plan != null && plan.valid ? new Color(0.3f, 0.85f, 1f) : new Color(1f, 0.65f, 0.2f);
        }

        private void DrawPoseControls()
        {
            GUILayout.Label("Saved poses", headingStyle);
            poseName = GUILayout.TextField(poseName, 64);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Save current"))
            {
                try
                {
                    if (string.IsNullOrWhiteSpace(poseName)) throw new ArgumentException("Give the pose a name first.");
                    loadedPose = RobotPose.Capture(arm, poseName.Trim());
                    loadedPose.Save();
                    RefreshSavedPoses();
                    savedPoseIndex = Array.IndexOf(savedPoseNames, loadedPose.name);
                    status = "Pose saved locally: " + loadedPose.name;
                }
                catch (Exception e) { status = "Could not save pose: " + e.Message; }
            }
            if (GUILayout.Button("Refresh", GUILayout.Width(65))) RefreshSavedPoses();
            GUILayout.EndHorizontal();
            if (savedPoseNames.Length == 0) GUILayout.Label("No saved poses yet.", smallStyle);
            else
            {
                poseScroll = GUILayout.BeginScrollView(poseScroll, GUILayout.MaxHeight(100));
                int next = GUILayout.SelectionGrid(savedPoseIndex, savedPoseNames, 1);
                if (next != savedPoseIndex) { savedPoseIndex = next; loadedPose = null; }
                GUILayout.EndScrollView();
            }
            GUILayout.BeginHorizontal();
            GUI.enabled = savedPoseIndex >= 0 && savedPoseIndex < savedPoseNames.Length;
            if (GUILayout.Button("Load")) LoadSelectedPose();
            if (GUILayout.Button("Play pose"))
            {
                if (LoadSelectedPose())
                {
                    try
                    {
                        StopAutomaticMotion();
                        loadedPose.Apply(arm, arm.instantMode);
                        ClearAngleEdits();
                        status = "Playing pose: " + loadedPose.name;
                    }
                    catch (Exception e) { status = "Could not play pose: " + e.Message; }
                }
            }
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (loadedPose != null) GUILayout.Label("Loaded: " + loadedPose.name, smallStyle);

            GUILayout.Space(10);
            GUILayout.Label("Pose sequence", headingStyle);
            if (sequence == null) { GUILayout.Label("No PoseSequence component is available.", smallStyle); return; }
            if (sequence.steps == null) sequence.steps = new System.Collections.Generic.List<PoseSequence.Step>();
            if (!string.IsNullOrEmpty(sequence.LastError)) GUILayout.Label(sequence.LastError, wrapStyle);
            GUILayout.Label("New step speed x" + stepSpeed.ToString("0.00", CultureInfo.InvariantCulture)
                + " - pause " + stepPause.ToString("0.0", CultureInfo.InvariantCulture) + " s", smallStyle);
            stepSpeed = GUILayout.HorizontalSlider(stepSpeed, 0.1f, 2f);
            stepPause = GUILayout.HorizontalSlider(stepPause, 0f, 5f);
            if (GUILayout.Button("Add saved pose to sequence") && LoadSelectedPose())
            {
                StopSequence();
                sequence.steps.Add(new PoseSequence.Step { pose = loadedPose, speedMultiplier = stepSpeed, pauseSeconds = stepPause });
                status = "Added " + loadedPose.name + " to the sequence.";
            }
            if (GUILayout.Button("Add current position"))
            {
                StopSequence();
                string name = string.IsNullOrWhiteSpace(poseName) ? "Step " + (sequence.steps.Count + 1) : poseName.Trim();
                sequence.steps.Add(new PoseSequence.Step
                { pose = RobotPose.Capture(arm, name), speedMultiplier = stepSpeed, pauseSeconds = stepPause });
            }
            if (sequence.steps != null)
            {
                for (int i = 0; i < sequence.steps.Count; i++)
                {
                    PoseSequence.Step step = sequence.steps[i];
                    string name = step != null && step.pose != null ? step.pose.name : "(missing pose)";
                    GUILayout.Label((sequence.IsPlaying && sequence.CurrentStepIndex == i ? "> " : "")
                        + (i + 1) + ". " + name, smallStyle);
                }
            }
            GUILayout.BeginHorizontal();
            GUI.enabled = sequence.steps != null && sequence.steps.Count > 0;
            if (GUILayout.Button("Play sequence"))
            {
                try { StopAutomaticMotion(); sequence.Play(); status = "Sequence playing."; }
                catch (Exception e) { status = "Could not play sequence: " + e.Message; }
            }
            if (GUILayout.Button("Stop")) { sequence.Stop(); arm.Stop(); status = "Sequence stopped."; }
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (GUILayout.Button("Clear sequence")) { sequence.Stop(); sequence.Clear(); status = "Sequence cleared."; }
        }

        private void DrawPickAndPlaceControls()
        {
            GUILayout.Label("Task recordings",headingStyle);
            if(motionRecorder==null||motionPlayer==null)
            {GUILayout.Label("Start the simulator to record a task.",wrapStyle);return;}
            GUILayout.Label("Record the whole task in one take. Move the arm and gripper, then Stop & save. Each take is a separate recording.",wrapStyle);
            bool previous=GUI.enabled;
            bool recording=motionRecorder.IsRecording;
            bool playing=motionPlayer.IsPlaying;
            GUI.enabled=previous&&!recording&&!playing&&!motionRecorder.HasUnsavedRecording;
            GUILayout.Label("Task name (optional)",smallStyle);
            taskRecordingName=GUILayout.TextField(taskRecordingName,64);
            if(GUILayout.Button("Start recording"))
            {
                motionRecorder.StartRecording(taskRecordingName,out _);
                status=motionRecorder.Status;
            }
            GUI.enabled=previous&&recording;
            if(GUILayout.Button("Stop & save"))
            {
                if(motionRecorder.StopAndSave(out _))taskRecordingName="";
                status=motionRecorder.Status;
            }
            GUI.enabled=previous;
            if(recording)GUILayout.Label("RECORDING  "+RecordingTime(motionRecorder.ElapsedSeconds),headingStyle);
            GUILayout.Label(motionRecorder.Status,wrapStyle);
            if(motionRecorder.HasUnsavedRecording)
            {
                if(GUILayout.Button("Retry saving take")){motionRecorder.StopAndSave(out _);status=motionRecorder.Status;}
                if(GUILayout.Button("Discard unsaved take")){motionRecorder.DiscardUnsavedRecording(out _);status=motionRecorder.Status;}
            }
            GUILayout.Space(8);
            GUILayout.Label("Saved tasks",headingStyle);
            GUI.enabled=previous&&!recording&&!playing&&!motionRecorder.HasUnsavedRecording;
            if(taskRecordings.Length>0)
            {
                selectedRecording=Mathf.Clamp(selectedRecording,0,taskRecordings.Length-1);
                var names=taskRecordings.Select((clip,i)=>(i+1)+". "+clip.name+"  ("+RecordingTime(clip.durationSeconds)+")").ToArray();
                recordingsScroll=GUILayout.BeginScrollView(recordingsScroll,GUILayout.MaxHeight(190));
                selectedRecording=GUILayout.SelectionGrid(selectedRecording,names,1);
                GUILayout.EndScrollView();
            }
            else GUILayout.Label("No task recordings yet. Save your first take above.",smallStyle);
            if(GUILayout.Button("Refresh recordings"))RefreshTaskRecordings();
            var link=arm.GetComponent<RobotHardwareLink>();
            bool following=link!=null&&link.FollowActive;
            bool canPlay=previous&&!recording&&!playing&&!motionRecorder.HasUnsavedRecording&&selectedRecording>=0&&selectedRecording<taskRecordings.Length;
            GUILayout.Space(6);
            GUI.enabled=canPlay&&!following;
            if(GUILayout.Button("Play selected — simulation only"))PlaySelectedRecording(false);
            GUI.enabled=canPlay&&following;
            if(GUILayout.Button("Play selected — REAL ARM"))PlaySelectedRecording(true);
            GUI.enabled=previous;
            if(GUILayout.Button("STOP playback")){motionPlayer.Stop();status=motionPlayer.Status;}
            if(playing)GUILayout.Label(motionPlayer.MovingToStart?"Moving to the recording's start":RecordingTime(motionPlayer.ElapsedSeconds)+" / "+RecordingTime(motionPlayer.DurationSeconds),smallStyle);
            GUILayout.Label(motionPlayer.Status,wrapStyle);
            if(!following)GUILayout.Label("To replay on the real arm, enable Direct arm control in Robot link first.",smallStyle);
            GUILayout.Label("Playback moves to the start, then plays the selected take once. STOP, manual control or leaving this window ends playback. Leaving while recording saves the partial take.",smallStyle);
        }

        private void RefreshTaskRecordings(string selectId=null)
        {
            if(motionRecorder==null)return;
            if(selectId==null&&selectedRecording>=0&&selectedRecording<taskRecordings.Length)selectId=taskRecordings[selectedRecording].id;
            taskRecordings=RobotMotionRecordingStore.List(motionRecorder.DirectoryPath,out string reason).ToArray();
            selectedRecording=Array.FindIndex(taskRecordings,c=>c.id==selectId);
            if(selectedRecording<0&&taskRecordings.Length>0)selectedRecording=0;
            if(!string.IsNullOrEmpty(reason))status=reason;
        }

        private void PlaySelectedRecording(bool hardware)
        {
            if(selectedRecording<0||selectedRecording>=taskRecordings.Length)return;
            if(!RobotMotionRecordingStore.Load(taskRecordings[selectedRecording].path,out var clip,out string reason))
            {status=reason;return;}
            motionPlayer.PlayOnce(clip,hardware,out _);status=motionPlayer.Status;
        }

        private static string RecordingTime(float seconds)
        {
            int whole=Mathf.Max(0,Mathf.FloorToInt(seconds));
            return (whole/60).ToString(CultureInfo.InvariantCulture)+":"+(whole%60).ToString("00",CultureInfo.InvariantCulture);
        }

        private void EnsureAngleFields()
        {
            if (angleFields.Length == arm.joints.Length) return;
            angleFields = new string[arm.joints.Length];
            angleDirty = new bool[arm.joints.Length];
            for (int i = 0; i < angleFields.Length; i++) angleFields[i] = arm.joints[i] != null ? Format(arm.joints[i].targetAngle) : "0";
        }

        private void ClearAngleEdits()
        {
            for (int i = 0; i < angleDirty.Length; i++) angleDirty[i] = false;
            if (Event.current != null) GUI.FocusControl(null);
        }

        private void SetJoint(int index, float value)
        {
            StopAutomaticMotion();
            arm.SetJointTarget(arm.joints[index].jointName, value);
            angleFields[index] = Format(value);
            angleDirty[index] = false;
        }

        private void StopSequence() { if (sequence != null && sequence.IsPlaying) sequence.Stop(); }

        public bool ComputeTargetSolution()
        {
            if (arm == null || inverseKinematics == null) { status = "Assign the arm and IK first."; return false; }
            if (targetDirty && !ApplyTargetFields()) return false;
            try
            {
                // Preview pauses interpolation without ending the existing hardware
                // follow session. Explicit STOP still goes through arm.Stop normally.
                inverseKinematics.StopForTargetControl(StopAutomaticMotion);
                var result = inverseKinematics.SolveTarget();
                status = result.reachable ? "IK computed without moving the arm." : "IK target is not reachable: " + result.message;
                return result.reachable;
            }
            catch (Exception e) { status = "Could not solve target: " + e.Message; return false; }
        }

        public bool MoveToComputedSolution()
        {
            if (arm == null || inverseKinematics == null) { status = "Assign the arm and IK first."; return false; }
            IkIndicator(out bool stale);
            if (stale || inverseKinematics.LastResult == null || !inverseKinematics.LastResult.reachable)
            { status = "Compute IK for the current target first."; return false; }
            try
            {
                // Sequence/planner cancellation is an input-mode change, not HOLD.
                inverseKinematics.StopForTargetControl(StopAutomaticMotion);
                bool moved = inverseKinematics.MoveToSolution();
                status = moved ? "Moving to the IK solution." : "Solution could not be applied. Compute IK again.";
                ClearAngleEdits();
                return moved;
            }
            catch (Exception e) { status = "Could not move to solution: " + e.Message; return false; }
        }

        public bool UseTargetBall()
        {
            if (arm == null || arm.target == null || arm.endEffector == null || inverseKinematics == null)
            { status = "Assign the arm, tool, target and IK before using the target ball."; return false; }
            inverseKinematics.StopForTargetControl(StopAutomaticMotion);
            arm.target.position = arm.endEffector.position;
            if (debugVisualizer != null) debugVisualizer.showTarget = true;
            SyncTargetFields();
            inverseKinematics.SetFollowTarget(true);
            status = "Target ball active: left-drag it to move the tool. Xbox input takes over when used; STOP stops both.";
            return true;
        }

        private void StopAutomaticMotion()
        {
            if(motionPlayer!=null)motionPlayer.CancelForManualControl();
            if (inverseKinematics != null) inverseKinematics.Cancel();
            if (motionPlanner != null && motionPlanner.IsPlaying) motionPlanner.Stop();
            StopSequence();
        }

        private void SyncTargetFields()
        {
            if (arm == null || arm.target == null || arm.robotOrigin == null) return;
            Vector3 mm = arm.TargetMillimeters;
            for (int i = 0; i < 3; i++) targetFields[i] = Format(mm[i]);
            lastTargetPosition = arm.target.position;
            targetInitialized = true;
            targetDirty = false;
        }

        private void OnTargetDragged() { OnTargetChanged(); }

        private void OnTargetChanged()
        {
            if(motionPlayer!=null)motionPlayer.CancelForManualControl();
            if (motionPlanner != null && motionPlanner.IsPlaying)
            {
                if (inverseKinematics != null) inverseKinematics.StopForTargetControl(motionPlanner.Stop);
                else motionPlanner.Stop();
                status = "Target changed; path playback stopped. Preview again to use the new target.";
            }
            SyncTargetFields();
        }

        private bool ApplyTargetFields()
        {
            Vector3 mm = Vector3.zero;
            for (int i = 0; i < 3; i++)
            {
                if (!TryNumber(targetFields[i], out float value))
                { status = "Enter finite XYZ values in millimetres."; return false; }
                mm[i] = value;
            }
            if (!JointMotion.IsFinite(arm.millimetersPerUnityUnit) || arm.millimetersPerUnityUnit <= 0f)
            { status = "Millimetres per Unity unit must be positive."; return false; }
            arm.target.position = arm.robotOrigin.TransformPoint(mm / arm.millimetersPerUnityUnit);
            OnTargetChanged();
            status = inverseKinematics != null && inverseKinematics.FollowTarget
                ? "Target updated. Follow is active." : "Target updated. Follow is off.";
            return true;
        }

        private void RefreshSavedPoses()
        {
            try
            {
                savedPoseNames = RobotPose.SavedPoseNames() ?? Array.Empty<string>();
                savedPoseIndex = savedPoseNames.Length > 0 ? Mathf.Clamp(savedPoseIndex, 0, savedPoseNames.Length - 1) : -1;
            }
            catch (Exception e) { savedPoseNames = Array.Empty<string>(); savedPoseIndex = -1; status = "Could not list local poses: " + e.Message; }
        }

        private bool LoadSelectedPose()
        {
            if (savedPoseIndex < 0 || savedPoseIndex >= savedPoseNames.Length)
            { status = "Select a saved pose first."; return false; }
            try
            {
                loadedPose = RobotPose.Load(savedPoseNames[savedPoseIndex]);
                if (loadedPose == null) throw new InvalidOperationException("The pose file was empty.");
                poseName = loadedPose.name;
                status = "Loaded " + loadedPose.name + ". Press Play pose to move.";
                return true;
            }
            catch (Exception e) { loadedPose = null; status = "Could not load pose: " + e.Message; return false; }
        }

        private static string Format(float value) => value.ToString("0.0", CultureInfo.InvariantCulture);
        private static string Metric(float value) => JointMotion.IsFinite(value) ? value.ToString("0.00", CultureInfo.InvariantCulture) : "n/a";

        private Color IkIndicator(out bool stale)
        {
            Color amber = new Color(1f, 0.65f, 0.2f);
            stale = targetDirty;
            var result = inverseKinematics != null ? inverseKinematics.LastResult : null;
            if (result == null) return amber;
            if (!ReferenceEquals(result, observedIkResult))
            {
                observedIkResult = result;
                // Invalid-input results have no geometric target datum. Remember the
                // target at observation so that later movement still clears red feedback.
                invalidResultTarget = arm != null && arm.target != null ? arm.target.position : Vector3.zero;
            }
            if (arm != null && arm.target != null)
            {
                Vector3 solvedTarget = result.status == IkStatus.InvalidInput ? invalidResultTarget : result.targetWorld;
                Vector3 delta = arm.target.position - solvedTarget;
                if (arm.robotOrigin != null) delta = arm.robotOrigin.InverseTransformVector(delta);
                float distanceMm = delta.magnitude * arm.millimetersPerUnityUnit;
                stale |= !JointMotion.IsFinite(distanceMm) || distanceMm > inverseKinematics.toleranceMillimeters;
            }
            if (stale) return amber;
            if (result.status == IkStatus.Reachable) return new Color(0.25f, 0.95f, 0.4f);
            if (result.status == IkStatus.OutsideGeometricBound || result.status == IkStatus.InvalidInput)
                return new Color(1f, 0.3f, 0.25f);
            return amber;
        }
        private static bool TryNumber(string value, out float number)
        {
            bool parsed = float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out number)
                || float.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out number);
            return parsed && !float.IsNaN(number) && !float.IsInfinity(number);
        }

        private static bool IsEnter(string controlName)
        {
            Event e = Event.current;
            if (GUI.GetNameOfFocusedControl() != controlName || e.type != EventType.KeyDown
                || (e.keyCode != KeyCode.Return && e.keyCode != KeyCode.KeypadEnter)) return false;
            e.Use();
            return true;
        }
    }
}
