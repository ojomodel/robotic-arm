using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using CareerFair.Robot.Hardware;

namespace CareerFair.Robot
{
    // Presentation only: movement, recording, replay and hardware gates remain
    // in their existing owners. This panel never connects or arms on its own.
    public sealed partial class RobotControlPanel
    {
        public bool portfolioDashboard = true;
        public bool PortfolioDashboardActive => portfolioDashboard;
        public bool PresentationView { get; private set; }
        public int PortfolioTab { get; private set; }
        private readonly string[] portfolioTabs = { "Joints", "Tasks", "Target", "Display", "Robot link", "Poses" };
        private readonly List<Texture2D> portfolioTextures = new List<Texture2D>();
        private GUISkin portfolioSkin;
        private GUIStyle portfolioTitle, portfolioKicker, portfolioValue, portfolioMuted, portfolioWrap,
            portfolioCard, portfolioPill, portfolioTab, portfolioSelectedTab, portfolioStop, portfolioAccent;
        private Vector2 portfolioScroll;
        private Camera portfolioCamera;
        private Rect originalCameraRect;
        private bool cameraRectOwned, cleanGuidesApplied;
        private bool showControllerGuide;
        private bool[] savedGuideFlags;

        public static Rect PortfolioRailBounds(float width, float height, bool presentation)
        {
            if (presentation) return Rect.zero;
            float railWidth = Mathf.Min(368f, Mathf.Max(260f, width * .29f));
            return new Rect(Mathf.Max(0, width - railWidth), 80, railWidth, Mathf.Max(0, height - 80));
        }
        public static Rect PortfolioViewportBounds(float width, float height, bool presentation)
        {
            float railWidth = presentation ? 0 : PortfolioRailBounds(width, height, false).width;
            return new Rect(0, 80, Mathf.Max(0, width - railWidth), Mathf.Max(0, height - 136));
        }
        public static string PortfolioConnectionText(bool connected, bool following, string linkStatus)
        {
            if (following && connected) return "REAL ARM / FOLLOWING";
            if (!connected) return "SIMULATION";
            if (linkStatus == "Connected / held") return "CONNECTED / HELD";
            return "ROBOT LINK / CHECK STATUS";
        }
        private bool PortfolioHitTest(Vector2 p)
        {
            return p.y <= 80 || p.y >= Screen.height - 56 ||
                (!PresentationView && PortfolioRailBounds(Screen.width, Screen.height, false).Contains(p));
        }
        public void SetPortfolioView(bool presentation) { PresentationView = presentation; }
        // Both layouts operate on these same controls and runtime owners. A layout
        // switch must not issue a motion command or replace recording/link state.
        public void SetDashboardMode(bool studio)
        {
            if (portfolioDashboard == studio) return;
            if (!studio)
            {
                RestorePortfolioGuides();
                PresentationView = false;
                if (cameraRectOwned && portfolioCamera) portfolioCamera.rect = originalCameraRect;
                cameraRectOwned = false;
            }
            portfolioDashboard = studio;
            UpdatePortfolioPresentation();
        }
        public void SetPortfolioTab(int index)
        {
            PortfolioTab = Mathf.Clamp(index, 0, portfolioTabs.Length - 1);
            PresentationView = false; portfolioScroll = Vector2.zero;
        }
        private void UpdatePortfolioPresentation()
        {
            if (!portfolioDashboard)
            {
                RestorePortfolioGuides();
                if (cameraRectOwned && portfolioCamera) portfolioCamera.rect = originalCameraRect;
                cameraRectOwned = false; return;
            }
            if (PresentationView && !cleanGuidesApplied && debugVisualizer)
            {
                savedGuideFlags = new[] { debugVisualizer.showAxes, debugVisualizer.showPivots, debugVisualizer.showOrigin,
                    debugVisualizer.showTarget, debugVisualizer.showEndEffector, debugVisualizer.showTrajectory,
                    debugVisualizer.showColliderBounds, debugVisualizer.showJointLimits };
                // Presentation hides guides only. Leaving it restores the user's view.
                debugVisualizer.showAxes = debugVisualizer.showPivots = debugVisualizer.showOrigin = false;
                debugVisualizer.showTarget = debugVisualizer.showEndEffector = false;
                debugVisualizer.showTrajectory = debugVisualizer.showColliderBounds = debugVisualizer.showJointLimits = false;
                cleanGuidesApplied = true;
            }
            if (!PresentationView && cleanGuidesApplied) RestorePortfolioGuides();
            if (!portfolioCamera && orbitCamera) portfolioCamera = orbitCamera.GetComponent<Camera>();
            if (portfolioCamera && Screen.width > 0 && Screen.height > 0)
            {
                if (!cameraRectOwned) { originalCameraRect = portfolioCamera.rect; cameraRectOwned = true; }
                Rect v = PortfolioViewportBounds(Screen.width, Screen.height, PresentationView);
                portfolioCamera.rect = new Rect(v.x / Screen.width, (Screen.height - v.yMax) / Screen.height,
                    v.width / Screen.width, v.height / Screen.height);
            }
        }
        private Texture2D PortfolioTexture(Color color)
        {
            var texture = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
            texture.SetPixel(0, 0, color); texture.Apply(); portfolioTextures.Add(texture); return texture;
        }
        private void PreparePortfolioSkin()
        {
            if (portfolioSkin) return;
            portfolioSkin = Instantiate(GUI.skin); portfolioSkin.hideFlags = HideFlags.HideAndDontSave;
            Color ink = new Color(.88f, .93f, .97f), muted = new Color(.56f, .66f, .75f);
            var button = PortfolioTexture(new Color(.16f, .21f, .27f));
            var hover = PortfolioTexture(new Color(.21f, .30f, .36f));
            var card = PortfolioTexture(new Color(.09f, .13f, .18f));
            var field = PortfolioTexture(new Color(.055f, .085f, .12f));
            var accent = PortfolioTexture(new Color(.13f, .70f, .69f));
            foreach (GUIStyle style in new[] { portfolioSkin.label, portfolioSkin.button, portfolioSkin.textField,
                portfolioSkin.toggle, portfolioSkin.box })
            { style.fontSize = 13; style.normal.textColor = ink; style.hover.textColor = ink; style.active.textColor = ink; }
            portfolioSkin.button.normal.background = button; portfolioSkin.button.hover.background = hover;
            portfolioSkin.button.active.background = hover; portfolioSkin.button.onNormal.background = hover;
            portfolioSkin.button.padding = new RectOffset(10, 10, 8, 8); portfolioSkin.button.margin = new RectOffset(2, 2, 3, 3);
            portfolioSkin.textField.normal.background = field; portfolioSkin.textField.focused.background = field;
            portfolioSkin.textField.padding = new RectOffset(9, 9, 8, 8);
            portfolioSkin.box.normal.background = card; portfolioSkin.box.border = new RectOffset();
            portfolioSkin.box.padding = new RectOffset(13, 13, 12, 12);
            portfolioSkin.horizontalSlider.normal.background = PortfolioTexture(new Color(.22f, .28f, .34f));
            portfolioSkin.horizontalSlider.fixedHeight = 4; portfolioSkin.horizontalSlider.margin = new RectOffset(3, 3, 11, 11);
            portfolioSkin.horizontalSliderThumb.normal.background = accent;
            portfolioSkin.horizontalSliderThumb.hover.background = accent;
            portfolioSkin.horizontalSliderThumb.active.background = accent;
            portfolioSkin.horizontalSliderThumb.fixedWidth = 12; portfolioSkin.horizontalSliderThumb.fixedHeight = 14;
            portfolioTitle = new GUIStyle(portfolioSkin.label) { fontSize = 22, fontStyle = FontStyle.Bold };
            portfolioKicker = new GUIStyle(portfolioSkin.label) { fontSize = 10, fontStyle = FontStyle.Bold };
            portfolioKicker.normal.textColor = muted;
            portfolioValue = new GUIStyle(portfolioSkin.label) { fontSize = 18, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleRight };
            portfolioMuted = new GUIStyle(portfolioSkin.label) { fontSize = 11, wordWrap = true };
            portfolioMuted.normal.textColor = muted;
            portfolioWrap = new GUIStyle(portfolioSkin.label) { fontSize = 12, wordWrap = true };
            portfolioCard = new GUIStyle(portfolioSkin.box) { padding = new RectOffset(11, 11, 8, 8) };
            portfolioPill = new GUIStyle(portfolioSkin.box) { fontSize = 10, fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter, padding = new RectOffset(4, 4, 0, 0) };
            portfolioPill.normal.textColor = new Color(.32f, .90f, .83f);
            portfolioTab = new GUIStyle(portfolioSkin.button) { fontSize = 12, padding = new RectOffset(4, 4, 10, 10) };
            portfolioSelectedTab = new GUIStyle(portfolioTab); portfolioSelectedTab.normal.background = accent;
            portfolioSelectedTab.normal.textColor = new Color(.015f, .075f, .09f);
            portfolioStop = new GUIStyle(portfolioSkin.button) { fontStyle = FontStyle.Bold };
            portfolioStop.normal.background = PortfolioTexture(new Color(.73f, .18f, .25f));
            portfolioStop.hover.background = PortfolioTexture(new Color(.87f, .25f, .31f));
            portfolioAccent = new GUIStyle(portfolioSkin.button) { fontStyle = FontStyle.Bold };
            portfolioAccent.normal.background = accent; portfolioAccent.normal.textColor = new Color(.015f, .075f, .09f);
        }
        private void ReleasePortfolioResources()
        {
            RestorePortfolioGuides();
            if (cameraRectOwned && portfolioCamera) portfolioCamera.rect = originalCameraRect;
            foreach (var texture in portfolioTextures) if (texture) Destroy(texture);
            if (portfolioSkin) Destroy(portfolioSkin);
        }
        private void RestorePortfolioGuides()
        {
            if (cleanGuidesApplied && debugVisualizer && savedGuideFlags != null)
            {
                debugVisualizer.showAxes = savedGuideFlags[0]; debugVisualizer.showPivots = savedGuideFlags[1];
                debugVisualizer.showOrigin = savedGuideFlags[2]; debugVisualizer.showTarget = savedGuideFlags[3];
                debugVisualizer.showEndEffector = savedGuideFlags[4]; debugVisualizer.showTrajectory = savedGuideFlags[5];
                debugVisualizer.showColliderBounds = savedGuideFlags[6]; debugVisualizer.showJointLimits = savedGuideFlags[7];
            }
            cleanGuidesApplied = false; savedGuideFlags = null;
        }
        private static void PortfolioFill(Rect rect, Color color)
        {
            Color old = GUI.color; GUI.color = color; GUI.DrawTexture(rect, Texture2D.whiteTexture); GUI.color = old;
        }
        private void DrawPortfolioDashboard()
        {
            PreparePortfolioSkin(); var previousSkin = GUI.skin; GUI.skin = portfolioSkin;
            try
            {
                var link = arm ? arm.GetComponent<RobotHardwareLink>() : null;
                DrawPortfolioHeader(link);
                if (!PresentationView) DrawPortfolioRail(link);
                DrawPortfolioFooter();
            }
            finally { GUI.skin = previousSkin; }
        }
        private void DrawPortfolioHeader(RobotHardwareLink link)
        {
            string label = PortfolioConnectionText(link != null && link.Connected, link != null && link.FollowActive, link?.Status);
            PortfolioFill(new Rect(0, 0, Screen.width, 80), new Color(.035f, .055f, .08f));
            PortfolioFill(new Rect(20, 23, 4, 32), new Color(.13f, .70f, .69f));
            GUI.Label(new Rect(36, 14, 280, 31), "Career fair robot", portfolioTitle);
            GUI.Label(new Rect(37, 46, 280, 18), Screen.width >= 1020 ? "ROBOT ARM / CONTROL STUDIO" : label, portfolioKicker);
            float stopWidth = 126, right = Screen.width - 16;
            if (GUI.Button(new Rect(right - stopWidth, 20, stopWidth, 40), "HOLD / STOP", portfolioStop)) PortfolioStopAll();
            if (GUI.Button(new Rect(right - stopWidth - 112, 20, 102, 40), PresentationView ? "Controls" : "Presentation"))
                SetPortfolioView(!PresentationView);
            if (Screen.width >= 780 && GUI.Button(new Rect(right - stopWidth - 214, 20, 92, 40), "Robot link")) SetPortfolioTab(4);
            if (Screen.width >= 1020) GUI.Box(new Rect(326, 26, Mathf.Min(225, Screen.width - 730), 29), label, portfolioPill);
            if (Screen.width >= 1200 && link)
                GUI.Label(new Rect(570, 31, 230, 24), "COMMAND LIMIT  " + link.physicalSpeed.ToString("0", CultureInfo.InvariantCulture) + "°/s", portfolioKicker);
        }
        private void PortfolioStopAll()
        {
            StopAutomaticMotion(); if (arm) arm.Stop();
            var link = arm ? arm.GetComponent<RobotHardwareLink>() : null;
            if (link != null && link.Connected) link.Hold("Held by dashboard STOP");
            ClearAngleEdits(); status = "Stopped. Target follow and playback are off.";
        }
        private void DrawPortfolioRail(RobotHardwareLink link)
        {
            Rect rail = PortfolioRailBounds(Screen.width, Screen.height, false);
            PortfolioFill(rail, new Color(.055f, .08f, .115f));
            GUILayout.BeginArea(new Rect(rail.x + 14, rail.y + 14, rail.width - 28, rail.height - 28));
            for (int row = 0; row < 2; row++)
            {
                GUILayout.BeginHorizontal();
                for (int column = 0; column < 3; column++)
                {
                    int tab = row * 3 + column;
                    if (GUILayout.Button(portfolioTabs[tab], PortfolioTab == tab ? portfolioSelectedTab : portfolioTab)) SetPortfolioTab(tab);
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.Space(12);
            var webcam = arm ? arm.GetComponent<WebcamRobotTracking>() : null;
            if (webcam && GUILayout.Button(webcam.Following ? "Camera tracking · FOLLOWING" : "Camera tracking", portfolioTab)) webcam.TogglePanel();
            portfolioScroll = GUILayout.BeginScrollView(portfolioScroll);
            if (!arm) GUILayout.Label("Robot model unavailable.", portfolioWrap);
            else if (PortfolioTab == 0) DrawPortfolioJoints(link);
            else if (PortfolioTab == 1) DrawPortfolioTasks(link);
            else if (PortfolioTab == 2) DrawTargetControls();
            else if (PortfolioTab == 3) { DrawDebugControls(); GUILayout.Space(8); DrawCollisionSummary(); }
            else if (PortfolioTab == 4) DrawHardwareControls();
            else DrawPoseControls();
            GUILayout.EndScrollView();
            GUILayout.Space(8);
            GUILayout.Label(status, portfolioMuted, GUILayout.MaxHeight(48));
            GUILayout.EndArea();
        }
        private void DrawPortfolioJoints(RobotHardwareLink link)
        {
            DrawPortfolioSpeed(link); GUILayout.Space(10);
            GUILayout.Label("JOINT CONTROLS", portfolioKicker);
            GUILayout.Label("Model angles / degrees", portfolioMuted);
            EnsureAngleFields();
            for (int i = 0; i < arm.joints.Length; i++)
            {
                var joint = arm.joints[i]; if (!joint) continue;
                GUILayout.BeginVertical(portfolioCard);
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(joint.jointName, GUI.skin.label)) { SelectedJointIndex = i; JointSelected?.Invoke(joint); }
                GUILayout.Label(joint.currentAngle.ToString("0.0", CultureInfo.InvariantCulture) + "°", portfolioValue, GUILayout.Width(95));
                GUILayout.EndHorizontal();
                float target = GUILayout.HorizontalSlider(joint.targetAngle, joint.minimumAngle, joint.maximumAngle);
                if (!Mathf.Approximately(target, joint.targetAngle)) SetJoint(i, target);
                if (SelectedJointIndex == i)
                {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Target", portfolioMuted, GUILayout.Width(43));
                string field = "portfolio-angle-" + i;
                if (!angleDirty[i] && GUI.GetNameOfFocusedControl() != field) angleFields[i] = Format(joint.targetAngle);
                GUI.SetNextControlName(field);
                string input = GUILayout.TextField(angleFields[i], 12, GUILayout.Width(65));
                if (input != angleFields[i]) { angleFields[i] = input; angleDirty[i] = true; }
                if (GUILayout.Button("Set", GUILayout.Width(42)) || IsEnter(field))
                {
                    if (TryNumber(angleFields[i], out float degrees)) SetJoint(i, joint.Clamp(degrees));
                    else status = "Enter a finite joint angle.";
                }
                if (GUILayout.Button("Focus", GUILayout.Width(54)) && orbitCamera) orbitCamera.Focus(joint.transform);
                GUILayout.EndHorizontal();
                }
                GUILayout.EndVertical(); GUILayout.Space(6);
            }
            if (arm.operationalHome != null) GUILayout.Label("Home: " + arm.operationalHome.name, portfolioMuted);
            if (GUILayout.Button("Move to home")) MoveHomeFromUi(false);
        }
        private void DrawPortfolioSpeed(RobotHardwareLink link)
        {
            GUILayout.BeginVertical(portfolioCard);
            GUILayout.Label("MOTION SPEED", portfolioKicker);
            if (link)
            {
                GUILayout.Label("Robot speed  " + link.physicalSpeed.ToString("0", CultureInfo.InvariantCulture) + "°/s", portfolioWrap);
                bool wasEnabled = GUI.enabled;
                GUI.enabled = wasEnabled && link.CanEditSpeed;
                float requested = GUILayout.HorizontalSlider(link.physicalSpeed, RobotSpeedSettings.Minimum, RobotSpeedSettings.Maximum);
                if (requested != link.physicalSpeed) link.SetRequestedSpeed(Mathf.Round(requested));
                GUI.enabled = wasEnabled;
                GUILayout.BeginHorizontal();
                GUILayout.Label("1°/s", portfolioMuted); GUILayout.FlexibleSpace(); GUILayout.Label("400°/s", portfolioMuted);
                GUILayout.EndHorizontal();
                if (!link.CanEditSpeed) GUILayout.Label("Press HOLD / STOP to adjust, then resume Direct arm control.", portfolioMuted);
                if (link.Connected) GUILayout.Label("Effective command limit  " + link.EffectivePhysicalSpeed.ToString("0", CultureInfo.InvariantCulture) + "°/s", portfolioMuted);
            }
            var controller = arm.GetComponent<XboxRobotControl>();
            if (controller)
            {
                GUILayout.Label("Controller response  " + controller.controllerSpeedMultiplier.ToString("0.0", CultureInfo.InvariantCulture) + "×");
                controller.controllerSpeedMultiplier = GUILayout.HorizontalSlider(controller.controllerSpeedMultiplier, .5f, 5f);
                GUILayout.BeginHorizontal();
                controller.SetEnabledFromUi(GUILayout.Toggle(controller.controllerEnabled, "Controller enabled"));
                GUILayout.Label(controller.Connected ? "Connected" : "Not connected", portfolioMuted);
                GUILayout.EndHorizontal();
                if (GUILayout.Button(showControllerGuide ? "Controller guide  -" : "Controller guide  +", portfolioTab))
                    showControllerGuide = !showControllerGuide;
                if (showControllerGuide)
                    GUILayout.Label("Left stick: across the table\nRight stick left/right: BASE ONLY\nRight stick up/down: height\nLB / RB: wrist tilt\nLT: open gripper  /  RT: close\nB: immediate STOP", portfolioMuted);
            }
            else
            {
                GUILayout.Label("Simulation speed  " + arm.speedMultiplier.ToString("0.0", CultureInfo.InvariantCulture) + "×");
                arm.speedMultiplier = GUILayout.HorizontalSlider(arm.speedMultiplier, .5f, 5f);
            }
            if (link)
            {
                GUILayout.Label("Command limit; actual movement depends on acceleration, travel and servo capability. Your setting is saved.", portfolioMuted);
            }
            GUILayout.EndVertical();
        }
        private void DrawPortfolioTasks(RobotHardwareLink link)
        {
            GUILayout.Label("ONE-TAKE TASKS", portfolioKicker);
            GUILayout.Label("Record your arm movements, then replay one saved take.", portfolioWrap);
            if (!motionRecorder || !motionPlayer) return;
            bool recording = motionRecorder.IsRecording, playing = motionPlayer.IsPlaying, pending = motionRecorder.HasUnsavedRecording;
            bool previous = GUI.enabled;
            GUILayout.Space(8); GUILayout.BeginVertical(portfolioCard);
            GUILayout.Label(recording ? "RECORDING  " + RecordingTime(motionRecorder.ElapsedSeconds) : "New motion take", portfolioTitle);
            GUI.enabled = previous && !recording && !playing && !pending;
            GUILayout.Label("Task name (optional)", portfolioMuted);
            taskRecordingName = GUILayout.TextField(taskRecordingName, 64);
            if (GUILayout.Button("Start recording", portfolioAccent)) { motionRecorder.StartRecording(taskRecordingName, out _); status = motionRecorder.Status; }
            GUI.enabled = previous && recording;
            if (GUILayout.Button("Stop & save")) { if (motionRecorder.StopAndSave(out _)) taskRecordingName = ""; status = motionRecorder.Status; }
            GUI.enabled = previous;
            GUILayout.Label(motionRecorder.Status, portfolioMuted);
            GUILayout.Label("Saves joint motion and timing. Screen video is recorded separately.", portfolioMuted);
            if (pending)
            {
                if (GUILayout.Button("Retry saving take")) { motionRecorder.StopAndSave(out _); status = motionRecorder.Status; }
                if (GUILayout.Button("Discard unsaved take")) { motionRecorder.DiscardUnsavedRecording(out _); status = motionRecorder.Status; }
            }
            GUILayout.EndVertical(); GUILayout.Space(13);
            GUILayout.Label("SAVED TASKS", portfolioKicker);
            GUI.enabled = previous && !recording && !playing && !pending;
            for (int i = 0; i < taskRecordings.Length; i++)
            {
                var clip = taskRecordings[i];
                if (GUILayout.Button(clip.name + "   /   " + RecordingTime(clip.durationSeconds),
                    i == selectedRecording ? portfolioSelectedTab : portfolioTab)) selectedRecording = i;
            }
            if (taskRecordings.Length == 0) GUILayout.Label("Your saved takes will appear here.", portfolioMuted);
            if (GUILayout.Button("Refresh tasks")) RefreshTaskRecordings();
            bool following = link && link.FollowActive;
            GUI.enabled = previous && !recording && !playing && !pending && selectedRecording >= 0 && selectedRecording < taskRecordings.Length;
            if (GUILayout.Button(following ? "Play once / REAL ARM" : "Play once / simulation", portfolioAccent)) PlaySelectedRecording(following);
            GUI.enabled = previous;
            if (GUILayout.Button("Stop playback")) { motionPlayer.Stop(); status = motionPlayer.Status; }
            if (playing) GUILayout.Label(motionPlayer.MovingToStart ? "Moving to the take's start" :
                RecordingTime(motionPlayer.ElapsedSeconds) + " / " + RecordingTime(motionPlayer.DurationSeconds), portfolioWrap);
            GUILayout.Label(motionPlayer.Status, portfolioMuted);
            GUILayout.Label("Real-arm playback requires the existing Robot link arming checks. Manual input or HOLD stops playback.", portfolioMuted);
        }
        private void DrawPortfolioFooter()
        {
            float width = PortfolioViewportBounds(Screen.width, Screen.height, PresentationView).width;
            PortfolioFill(new Rect(0, Screen.height - 56, width, 56), new Color(.035f, .055f, .08f));
            float x = 16, y = Screen.height - 44;
            if (GUI.Button(new Rect(x, y, 68, 31), "Frame") && orbitCamera) orbitCamera.FrameRobot();
            if (GUI.Button(new Rect(x + 75, y, 60, 31), "Front") && orbitCamera) orbitCamera.SetView(180, 15);
            if (GUI.Button(new Rect(x + 142, y, 54, 31), "Side") && orbitCamera) orbitCamera.SetView(90, 15);
            if (GUI.Button(new Rect(x + 205, y, 90, 31), "Classic UI")) SetDashboardMode(false);
            if (width > 550)
            {
                string motion = motionRecorder && motionRecorder.IsRecording ? "MOTION REC  " + RecordingTime(motionRecorder.ElapsedSeconds)
                    : motionPlayer && motionPlayer.IsPlaying ? "REPLAY  " + RecordingTime(motionPlayer.ElapsedSeconds)
                    : "Drag to orbit  /  Scroll to zoom";
                GUI.Label(new Rect(330, y + 8, Mathf.Max(0, width - 340), 25), motion, portfolioMuted);
            }
        }
    }
}
