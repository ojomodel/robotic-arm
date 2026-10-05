using System;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Mouse-only scene navigation and a camera-plane target drag. Unity units are metres.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Camera))]
    public sealed class RobotOrbitCamera : MonoBehaviour
    {
        public RobotArm arm;
        public RobotControlPanel controlPanel;
        public Transform focusTarget;
        public bool followFocus;
        public float orbitDegreesPerPixel = 0.22f;
        public float zoomSensitivity = 0.14f;
        public float minimumDistance = 0.08f;
        public float maximumDistance = 3f;
        public float targetPickRadiusPixels = 20f;
        public bool allowTargetDrag = true;

        public event Action TargetDragged;
        public bool IsDraggingTarget { get; private set; }
        public Vector3 FocusPoint => focusPoint;

        private Camera viewCamera;
        private Vector3 focusPoint;
        private Vector3 lastMouse;
        private Vector3 dragOffset;
        private Plane dragPlane;
        private float distance;
        private float yaw;
        private float pitch;
        private bool initialized;
        private bool draggingOrbit;

        private void Start()
        {
            if (arm == null) arm = FindAnyObjectByType<RobotArm>();
            if (controlPanel == null) controlPanel = FindAnyObjectByType<RobotControlPanel>();
            Initialize();
        }

        private void Initialize()
        {
            if (initialized) return;
            viewCamera = GetComponent<Camera>();
            focusPoint = focusTarget != null ? focusTarget.position
                : arm != null && arm.robotOrigin != null ? arm.robotOrigin.position + Vector3.up * 0.18f
                : Vector3.up * 0.18f;
            Vector3 offset = transform.position - focusPoint;
            distance = Mathf.Clamp(offset.magnitude, minimumDistance, maximumDistance);
            if (offset.sqrMagnitude < 0.000001f) offset = new Vector3(0.65f, 0.45f, -0.65f);
            Quaternion direction = Quaternion.LookRotation(-offset.normalized, Vector3.up);
            yaw = direction.eulerAngles.y;
            pitch = Mathf.DeltaAngle(0f, direction.eulerAngles.x);
            pitch = Mathf.Clamp(pitch, -85f, 85f);
            lastMouse = Input.mousePosition;
            initialized = true;
        }

        public void Focus(Transform joint)
        {
            if (joint == null) return;
            Initialize();
            focusTarget = joint;
            followFocus = true;
            focusPoint = joint.position;
            ApplyView();
        }

        public void FrameRobot()
        {
            Initialize();
            focusTarget = null;
            followFocus = false;
            if (arm != null)
            {
                Renderer[] renderers = arm.GetComponentsInChildren<Renderer>();
                bool hasBounds = false;
                Bounds bounds = new Bounds();
                foreach (Renderer renderer in renderers)
                {
                    if (!renderer.enabled || !renderer.gameObject.activeInHierarchy) continue;
                    if (arm.target != null && (renderer.transform == arm.target || renderer.transform.IsChildOf(arm.target))) continue;
                    if (!hasBounds) { bounds = renderer.bounds; hasBounds = true; }
                    else bounds.Encapsulate(renderer.bounds);
                }
                if (hasBounds)
                {
                    focusPoint = bounds.center;
                    float halfFov = Mathf.Deg2Rad * viewCamera.fieldOfView * 0.5f;
                    float horizontalFov = Mathf.Atan(Mathf.Tan(halfFov) * viewCamera.aspect);
                    float fitFov = Mathf.Min(halfFov, horizontalFov);
                    distance = Mathf.Clamp(bounds.extents.magnitude / Mathf.Max(0.1f, Mathf.Sin(fitFov)) * 1.15f,
                        minimumDistance, maximumDistance);
                }
                else if (arm.robotOrigin != null) focusPoint = arm.robotOrigin.position + Vector3.up * 0.18f;
            }
            yaw = 145f;
            pitch = 23f;
            ApplyView();
        }

        public void OrbitByDegrees(float yawDelta, float pitchDelta)
        {
            if (!JointMotion.IsFinite(yawDelta) || !JointMotion.IsFinite(pitchDelta)) return;
            Initialize();
            yaw = Mathf.Repeat(yaw + yawDelta, 360f);
            pitch = Mathf.Clamp(pitch + pitchDelta, -85f, 85f);
            ApplyView();
        }

        public void SetView(float yawDegrees, float pitchDegrees)
        {
            if (!JointMotion.IsFinite(yawDegrees) || !JointMotion.IsFinite(pitchDegrees)) return;
            Initialize();
            yaw = Mathf.Repeat(yawDegrees, 360f);
            pitch = Mathf.Clamp(pitchDegrees, -85f, 85f);
            ApplyView();
        }

        private void LateUpdate()
        {
            Initialize();
            Vector3 mouse = Input.mousePosition;
            Vector3 delta = mouse - lastMouse;
            lastMouse = mouse;
            bool overPanel = controlPanel != null && controlPanel.IsPointerOverPanel(mouse);
            bool viewChanged = false;
            if (followFocus && focusTarget != null)
            {
                if ((focusPoint - focusTarget.position).sqrMagnitude > 0.000000001f) viewChanged = true;
                focusPoint = focusTarget.position;
            }

            if (Input.GetMouseButtonUp(0) || !Application.isFocused)
            { IsDraggingTarget = false; draggingOrbit = false; }
            if (!Application.isFocused) return;

            if (Input.GetMouseButtonDown(0)) draggingOrbit = !overPanel;

            if (!overPanel && allowTargetDrag && (controlPanel == null || controlPanel.TargetMarkerVisible)
                && arm != null && arm.target != null && Input.GetMouseButtonDown(0))
            {
                Vector3 point = viewCamera.WorldToScreenPoint(arm.target.position);
                if (point.z > 0 && Vector2.Distance(new Vector2(point.x, point.y), mouse) <= targetPickRadiusPixels)
                {
                    dragPlane = new Plane(transform.forward, arm.target.position);
                    Ray ray = viewCamera.ScreenPointToRay(mouse);
                    if (dragPlane.Raycast(ray, out float enter))
                    {
                        dragOffset = arm.target.position - ray.GetPoint(enter);
                        IsDraggingTarget = true;
                        draggingOrbit = false;
                    }
                }
            }
            if (IsDraggingTarget && Input.GetMouseButton(0) && !overPanel && arm != null && arm.target != null)
            {
                Ray ray = viewCamera.ScreenPointToRay(mouse);
                if (dragPlane.Raycast(ray, out float enter))
                {
                    arm.target.position = ray.GetPoint(enter) + dragOffset;
                    TargetDragged?.Invoke();
                }
            }

            if (!overPanel && !IsDraggingTarget)
            {
                if (Input.GetMouseButton(1) || (draggingOrbit && Input.GetMouseButton(0)))
                {
                    // A button's first frame may follow a large cursor move onto the window.
                    if (!Input.GetMouseButtonDown(1) && !Input.GetMouseButtonDown(0))
                    {
                        yaw = Mathf.Repeat(yaw + delta.x * orbitDegreesPerPixel, 360f);
                        pitch = Mathf.Clamp(pitch - delta.y * orbitDegreesPerPixel, -85f, 85f);
                        viewChanged = true;
                    }
                }
                if (Input.GetMouseButton(2) && !Input.GetMouseButtonDown(2))
                {
                    float unitsPerPixel = 2f * distance * Mathf.Tan(viewCamera.fieldOfView * 0.5f * Mathf.Deg2Rad)
                        / Mathf.Max(1, Screen.height);
                    focusPoint -= (transform.right * delta.x + transform.up * delta.y) * unitsPerPixel;
                    followFocus = false;
                    viewChanged = true;
                }
                float scroll = Input.mouseScrollDelta.y;
                if (Mathf.Abs(scroll) > 0.001f)
                {
                    distance = Mathf.Clamp(distance * Mathf.Exp(-scroll * zoomSensitivity), minimumDistance, maximumDistance);
                    viewChanged = true;
                }
            }
            if (viewChanged) ApplyView();
        }

        private void ApplyView()
        {
            Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
            transform.SetPositionAndRotation(focusPoint - rotation * Vector3.forward * distance, rotation);
        }

        private void OnDisable() { IsDraggingTarget = false; draggingOrbit = false; }
    }
}
