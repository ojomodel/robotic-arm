using System;
using System.IO;
using UnityEngine;

namespace CareerFair.Robot
{
    /// <summary>Local display and screenshots only; independent of robot motion.</summary>
    [DefaultExecutionOrder(-100)]
    public sealed class SimulationPresentation : MonoBehaviour
    {
        [Range(15, 120)] public int frameRate = 60;
        public string lastScreenshotPath;

        private void Awake()
        {
            QualitySettings.vSyncCount = 1;
            Application.targetFrameRate = frameRate;
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.F12)) SaveScreenshot();
        }

        public void SaveScreenshot()
        {
            string folder = Path.Combine(Application.persistentDataPath, "Screenshots");
            Directory.CreateDirectory(folder);
            lastScreenshotPath = Path.Combine(folder, "Career-fair-robot-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + ".png");
            ScreenCapture.CaptureScreenshot(lastScreenshotPath);
            Debug.Log("Simulation screenshot: " + lastScreenshotPath);
        }
    }
}
