# About this source snapshot

This repository was organized from the existing robot project for public review in October 2026. Its commits record that publication work; they do not recreate a historical development timeline.

## Included source

- The ESP32 `UnityRobotBridge` sketch, its three local headers, and eight existing C++ test/stub files were copied from the working project's firmware directory.
- The 35 Unity runtime C# files were copied from the existing project's `Assets/Scripts` tree. They are grouped in `unity/Runtime` for this review bundle; namespaces and source contents are unchanged.
- Documentation in this repository describes the snapshot. It does not replace the original project's calibration records or imply a fresh hardware validation.

The code was already present before this repository was prepared. No replacement firmware, IK implementation or test results were invented to fill gaps.

## Deliberately omitted

- Unity `Library`, `Temp`, `Logs`, build outputs, executable players, Editor automation, and recovery files.
- Scene files, serialized CAD hierarchy and meshes, local calibration profiles, controller preferences, and personal motion recordings.
- Superseded firmware/calibration utilities, machine-specific deployment scripts, serial logs, local evidence archives, and generated binaries.
- Native Fusion archives, large STEP/STL exports and imported component models whose redistribution rights were not established for this publication.
- The large demo video and full photo collection; the portfolio hosts the video and the repository includes a few compressed images.

The original Unity scene and CAD assets do exist locally, but they are not part of this source bundle. There is no complete reconstruction package here. The exact external servo supply model/rating and a finished electrical schematic were not established from the selected source material, so the electronics notes avoid inventing them.

## Validation boundaries

Project records support existing software checks and physical controller-driven movement. The repository preparation itself did not connect to, reset, flash, arm or move the robot. Historical software checks and command telemetry are not measurements of shaft position, load, collision clearance or repeatability.

In particular, implemented recording/playback and IK code should not be read as proof of a physically validated autonomous pick-and-place system. Those distinctions are reflected in the main README's status section.

## License

No new open-source license was selected. Existing source is presented for review without adding a grant of reuse rights. Third-party dependencies are referenced rather than vendored and retain their own licenses.
