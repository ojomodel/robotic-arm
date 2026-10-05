# Unity control software

This folder contains **35 existing C# source files** from the project's `Assets/Scripts` directory. They power the robot's joint controls, 3D visualization, inverse kinematics, Xbox input, task recording and USB connection to the ESP32.

This is a source bundle for code review, not a complete, ready-to-open Unity project. Scenes, CAD meshes, materials, serialized joint references, local calibration and recordings are not included. The original scripts are preserved unchanged.

## Start here

| Component | Purpose |
| --- | --- |
| [RobotArm](Runtime/RobotArm.cs), [RobotJoint](Runtime/RobotJoint.cs), [GripperLinkage](Runtime/GripperLinkage.cs) | Joint state, bounded motion, Home targets and linked gripper geometry. |
| [ForwardKinematics](Runtime/ForwardKinematics.cs), [InverseKinematics](Runtime/InverseKinematics.cs) | CAD-hierarchy forward kinematics, bounded position-only IK and a continuous local solver. |
| [XboxRobotControl](Runtime/XboxRobotControl.cs), [CartesianArmJog](Runtime/CartesianArmJog.cs) | Controller input, incremental Cartesian movement and manual takeover. |
| [RobotHardwareLink](Runtime/RobotHardwareLink.cs), [UsbRobotTransport](Runtime/UsbRobotTransport.cs) | Explicit connection/arming, command telemetry, CF1 protocol negotiation, USB streaming and HOLD handling. |
| [RobotHardwareMapping](Runtime/RobotHardwareMapping.cs), [VerifiedJointTravel](Runtime/VerifiedJointTravel.cs) | Model-to-servo command mapping and checked command intervals. |
| [MotionRecordingCapture](Runtime/MotionRecordingCapture.cs), [MotionRecordingPlayer](Runtime/MotionRecordingPlayer.cs) | Timestamped five-channel recording and once-only replay, including pauses and cancellation. |
| [RobotControlPanel](Runtime/RobotControlPanel.cs), [RobotPortfolioDashboard](Runtime/RobotPortfolioDashboard.cs) | Classic and Studio control interfaces. |

The remaining files supply motion planning, pose storage, speed preferences, camera orbit, visualization and shared helpers. Older waypoint and coordinated-jog implementations remain in the bundle; the current interface uses continuous recordings and `CartesianArmJog`.

## Implementation and validation status

| Feature | Evidence and limits |
| --- | --- |
| Joint interface, Xbox input and USB arm control | Implemented. Physical controller-driven movement was tested during development and reported to follow the Unity model. This does not establish measured positioning accuracy. |
| IK and Cartesian movement | Implemented and exercised in native Unity software checks. IK solves position within configured model bounds; it does not guarantee tool orientation or collision clearance. Failure to converge does not prove a target impossible. |
| Task recording and playback | Implemented and exercised with model and fake-transport checks. A completed physical pick-and-place replay has not been independently validated. Recorded angles and returned telemetry are commands, not measured shaft positions. |
| Motion timing and command limits | Implemented in the Unity/firmware path. The selectable command-rate cap is not a measured mechanical speed or payload specification. |
| Physical calibration and repeatability | Further work: wider verified travel, coordinated-motion checks, repeatability measurement and mechanical refinement. Direct operator limits are distinct from physically checked travel. |

These are existing development results, not a fresh test run of this extracted bundle. There are no payload, torque, accuracy or repeatability benchmarks claimed here.

## Dependencies and reconstruction

The source project records **Unity 6000.5.8f1** and uses `UnityEngine`, legacy input and IMGUI. Its package manifest includes the built-in `imgui`, `jsonserialize`, `physics`, `imageconversion`, `screencapture` and `unitywebrequest` modules. JSON stores also use .NET `System.Runtime.Serialization`.

Xbox input calls Windows `xinput1_4.dll`; the serial transport calls Windows `kernel32.dll`. The hardware path is therefore Windows-specific. It connects to the matching [ESP32 firmware](../firmware/) over USB using the project's CF1 protocol.

Reconstructing the simulation requires an appropriate Unity project, the original CAD-derived hierarchy, five configured joint components, gripper links, tool/target transforms and Inspector references. Those assets are not recreated here. No personal calibration profile is supplied; model angles must not be sent as raw servo angles.

The dashboard also references an experimental, **simulation-only** webcam integration. Its two C# files are retained to preserve those source dependencies. The separate Python camera helper, virtual environment and model assets are not included, so that optional feature cannot run from this bundle.

Some early simulation-only tooltips remain in the original model classes. The later hardware connection is implemented separately in `RobotHardwareLink`; those historical strings do not describe the complete application.

## Hardware behavior

Connecting does not automatically arm or home the physical robot. HOLD retains the last holding command; disabling servo signals is a separate operation. Reconnection does not automatically resume motion. The arm has no encoder feedback, automatic collision interlock or servo-supply sensing, and its servo power is external to the ESP32.

See the [portfolio case study](https://oluwaseuno.com/robotic-arm.html) for the assembled robot and demonstration.
