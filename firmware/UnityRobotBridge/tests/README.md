# Existing firmware host tests

These are existing project test sources, copied unchanged. They do not open a serial port or move hardware.

| Test | Scope |
| --- | --- |
| `bridge_tests.cpp` | CF1 parsing, startup Home, state changes, bounds, sequences, watchdog and release |
| `hardware_wrapper_tests.cpp` | Actual sketch compiled against mocks; output-off ordering, explicit enable, NVS read-only access and I2C failure handling |
| `synchronized_motion_tests.cpp` | Joint-space segment timing, retargeting, duplicate commands, rate limits and simulated packet streams |

From the repository root, with a C++17 compiler available:

```sh
mkdir -p build/tests
g++ -std=c++17 -Wall -Wextra -Werror -pedantic firmware/UnityRobotBridge/tests/bridge_tests.cpp -o build/tests/bridge_tests
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I firmware/UnityRobotBridge/tests/stubs firmware/UnityRobotBridge/tests/hardware_wrapper_tests.cpp -o build/tests/hardware_wrapper_tests
g++ -std=c++17 -Wall -Wextra -Werror -pedantic firmware/UnityRobotBridge/tests/synchronized_motion_tests.cpp -o build/tests/synchronized_motion_tests
./build/tests/bridge_tests
./build/tests/hardware_wrapper_tests
./build/tests/synchronized_motion_tests
```

The commands above use a POSIX shell. On Windows, use equivalent directory creation and executable names supported by your compiler.

**Validation status:** these host tests were not rerun when preparing this repository. The last recorded host compiler attempt for this revision was blocked by the development machine's Windows Application Control. Earlier test-result files and executables were deliberately excluded so they cannot be mistaken for current passing results. Existing native ESP32 compilation and servo-power-off device checks are separate evidence; they do not establish physical performance. These three tests are not a complete test suite for every timestamped trajectory behavior.
