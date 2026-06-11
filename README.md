# px4_msgs

<!-- status -->
[![Build package](https://github.com/PX4/px4_msgs/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/PX4/px4_msgs/actions/workflows/build.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/PX4/px4_msgs/blob/main/.pre-commit-config.yaml)
[![Latest release](https://img.shields.io/github/v/release/PX4/px4_msgs?logo=github&label=release&color=blue)](https://github.com/PX4/px4_msgs/releases/latest)
[![Last commit](https://img.shields.io/github/last-commit/PX4/px4_msgs/main?logo=git&logoColor=white)](https://github.com/PX4/px4_msgs/commits/main)
<!-- ecosystem -->
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling%20%7C%20Lyrical-22314E?logo=ros&logoColor=white)](https://index.ros.org/p/px4_msgs/)
[![REP-2004 Quality Level 3](https://img.shields.io/badge/Quality%20Level-3-yellowgreen)](QUALITY_DECLARATION.md)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-FE5196?logo=conventionalcommits&logoColor=white)](https://www.conventionalcommits.org)
[![License: BSD-3-Clause](https://img.shields.io/github/license/PX4/px4_msgs?color=brightgreen)](https://github.com/PX4/px4_msgs/blob/main/LICENSE)
[![PRs welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/PX4/px4_msgs/blob/main/CONTRIBUTING.md)
[![Dronecode Discord](https://img.shields.io/discord/1022170275984457759?logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/dronecode)

ROS 2 message and service definitions for the [PX4 Autopilot](https://px4.io/).

## Overview

`px4_msgs` is a ROS 2 **interface package**: it contains only `.msg` and `.srv` definitions and the build rules that turn them into the C++/Python interfaces used by ROS 2 applications. The definitions mirror PX4's internal [uORB](https://docs.px4.io/main/en/middleware/uorb.html) topics, so that a ROS 2 node can exchange them with the autopilot over the [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html) (the `uxrce_dds_client` running on PX4 talks to the `MicroXRCEAgent` running on the companion side, which publishes/subscribes these topics on the ROS 2 graph).

Because the definitions come straight from PX4, this package is the single source of truth that keeps a ROS 2 workspace ABI-compatible with a given PX4 firmware version.

## Installation

### From source (recommended today)

Clone the branch that matches your PX4 version (see [Supported versions](#supported-versions-and-compatibility)) into a colcon workspace and build:

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash      # or humble / kilted / rolling
colcon build --packages-select px4_msgs
```

See [Using colcon to build packages](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#build-a-package) and the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html) for the full workflow.

### From a binary (apt)

Once the package has completed its [build-farm release](#releasing) it is installable as a Debian package:

```sh
sudo apt install ros-${ROS_DISTRO}-px4-msgs
```

You can also build a `.deb` locally — see [Building Debian packages](#building-debian-packages).

## Usage

After sourcing the workspace, the generated interfaces are available like any other ROS 2 message package:

```cpp
#include <px4_msgs/msg/vehicle_status.hpp>
// ...
sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
  "/fmu/out/vehicle_status", rclcpp::QoS(10).best_effort(),
  [](px4_msgs::msg::VehicleStatus::SharedPtr msg) { /* ... */ });
```

```python
from px4_msgs.msg import VehicleStatus
```

> **Note:** PX4 publishes with *best-effort* QoS; subscribers must use a compatible QoS profile (as above) or they will not receive any data. Topic names and directions (`/fmu/out/...`, `/fmu/in/...`) are documented in the [PX4 uXRCE-DDS topic list](https://docs.px4.io/main/en/middleware/dds_topics.html).

## Package layout

```text
px4_msgs/
├── msg/            # message definitions (*.msg), generated from PX4 uORB
├── srv/            # service definitions (*.srv)
├── CMakeLists.txt  # globs msg/ and srv/ and calls rosidl_generate_interfaces
└── package.xml     # ament_cmake, format 3, member of rosidl_interface_packages
```

## Supported versions and compatibility

`px4_msgs` is a pure interface package, so the definitions build on **any maintained ROS 2 distribution**. What ties a checkout to a firmware is the **PX4 release line**, not the ROS 2 distribution: each PX4 release has its own branch and `main` tracks PX4 `main`. Pick the branch that matches the PX4 version you fly, then build it on whichever ROS 2 distribution you run.

### PX4 release → px4_msgs branch

| PX4 release             | px4_msgs branch                                                     |
|-------------------------|--------------------------------------------------------------------|
| `main` (in development) | [`main`](https://github.com/PX4/px4_msgs)                          |
| v1.17                   | [`release/1.17`](https://github.com/PX4/px4_msgs/tree/release/1.17) |
| v1.16                   | [`release/1.16`](https://github.com/PX4/px4_msgs/tree/release/1.16) |
| v1.15                   | [`release/1.15`](https://github.com/PX4/px4_msgs/tree/release/1.15) |
| v1.14                   | [`release/1.14`](https://github.com/PX4/px4_msgs/tree/release/1.14) |
| v1.13                   | [`release/1.13`](https://github.com/PX4/px4_msgs/tree/release/1.13) |

### ROS 2 distribution → Ubuntu

The ROS 2 distribution fixes the Ubuntu version ([REP 2000](https://www.ros.org/reps/rep-2000.html)). CI builds `main` against every maintained distribution:

| ROS 2 distribution | Ubuntu             | Type                | Built in CI                                                  |
|--------------------|--------------------|---------------------|-------------------------------------------------------------|
| Humble             | 22.04 (Jammy)      | LTS, until 2027     | yes, native runner                                          |
| Jazzy              | 24.04 (Noble)      | LTS, until 2029     | yes, native runner                                          |
| Kilted             | 24.04 (Noble)      | non-LTS, until 2026 | yes, native runner                                          |
| Rolling            | 24.04 (Noble)      | development         | yes, native runner                                          |
| Lyrical            | 26.04 (Resolute)   | LTS                 | yes, in the `ros:lyrical` container (no 26.04 runner yet)   |
| Foxy               | 20.04 (Focal)      | EOL since 2023      | no                                                          |

The definitions use only basic message features (built in types, fixed size arrays, nested messages), so any branch also builds on the older distributions if you still need them; the `release/*` branches were originally tested on Foxy, Humble and Rolling. Windows (Jazzy) and macOS (Jazzy, via [RoboStack](https://robostack.github.io/)) are covered by CI as well.

## How the definitions are kept in sync (px4_msgs quirks)

This package is **generated, not hand-written** — keep these PX4-specific points in mind:

- **Do not edit `msg/` or `srv/` here.** The definitions are copied from the [uORB definitions](https://github.com/PX4/PX4-Autopilot/tree/main/msg) in PX4-Autopilot. Fix a message at the source (PX4-Autopilot) instead. See [CONTRIBUTING.md](CONTRIBUTING.md).
- **Automatic sync.** When the uORB definitions change on PX4-Autopilot `main`, a [CI pipeline](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/metadata.yml) pushes the updated definitions here, so `main` stays in lock-step with PX4-Autopilot `main`.
- **Flat layout requirement.** The ROS 2 generation pipeline requires every definition to live **directly** under `msg/` (no sub-directories) — PX4's `msg/versioned/` files are flattened into `msg/` during the sync.
- **Manual sync** (only needed for a custom/detached PX4 version): check out the branch for the PX4 version you forked from, then refresh the definitions:
  ```sh
  rm -f msg/*.msg srv/*.srv
  cp ~/PX4-Autopilot/msg/*.msg msg/
  cp ~/PX4-Autopilot/msg/versioned/*.msg msg/
  cp ~/PX4-Autopilot/srv/*.srv srv/
  ```

## Building Debian packages

Official binaries are produced by the [ROS build farm](https://build.ros.org/) via [`bloom`](https://wiki.ros.org/bloom). To build a `.deb` locally (e.g. to self-host or test packaging), use the bundled builder container, which wraps `bloom-generate rosdebian` and writes the result to `./out`:

```sh
./scripts/build_deb_host.sh                 # default: humble
ROS_DISTRO=jazzy ./scripts/build_deb_host.sh
```

The same packaging runs in CI for every supported distribution on `amd64` and `arm64` ([`Package (.deb)`](.github/workflows/package.yml)). `.deb` packages are Debian/Ubuntu-specific; there is no equivalent on Windows or macOS.

## Releasing

Releases are automated: a GitHub release (created from an upstream PX4 version bump) triggers the `.deb` packaging and a `bloom` pull request to [`ros/rosdistro`](https://github.com/ros/rosdistro), using the [`PX4/px4_msgs-release`](https://github.com/PX4/px4_msgs-release) release repository. See the [release process](CONTRIBUTING.md#release-process) in CONTRIBUTING for the one-time setup.

Each ROS 2 distribution is released from the matching PX4 release line (see the table above), and `bloom` only accepts increasing versions per distribution. So a later patch on an older line (e.g. `1.16.3` after `1.17.0`) is released to the distributions that track `1.16`, and never overrides a newer release already published for another distribution.

## Contributing

Contributions to the packaging, build, CI, and docs are welcome — read the [contributing guide](CONTRIBUTING.md) and use the [pull request template](.github/PULL_REQUEST_TEMPLATE.md). Remember that message/service definitions are changed in [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), not here. All participants are expected to follow our [Code of Conduct](CODE_OF_CONDUCT.md).

## Quality declaration

`px4_msgs` claims Quality Level 3 — see the [Quality Declaration](QUALITY_DECLARATION.md) ([REP-2004](https://www.ros.org/reps/rep-2004.html)).

## Support

- **Bugs / features (this package):** [GitHub Issues](https://github.com/PX4/px4_msgs/issues)
- **Message/field changes:** [PX4-Autopilot issues](https://github.com/PX4/PX4-Autopilot/issues)
- **Questions / troubleshooting:** [PX4 Discord](https://discord.gg/dronecode)
- **Security:** see the [security policy](SECURITY.md)
- **Release history:** [CHANGELOG.rst](CHANGELOG.rst)

## License

Released under the [BSD 3-Clause License](LICENSE).
