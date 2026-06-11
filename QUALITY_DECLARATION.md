This document is a declaration of software quality for the `px4_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `px4_msgs` Quality Declaration

The package `px4_msgs` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`px4_msgs` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`px4_msgs` is at a stable version, i.e. `>= 1.0.0`. The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message and service definitions found in the `msg/` and `srv/` directories make up the public API of the package. These definitions are generated from, and kept in sync with, the [uORB message definitions](https://github.com/PX4/PX4-Autopilot/tree/main/msg) in the PX4-Autopilot repository.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`px4_msgs` will not break public API within a released ROS 2 distribution, i.e. no major releases once the ROS distribution is released. Each PX4 release line is tracked on a dedicated `release/*` branch, as documented in the [README](README.md).

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`px4_msgs` contains generated C/C++ code and therefore is subject to ABI breaks. ABI stability is maintained within a released ROS 2 distribution by only making message changes on the development (`main`) and unreleased branches.

## Change Control Process [2]

`px4_msgs` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, following the [contributing guidelines](CONTRIBUTING.md). Message and service definitions are not edited directly here; they are synchronized from PX4-Autopilot.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers). The CI is configured in [`.github/workflows/build.yml`](.github/workflows/build.yml) and builds the package against the ROS 2 distributions currently on the build farm.

### Documentation Policy [2.v]

All pull requests that change the public API must update documentation accordingly. As the public API is generated from upstream uORB definitions, documentation for the messages is maintained alongside those definitions in PX4-Autopilot.

## Documentation [3]

### Feature Documentation [3.i]

`px4_msgs` provides ROS 2 message and service definitions. Usage is documented in the [README](README.md) and in the [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html).

### Public API Documentation [3.ii]

The public API consists of the message and service definitions, which are self-documenting and carry inline comments inherited from the upstream uORB definitions.

### License [3.iii]

The license for `px4_msgs` is BSD-3-Clause, and a summary can be found in the [package.xml](package.xml). The full license text is in [LICENSE](LICENSE). There is an automated test (`ament_copyright`) that runs to confirm the license headers, configured via the `BUILD_TESTING` block of [CMakeLists.txt](CMakeLists.txt).

### Copyright Statement [3.iv]

The copyright holders are listed in [LICENSE](LICENSE).

## Testing [4]

As `px4_msgs` is an interface-only package, it does not contain feature or unit tests beyond linting and the verification that all interfaces build and generate type support successfully across all target platforms via CI.

### Feature Testing [4.i] / Public API Testing [4.ii]

Not applicable. Successful generation of the interfaces in CI exercises the public API.

### Coverage [4.iii] / Performance [4.iv]

Not applicable for an interface-only package.

### Linters and Static Analysis [4.v]

`px4_msgs` uses and passes all the standard linters and static analysis tools for an interface package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis). The linters are enabled through `ament_lint_auto` / `ament_lint_common` in the `BUILD_TESTING` block of [CMakeLists.txt](CMakeLists.txt) and additionally via [`pre-commit`](.pre-commit-config.yaml).

## Dependencies [5]

`px4_msgs` has the following runtime ROS dependencies, which are at or above Quality Level 3:

* `builtin_interfaces`
* `rosidl_default_runtime`

It has several "buildtool" dependencies, which do not affect the resulting quality, and "test" dependencies that are only used to test the package itself.

## Platform Support [6]

`px4_msgs` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against them via CI.

## Security [7]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html). See [SECURITY.md](SECURITY.md).
