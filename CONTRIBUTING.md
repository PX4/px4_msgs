# Contributing

Thanks for your interest in contributing to `px4_msgs`!

## Message and service definitions are auto-generated

**Do not** commit changes directly to this repository that modify the message or service definitions. All `*.msg` and `*.srv` files are generated from the [uORB message definitions](https://github.com/PX4/PX4-Autopilot/tree/main/msg) in the [PX4-Autopilot repository](https://github.com/PX4/PX4-Autopilot).

The definitions here are **synchronized automatically**: a [CI/CD pipeline](https://github.com/PX4/PX4-Autopilot/blob/main/.github/workflows/metadata.yml) in PX4-Autopilot pushes the updated ROS message definitions to this repository whenever the upstream definitions change, and a new versioned set is published when a PX4 release is created. As a result, the `main` branch here is kept in sync with the `main` branch of PX4-Autopilot.

Any fix or improvement to a message or service definition must therefore be made on the uORB message files in PX4-Autopilot — follow the [PX4 contributing guide](https://github.com/PX4/PX4-Autopilot/blob/main/CONTRIBUTING.md).

## Contributing to the repository infrastructure

Changes to the packaging, build system, CI, or documentation **are** welcome here. To contribute:

1. Fork the repository and create a topic branch off `main`.
2. Make your changes and update the [`CHANGELOG.rst`](CHANGELOG.rst) when appropriate.
3. Ensure the package still builds and tests pass:
   ```sh
   colcon build --packages-select px4_msgs
   colcon test --packages-select px4_msgs
   ```
4. (Optional but recommended) install and run [`pre-commit`](https://pre-commit.com):
   ```sh
   pre-commit install
   pre-commit run --all-files
   ```
5. Open a pull request, filling in the [pull request template](.github/PULL_REQUEST_TEMPLATE.md).

### Commit conventions

Please write [Conventional Commits](https://www.conventionalcommits.org/) and sign off your work under the [Developer Certificate of Origin](https://developercertificate.org/) using `git commit -s`.

### Code of Conduct

By participating, you are expected to uphold our [Code of Conduct](CODE_OF_CONDUCT.md).

## Release process

Releases are automated end to end:

1. The upstream PX4 message-sync bot updates `main`/`release/**` and bumps the package version. The [`Create GitHub release`](.github/workflows/create-release.yml) workflow detects a new `vX.Y.Z` and publishes a GitHub release, with notes auto-generated from the message/service diff against the previous release by [`scripts/generate_release_notes.sh`](scripts/generate_release_notes.sh).
2. Publishing a release triggers the [`Package (.deb)`](.github/workflows/package.yml) workflow, which builds the Debian packages for every supported ROS 2 distribution and architecture and attaches them to the release.
3. It also triggers the [`Release to rosdistro (bloom)`](.github/workflows/release.yml) workflow, which opens the release pull request against [`ros/rosdistro`](https://github.com/ros/rosdistro) so the package is built by the official ROS build farm.

The in-repo [`CHANGELOG.rst`](CHANGELOG.rst) can be regenerated from the tag history at any time with [`scripts/generate_changelog.sh --in-place`](scripts/generate_changelog.sh).

### One-time setup required

These steps must be done once by a maintainer before the automation is fully operational:

- Releases use a single release repository, [`PX4/px4_msgs-release`](https://github.com/PX4/px4_msgs-release). (The separate `PX4/px4_msgs2-release` was a ROS 2-only split; it has been archived in favour of the single `px4_msgs-release` and is not referenced by any current ROS distribution.) Seed a bloom track for each current distribution (`humble`, `jazzy`, `kilted`, `rolling`) once with an interactive `bloom-release` using `main` as the upstream devel branch, e.g.:
  ```sh
  bloom-release --rosdistro jazzy --new-track px4_msgs
  ```
- That first run also opens the pull request that adds the `release:` block for `px4_msgs` to [`ros/rosdistro`](https://github.com/ros/rosdistro) (today it has only `source:`/`doc:`, so the package is not yet built as binaries on the farm). After the tracks and the `release:` block exist, the [`Release to rosdistro (bloom)`](.github/workflows/release.yml) workflow automates subsequent releases.
- Add a `BLOOM_GITHUB_TOKEN` repository secret: a token that can push to `PX4/px4_msgs-release` and open a pull request on `ros/rosdistro`.
- Add a `RELEASE_TOKEN` repository secret (a PAT) so that releases created by the automation cascade into the packaging and bloom workflows. Releases created with the default `GITHUB_TOKEN` do not trigger downstream workflows.
