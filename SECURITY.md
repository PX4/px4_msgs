# Security Policy

This package follows the ROS 2 vulnerability disclosure guidance in [REP-2006](https://www.ros.org/reps/rep-2006.html).

## Scope

`px4_msgs` contains only ROS 2 message and service definitions, which are generated and synchronized automatically from [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot). Vulnerabilities in the *content* of the definitions or in PX4 itself should be reported to the PX4 project. This policy covers the packaging, build, and CI tooling maintained in this repository.

## Reporting a Vulnerability

Please **do not** open a public GitHub issue for security-sensitive reports.

Instead, use one of the following private channels:

- Open a [private security advisory][advisory] on this repository (**Security → Report a vulnerability**), or
- Contact the maintainers listed in [`package.xml`](package.xml).

For vulnerabilities affecting PX4 more broadly, follow the [PX4 security policy](https://github.com/PX4/PX4-Autopilot/security/policy).

When reporting, please include:

- A description of the vulnerability and its potential impact.
- Steps to reproduce or a proof of concept.
- The affected branch / version (e.g. `main`, `release/1.17`).

We will acknowledge your report as soon as possible and keep you informed of the progress towards a fix.

[advisory]: https://github.com/PX4/px4_msgs/security/advisories/new
