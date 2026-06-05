# Security Policy

## Supported Versions

Security fixes are handled for the latest code on the default branch.

## Reporting a Vulnerability

Please do not open a public GitHub issue for security vulnerabilities.

Report suspected vulnerabilities privately through GitHub's private
vulnerability reporting feature, if enabled for this repository, or contact
the maintainer directly.

Include:

- A short description of the issue
- Steps to reproduce
- Affected files, commands, topics, launch files, or runtime configuration
- Potential impact
- Any suggested fix, if known

## Secrets and Credentials

Do not commit:

- API keys, tokens, passwords, SSH keys, or cloud credentials
- ROS bags containing private sensor data
- Local maps, RTAB-Map databases, logs, datasets, or calibration captures that
  contain sensitive environment information
- Personal machine paths or workstation-specific credentials

Enable GitHub secret scanning and push protection in repository settings before
accepting external contributions.

## Robotics Safety Notice

This project is a research and simulation prototype. Validate all navigation,
perception, mapping, and velocity-limiting behavior in simulation before using
it on physical hardware. Use emergency stop hardware and supervised testing
for any real robot deployment.
