# Contributing

Thank you for helping improve this project.

## Rules for Contributions

- Keep changes scoped to ROS 2 Humble, Gazebo, RTAB-Map, Nav2, and the existing
  package structure unless a larger design change is discussed first.
- Do not commit generated workspace folders such as `build/`, `install/`, or
  `log/`.
- Do not commit secrets, private datasets, local maps, RTAB-Map databases,
  ROS bags, or machine-specific configuration.
- Run formatting, linting, and package tests before submitting a pull request.
- Include clear reproduction steps for bug fixes and validation steps for
  behavior changes.

## Pull Request Checklist

- The change builds with `colcon build`.
- Relevant tests or manual validation steps are included.
- The README is updated when commands, topics, launch files, or behavior change.
- Security-sensitive information has been removed from logs and screenshots.
- Third-party assets include proper attribution and license information.
