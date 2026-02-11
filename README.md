# ROS2 Communication Framework

A modular ROS2 (Humble) project that demonstrates publish/subscribe messaging, service-driven coordination, parameter handling, and launch-based orchestration across multiple Python packages.

## Why This Project

Robotic systems are distributed software systems. This repository focuses on core ROS2 communication patterns in a structure that is easy to extend and review in a portfolio.

## Features

- `talker_pkg`: publishes messages on `chatter`
- `listener_pkg`: subscribes to `chatter`
- `service_pkg`: exposes `toggle_talker` (`std_srvs/SetBool`) and broadcasts talker state on `talker_enabled`
- Parameterized talker node (`message_prefix`, `publish_period_sec`)
- Single launch entrypoint for system bringup
- Lint + unit tests with `colcon test`

## Architecture

See `docs/architecture.md` for node/topic/service flow.

## Tech Stack

- ROS2 Humble
- Python (`rclpy`)
- `colcon`
- `pytest` + ROS2 ament linters

## Quickstart

From repository root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch all core nodes:

```bash
ros2 launch service_pkg framework.launch.py
```

Optional launch arguments:

```bash
ros2 launch service_pkg framework.launch.py message_prefix:="Portfolio Demo" publish_period_sec:=0.5
```

Toggle talker publishing from another terminal:

```bash
cd ros2_ws
source install/setup.bash
ros2 run service_pkg service_client false
ros2 run service_pkg service_client true
```

## Verification

Run all tests:

```bash
cd ros2_ws
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Repository Layout

```text
ros2_ws/
  src/
    talker_pkg/
    listener_pkg/
    service_pkg/
docs/
  architecture.md
.github/workflows/
  ci.yml
```

## Troubleshooting

If executables are missing, rebuild and re-source:

```bash
cd ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

If ROS2 cannot resolve packages, confirm:

```bash
ros2 pkg list | rg 'talker_pkg|listener_pkg|service_pkg'
```

## License

MIT. See `LICENSE`.
