# ROS2 Communication Framework

A modular ROS2 project demonstrating pub/sub messaging, service-driven coordination, parameter handling, and launch-based orchestration.

## Packages

- `talker_pkg`: publishes messages on `/chatter`
- `listener_pkg`: subscribes to `/chatter`
- `service_pkg`: exposes `/toggle_talker` (`std_srvs/SetBool`) and publishes talker state on `/talker_enabled`

## Workspace Layout

```text
src/
  talker_pkg/
  listener_pkg/
  service_pkg/
docs/
  architecture.md
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src
source install/setup.bash
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch service_pkg framework.launch.py
```

Optional launch args:

```bash
ros2 launch service_pkg framework.launch.py message_prefix:="Portfolio Demo" publish_period_sec:=0.5
```

## Service Toggle

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run service_pkg service_client false
ros2 run service_pkg service_client true
```

## Verify

```bash
ros2 topic echo /chatter
ros2 service call /toggle_talker std_srvs/srv/SetBool "{data: false}"
ros2 service call /toggle_talker std_srvs/srv/SetBool "{data: true}"
```

## CI

GitHub Actions workflow: `.github/workflows/ci.yml`
- ROS 2 Jazzy setup
- `rosdep install --from-paths src --ignore-src -r -y`
- `colcon build --base-paths src`
- `colcon test --base-paths src`
- `colcon test-result --verbose`

## Troubleshooting

If you see `Package 'service_pkg' not found`, rebuild and re-source:

```bash
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --base-paths src
source install/setup.bash
ros2 pkg prefix service_pkg
```

## License

MIT. See `LICENSE`.
