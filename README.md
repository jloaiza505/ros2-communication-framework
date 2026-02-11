# ros2-communication-framework
Minimal multi-node ROS2 system demonstrating publisher/subscriber communication, service coordination, parameter handling, and launch orchestration in a modular architecture.

Problem Statement

Modern robotic systems depend on reliable inter-node communication, but early-stage engineers often treat ROS2 as a scripting layer rather than a distributed system. This project addresses the need to understand deterministic message passing, service coordination, and launch orchestration in a modular ROS2 architecture. The goal is to design a minimal yet extensible multi-node system demonstrating topic publishing, service control, parameter handling, and structured launch configuration to establish a correct foundation for scalable robotic software.

## Build and run

From project root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run nodes in separate terminals:

```bash
cd ros2_ws
source install/setup.bash
ros2 run talker_pkg talker_node
```

```bash
cd ros2_ws
source install/setup.bash
ros2 run listener_pkg listener_node
```

```bash
cd ros2_ws
source install/setup.bash
ros2 run service_pkg service_server
```

```bash
cd ros2_ws
source install/setup.bash
ros2 run service_pkg service_client true
```

## Troubleshooting

If you see `No executable found`, rebuild and re-source:

```bash
cd ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables talker_pkg
ros2 pkg executables listener_pkg
ros2 pkg executables service_pkg
```
