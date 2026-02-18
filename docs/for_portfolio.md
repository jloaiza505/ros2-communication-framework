# ROS2 Communication Framework Portfolio

## Title
**ROS2 Communication Framework (Beginner Project)**

![Hero - ROS 2 communication framework bringup](assets/images/ros2-comm-framework-hero.png)

## Problem Statement
This project is a small ROS 2 setup to practice core communication patterns:
- publish/subscribe
- service calls
- launch files

It has three packages:
- `talker_pkg` publishes text messages
- `listener_pkg` receives those messages
- `service_pkg` can turn the talker on or off at runtime

## System Architecture
Nodes and interfaces:
- `talker_node` publishes `std_msgs/String` on `chatter`
- `listener_node` subscribes to `chatter`
- `service_server` provides `toggle_talker` (`std_srvs/SetBool`)
- `service_server` publishes `talker_enabled` (`std_msgs/Bool`)
- `talker_node` listens to `talker_enabled` to stop/start publishing

![Architecture - node/topic/service flow](assets/images/ros2-comm-framework-architecture.png)
![Optional demo - runtime toggle behavior](assets/gifs/ros2-comm-framework-toggle-demo.gif)

## Technical Implementation
`ros2_ws/src/service_pkg/launch/framework.launch.py` starts:
- `talker_node`
- `listener_node`
- `service_server`

Talker parameters:
- `message_prefix` (default: `Hello ROS2`)
- `publish_period_sec` (default: `1.0`)

Flow:
1. Talker publishes messages on a timer.
2. Listener prints received messages.
3. Client sends `true/false` to `toggle_talker`.
4. Server publishes the enabled state.
5. Talker updates its internal state.

### Software Stack
- ROS 2 Humble
- Python with `rclpy`
- `std_msgs` and `std_srvs`
- `colcon`

### Core Engineering Decisions
- Keep packages simple and separate by role (`talker_pkg`, `listener_pkg`, `service_pkg`)
- Use a service to control talker state instead of restarting nodes
- Use launch arguments to change message text and frequency

## Key Challenges and Solutions
- Challenge: managing communication across multiple nodes
  - Solution: defined one topic/service per function and documented them in `docs/architecture.md`
- Challenge: controlling publish behavior while running
  - Solution: `toggle_talker` service plus `talker_enabled` topic

## Results
What works:
- One-command launch for all core nodes
- Talker/listener message exchange on `chatter`
- Runtime talker on/off control using service calls

Evidence images:
- ![Result - chatter messages received in listener logs](assets/images/ros2-comm-framework-result-listener-logs.png)
- ![Result - service toggle request/response and state change](assets/images/ros2-comm-framework-result-service-toggle.png)

## Reproducibility
From repository root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch service_pkg framework.launch.py
```

Run with custom launch parameters:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch service_pkg framework.launch.py message_prefix:="Portfolio Demo" publish_period_sec:=0.5
```

Toggle talker state from another terminal:

```bash
cd ros2_ws
source install/setup.bash
ros2 run service_pkg service_client false
ros2 run service_pkg service_client true
```

![Reproducibility - local build and launch terminals](assets/images/ros2-comm-framework-reproducibility-setup.png)

Key files for replication:
- `README.md`
- `docs/architecture.md`
- `ros2_ws/src/service_pkg/launch/framework.launch.py`
- `ros2_ws/src/talker_pkg/talker_pkg/talker_node.py`
- `ros2_ws/src/listener_pkg/listener_pkg/listener_node.py`
- `ros2_ws/src/service_pkg/service_pkg/service_server.py`
- `ros2_ws/src/service_pkg/service_pkg/service_client.py`
- `ros2_ws/src/talker_pkg/package.xml`
- `ros2_ws/src/listener_pkg/package.xml`
- `ros2_ws/src/service_pkg/package.xml`

## What to Improve
- Add QoS settings as launch/parameter options for experimentation
- Add a small script or doc section to collect demo screenshots automatically
- Improve logging so it is easier to track state changes during demos
