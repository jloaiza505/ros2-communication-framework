# Architecture

## Node Graph

- `talker_node` (`talker_pkg`)
- `listener_node` (`listener_pkg`)
- `service_server` (`service_pkg`)
- `service_client` (`service_pkg`, ad-hoc CLI requester)

## Communication Contracts

- Topic: `chatter` (`std_msgs/String`)
  - Publisher: `talker_node`
  - Subscriber: `listener_node`

- Topic: `talker_enabled` (`std_msgs/Bool`)
  - Publisher: `service_server`
  - Subscriber: `talker_node`

- Service: `toggle_talker` (`std_srvs/SetBool`)
  - Server: `service_server`
  - Client: `service_client`

## Runtime Behavior

1. `talker_node` publishes `{message_prefix} {counter}` on a timer.
2. `listener_node` logs each received message.
3. `service_client` calls `toggle_talker` with `true` or `false`.
4. `service_server` responds and publishes the resulting enabled state.
5. `talker_node` updates its internal enabled flag from `talker_enabled`.

## Parameters

- `talker_node.message_prefix` (string, default: `Hello ROS2`)
- `talker_node.publish_period_sec` (float, default: `1.0`)

## Launch

`service_pkg/launch/framework.launch.py` starts:

- `talker_node`
- `listener_node`
- `service_server`

with configurable talker parameters.
