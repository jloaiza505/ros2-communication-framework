# Contributing

## Development Setup

1. Use Ubuntu 22.04 with ROS2 Humble installed.
2. Build from workspace root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Coding Standards

- Python code should pass `ament_flake8` and `ament_pep257` checks.
- Keep node behavior deterministic and observable through logs.
- Update `README.md` and `docs/architecture.md` when changing communication contracts.

## Testing

Run before opening a PR:

```bash
cd ros2_ws
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Pull Request Checklist

- [ ] Build succeeds with `colcon build --symlink-install`
- [ ] Tests pass with `colcon test`
- [ ] New behavior is documented
- [ ] `CHANGELOG.md` updated for user-visible changes
