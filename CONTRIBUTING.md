# Contributing

## Development Setup

1. Use Ubuntu 24.04 with ROS 2 Jazzy installed.
2. Build from workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src
source install/setup.bash
```

## Coding Standards

- Python code should pass `ament_flake8` and `ament_pep257` checks.
- Keep node behavior deterministic and observable through logs.
- Update `README.md` and `docs/architecture.md` when changing communication contracts.

## Testing

Run before opening a PR:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --base-paths src
colcon test-result --verbose
```

## Pull Request Checklist

- [ ] Build succeeds with `colcon build --base-paths src`
- [ ] Tests pass with `colcon test --base-paths src`
- [ ] New behavior is documented
- [ ] `CHANGELOG.md` updated for user-visible changes
