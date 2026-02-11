# Changelog

## [0.1.0] - 2026-02-11

### Added
- Launch-based bringup via `service_pkg/launch/framework.launch.py`
- Talker parameters: `message_prefix`, `publish_period_sec`
- Service-controlled enable/disable flow over `toggle_talker` + `talker_enabled`
- Service behavior unit tests (`test_service_logic.py`)
- CI workflow for ROS2 build/test on GitHub Actions
- Portfolio docs: architecture and contribution guidelines

### Changed
- Replaced placeholder package metadata and aligned licenses to MIT
- Updated lint test configuration for reliable execution in constrained environments
