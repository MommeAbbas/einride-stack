# Interfaces

Shared ROS 2 messages used by the autonomy stack. The package defines perception
predictions and health, dock maps and occupancy, planned trajectories,
coordinator requests and status, mission state, and simulation truth/events.

Message definitions live in `msg/`. `test/test_contracts.py` checks constants,
defaults, and the public field contract.

```bash
colcon build --packages-select interfaces
colcon test --packages-select interfaces
```
