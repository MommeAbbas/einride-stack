# Planning

Mission and exploration planning around standard Nav2. It sends mission dock
targets to `NavigateToPose`, evaluates dock occupancy, coordinates autonomous
frontier exploration, and selects camera viewpoints for dock-tag coverage.
Prediction-risk code remains available behind configuration switches.

- `config/planning.yaml`: mission, costmap, planner, and risk parameters.
- `config/explore.yaml`: `explore_lite` parameters.
- `config/coverage.yaml`: camera-coverage and next-view parameters.
- `launch/planning.launch.py`: package launch entry point.

```bash
ros2 launch planning planning.launch.py
colcon test --packages-select planning
```
