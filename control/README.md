# Control

Nav2 control coordination and safety adapters. The package connects mission
goals to Nav2, handles docking actions, compensates hardware velocity deadbands,
processes depth points, and provides collision-safe recovery and escape logic.
`nav2_collision_monitor` remains the final authority over physical velocity.

- `config/control.yaml`: Nav2 controller, behavior, collision monitor, and BT setup.
- `config/docking.yaml`: docking server configuration.
- `config/escape.yaml`: escape-state and swept-footprint parameters.
- `config/hardware_motion.yaml`: measured hardware motion limits.

```bash
ros2 launch control control.launch.py
colcon test --packages-select control
```
