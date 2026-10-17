# Docking control plugin

C++ Open Navigation docking plugin for tag-relative, non-charging docks. The
`control::TagDock` plugin selects live tag observations and applies lateral,
heading, and dock-axis acceptance limits before reporting success.

The plugin declaration is in `docks.xml`; its implementation is
`src/tag_dock.cpp`.

```bash
colcon build --packages-select control_docking
colcon test --packages-select control_docking
```
