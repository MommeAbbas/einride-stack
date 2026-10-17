# Simulation and evaluation

Multi-robot Gazebo simulation and evaluation harness. It creates traffic robots,
publishes ground-truth poses for evaluation, emulates detections, records bags,
collects metrics, and launches the integrated stack and hardware profiles.

This package is an optional integration adapter. Its current world launch and
model resolver reference an external robot simulation package that is not
included here. Adapt `package.xml`, `launch/world.launch.py`, `sim/world.py`,
and `sim/truth_node.py` for another robot before building it.

Experiment outputs, bags, models, mission-server source, and credentials are not
part of this repository. A compatible external mission server can be supplied to
the experiment runner with `--mission-source` or `AUTONOMY_MISSION_SOURCE`.
