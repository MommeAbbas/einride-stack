# Perception

Robot detection, distance estimation, tracking, trajectory prediction, and dock
occupancy inference. Detector backends support TensorRT and ONNX Runtime while
publishing one common ROS message contract. RGB and depth encodings are converted
explicitly for simulation and hardware inputs.

- `config/detector.yaml`: model backend, paths, confidence, NMS, and rate limits.
- `config/perception.yaml`: ranging, association, filtering, and prediction parameters.
- `launch/perception.launch.py`: package launch entry point.

Model binaries are runtime inputs and are not included.

```bash
ros2 launch perception perception.launch.py
colcon test --packages-select perception
```
