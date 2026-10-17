#!/usr/bin/env python3
"""Exercise the real ROS detector callback and output topic on the three supplied images."""
import argparse
import json
from pathlib import Path
import time
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from perception.detector_node import DetectorNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images'); parser.add_argument('output')
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'backend:=onnx', '-p', 'use_sim_time:=false'])
    detector = DetectorNode()
    probe = rclpy.create_node('detector_images_check')
    received = []
    probe.create_subscription(Detection2DArray, '/sim/detections', received.append, 10)
    pub = probe.create_publisher(Image, '/oak/rgb/image_raw', qos_profile_sensor_data)
    executor = SingleThreadedExecutor(); executor.add_node(detector); executor.add_node(probe)
    rows = []
    try:
        deadline = time.monotonic() + 10
        while (pub.get_subscription_count() == 0 or detector.publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert pub.get_subscription_count() and detector.publisher.get_subscription_count(), 'ROS discovery failed'
        for path in sorted(Path(args.images).glob('*.jpg')):
            image = cv2.imread(str(path)); assert image is not None
            msg = Image(height=image.shape[0], width=image.shape[1], encoding='bgr8', step=image.shape[1] * 3, data=image.tobytes())
            msg.header.frame_id = 'oak_rgb_camera_optical_frame'
            msg.header.stamp = probe.get_clock().now().to_msg()
            received.clear(); start = time.monotonic(); pub.publish(msg)
            while not received and time.monotonic() - start < 10:
                executor.spin_once(timeout_sec=.1)
            assert received, f'No detector output for {path}'
            result = received[0]
            assert result.header == msg.header, 'Capture header changed'
            rows.append({'image': path.name, 'width': msg.width, 'height': msg.height,
                         'elapsed_wall_ms': (time.monotonic() - start) * 1000,
                         'boxes': [{'cx': d.bbox.center.position.x, 'cy': d.bbox.center.position.y,
                                    'width': d.bbox.size_x, 'height': d.bbox.size_y,
                                    'confidence': d.results[0].hypothesis.score} for d in result.detections]})
            time.sleep(.11)  # Check harness only: respect the configured 10 Hz input limit.
        assert len(rows) == 3, 'Expected the three supplied test images'
        Path(args.output).write_text(json.dumps(rows, indent=2) + '\n')
        print(json.dumps(rows, indent=2))
    finally:
        executor.shutdown(); detector.destroy_node(); probe.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
