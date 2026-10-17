"""Real RGB detector with the emulator's capture-stamped Detection2DArray contract."""
from pathlib import Path
import time
import json
from std_msgs.msg import String
from rclpy.clock import Clock,ClockType
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from .detector import load_backend, letterbox, boxes_from_output


class DetectorNode(Node):
    def __init__(self):
        super().__init__('perception_detector')
        config = Path(get_package_share_directory('perception')) / 'config/detector.yaml'
        for key, value in yaml.safe_load(config.read_text())['/**']['ros__parameters'].items():
            if key != 'use_sim_time':
                self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.threshold, self.iou, self.rate = p('confidence_threshold'), p('nms_iou_threshold'), p('rate_limit')
        if not 0 < self.threshold <= 1 or not 0 < self.iou <= 1 or self.rate <= 0 or p('onnx_threads') < 1:
            raise ValueError('Invalid detector threshold, rate or thread count')
        self.backend, active_backend, fallback = load_backend(p('backend'), p('engine_path'), p('onnx_path'), p('onnx_threads'))
        self.error_log_period = p('error_log_period')
        self.active_backend=active_backend;self.inference_count=0;self.runtime_start=time.monotonic()
        self.runtime_pub=self.create_publisher(String,'/perception/detector_runtime',10)
        self.runtime_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(p('runtime_report_period'),self.report_runtime,clock=self.runtime_clock)
        self.publisher = self.create_publisher(Detection2DArray, p('output_topic'), p('qos_depth'))
        self.subscription = self.create_subscription(Image, p('image_topic'), self.on_image,
            rclpy.qos.QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability))
        self.last = -float('inf')
        self.get_logger().info(f"Detector ready: active_backend={active_backend}, {p('image_topic')} -> {p('output_topic')}; fallback_reason={fallback}")

    def report_runtime(self):
        now=time.monotonic();elapsed=max(now-self.runtime_start,1e-9)
        self.runtime_pub.publish(String(data=json.dumps(dict(backend=self.active_backend,inference_hz=self.inference_count/elapsed,window_seconds=elapsed))))
        self.runtime_start=now;self.inference_count=0

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last < 1 / self.rate:
            return
        self.last = now
        try:
            if message.encoding not in ('rgb8', 'bgr8'):
                raise ValueError(f'Unsupported RGB encoding: {message.encoding}')
            rgb = np.ndarray((message.height, message.width, 3), np.uint8,
                buffer=bytes(message.data), strides=(message.step, 3, 1))
            if message.encoding == 'bgr8':
                rgb = rgb[:, :, ::-1]
            tensor, geometry = letterbox(rgb)
            boxes = boxes_from_output(self.backend(tensor), geometry, self.threshold, self.iou)
        except Exception as error:
            # A failure is missing evidence, never a successful saw-nothing frame.
            self.get_logger().error(f'Detector frame failed: {error}', throttle_duration_sec=self.error_log_period)
            return
        result = Detection2DArray(header=message.header)
        for x, y, w, h, score in boxes:
            detection = Detection2D(header=message.header)
            detection.bbox.center.position.x = x + w / 2
            detection.bbox.center.position.y = y + h / 2
            detection.bbox.size_x, detection.bbox.size_y = w, h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'robot'
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)
            result.detections.append(detection)
        self.publisher.publish(result)
        self.inference_count+=1

    def destroy_node(self):
        self.backend.close()
        return super().destroy_node()


def main():
    from .ros_support import run
    run(DetectorNode)


if __name__ == '__main__':
    main()
