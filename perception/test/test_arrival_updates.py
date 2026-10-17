import rclpy
from interfaces.msg import SagaState
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from perception.perception_node import PerceptionNode


def test_arrivals_wake_filter_only_during_running_event_and_pause_clears_queue():
    rclpy.init();node=PerceptionNode()
    try:
        boxes=Detection2DArray();scan=LaserScan()
        for _ in range(100):node.on_boxes(boxes);node.on_scan(scan)
        assert node.update_timer.is_canceled() and not node.queue.pending and not node.scans
        node.on_saga(SagaState(event_state='running'))
        node.on_boxes(boxes);node.on_scan(scan)
        assert not node.update_timer.is_canceled() and node.queue.pending and node.scans
        node.on_saga(SagaState(event_state='paused'))
        assert node.update_timer.is_canceled() and not node.queue.pending and not node.scans
        node.on_saga(SagaState(event_state='running'));node.on_boxes(boxes)
        node.refresh_map=lambda:False;node.sensor_update()
        assert node.update_timer.is_canceled() and not node.queue.pending and node.update_count==0
    finally:node.destroy_node();rclpy.shutdown()
