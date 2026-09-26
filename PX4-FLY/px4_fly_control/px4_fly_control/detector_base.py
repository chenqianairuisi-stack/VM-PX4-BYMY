"""One pending camera frame; inference never blocks camera ingestion."""

import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from px4_fly_interfaces.msg import VisualDetection


class LatestFrameDetector(Node):
    def __init__(self, name):
        super().__init__(
            name, parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.declare_parameter("camera_topic", "/tracker/front_camera/image_raw")
        self.declare_parameter("detection_topic", "/tracker/yolo/detection")
        self.declare_parameter("process_period", 0.02)
        self.bridge = CvBridge()
        self.pending = None
        self.pending_lock = threading.Lock()
        self.worker_group = MutuallyExclusiveCallbackGroup()
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            VisualDetection,
            str(self.get_parameter("detection_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.on_image,
            latest_qos,
        )
        self.create_timer(
            max(0.005, float(self.get_parameter("process_period").value)),
            self.process_latest,
            callback_group=self.worker_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def on_image(self, msg):
        with self.pending_lock:
            self.pending = msg

    def process_latest(self):
        with self.pending_lock:
            msg, self.pending = self.pending, None
        if msg is None:
            return
        output = VisualDetection()
        output.header = msg.header
        output.image_width, output.image_height = msg.width, msg.height
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result = self.detect(frame)
            if result is not None:
                cx, cy, width, height, confidence = result
                output.detected = True
                output.center_x, output.center_y = float(cx), float(cy)
                output.width, output.height = float(width), float(height)
                output.confidence = float(confidence)
                output.clipped = (
                    cx - width / 2 <= 1 / msg.width
                    or cy - height / 2 <= 1 / msg.height
                    or cx + width / 2 >= 1 - 1 / msg.width
                    or cy + height / 2 >= 1 - 1 / msg.height
                )
        except Exception as exc:
            self.get_logger().warning(
                f"Detection failed: {exc}", throttle_duration_sec=2.0
            )
        output.processed_stamp = self.get_clock().now().to_msg()
        self.pub.publish(output)


def run_detector(node_type, args=None):
    rclpy.init(args=args)
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
