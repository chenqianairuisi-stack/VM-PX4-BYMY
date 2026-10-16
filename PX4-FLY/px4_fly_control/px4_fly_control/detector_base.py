"""One pending camera frame; inference never blocks camera ingestion."""

import threading
import cv2

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
        self.annotated_pub = self.create_publisher(
            Image, "/tracker/yolo/annotated", qos_profile_sensor_data
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

    def detect_stamped(self, frame, stamp):
        return self.detect(frame)

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
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            result = self.detect_stamped(frame, stamp)
            if result is not None:
                cx, cy, width, height, confidence = result
                output.detected = True
                output.center_x, output.center_y = float(cx), float(cy)
                output.width, output.height = float(width), float(height)
                output.confidence = float(confidence)
                output.clipped = bool(
                    cx - width / 2 <= 1 / msg.width
                    or cy - height / 2 <= 1 / msg.height
                    or cx + width / 2 >= 1 - 1 / msg.width
                    or cy + height / 2 >= 1 - 1 / msg.height
                )
            if self.annotated_pub.get_subscription_count():
                annotated = frame.copy()
                if output.detected:
                    x1 = round((output.center_x - output.width / 2) * msg.width)
                    y1 = round((output.center_y - output.height / 2) * msg.height)
                    x2 = round((output.center_x + output.width / 2) * msg.width)
                    y2 = round((output.center_y + output.height / 2) * msg.height)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, f"{output.confidence:.2f}", (x1, max(15, y1-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                debug = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug.header = msg.header
                self.annotated_pub.publish(debug)
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
