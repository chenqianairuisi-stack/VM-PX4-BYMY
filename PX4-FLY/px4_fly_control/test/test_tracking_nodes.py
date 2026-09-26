"""ROS-node boundary tests; no PX4 process or arming commands required."""

import math
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import rclpy
from px4_fly_interfaces.msg import VisualDetection
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import CameraInfo
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from px4_fly_control.safe_visual_follow import SafeVisualFollow
from px4_fly_control.visual_tracking import Pose
from px4_fly_control.red_color_detector import RedColorDetector
from px4_fly_control.keyboard_control import KeyboardControl


@pytest.fixture
def controller():
    rclpy.init()
    node = SafeVisualFollow()
    node.sim_now = lambda: 10.2
    info = CameraInfo()
    info.width = 640
    info.height = 480
    info.k = [381.36, 0.0, 320.0, 0.0, 381.36, 240.0, 0.0, 0.0, 1.0]
    info.header.frame_id = "camera"
    node.on_camera_info(info)
    for stamp in [10.0, 10.1, 10.2]:
        node.history.add(
            Pose(stamp, np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3))
        )
    yield node
    node.destroy_node()
    rclpy.shutdown()


def detection(stamp=10.1, valid=True):
    m = VisualDetection()
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    m.header.frame_id = "camera"
    m.image_width = 640
    m.image_height = 480
    m.detected = valid
    m.center_x = 0.5
    m.center_y = 0.5
    m.width = 0.05
    m.height = 0.05 * 640 / 480
    m.confidence = 0.9
    return m


def test_explicit_miss_does_not_refresh_filter(controller):
    n = controller
    n.on_detection(detection())
    n.process_detection(10.2)
    assert n.accepted_frames == 1
    n.on_detection(detection(10.2, False))
    n.process_detection(10.2)
    assert not n.last_frame_detected and n.estimator.stamp == pytest.approx(10.1)
    assert n.track_available(10.3) is not None
    assert n.track_available(10.7) is None


def test_reject_duplicate_old_clipped_and_wrong_frame(controller):
    n = controller
    n.on_detection(detection())
    n.process_detection(10.2)
    n.on_detection(detection())
    n.process_detection(10.2)
    assert n.accepted_frames == 1
    m = detection(10.2)
    m.clipped = True
    n.on_detection(m)
    n.process_detection(10.2)
    assert n.accepted_frames == 1
    m = detection(10.3)
    m.header.frame_id = "other"
    n.on_detection(m)
    n.process_detection(10.3)
    assert n.accepted_frames == 1
    n.on_detection(detection(11.0))
    n.process_detection(12.0)
    assert n.accepted_frames == 1


def test_burst_requires_external_confirmation_not_distance(controller):
    n = controller
    n.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED)
    n.on_burst(Bool(data=True))
    assert not n.burst_confirmed
    n.state = "FINAL"
    n.on_burst(Bool(data=False))
    assert not n.burst_confirmed
    n.on_burst(Bool(data=True))
    assert n.burst_confirmed


def test_headstart_uses_airborne_stamp_and_false_resets(controller):
    n = controller
    n.on_airborne(Time(sec=8, nanosec=0))
    assert n.target_takeoff_time == 8.0
    n.on_target_event(Bool(data=False))
    assert n.target_takeoff_time is None
    n.on_airborne(Time(sec=20))
    assert n.target_takeoff_time is None


def test_direction_yaw_uses_camera_heading_offset(controller):
    n = controller
    n.position = SimpleNamespace(heading=-math.pi / 2)
    pose = n.history.at(10.1)
    pose.q = np.array([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])
    # In this model the camera looks world +E while PX4 reports heading -pi/2.
    assert n.setpoint_yaw_for_direction(math.pi / 2, pose) == pytest.approx(-math.pi / 2)
    # A target to the world +N must map to the opposite PX4 yaw convention.
    assert abs(abs(n.setpoint_yaw_for_direction(0.0, pose)) - math.pi) < 1e-6


def test_hsv_sphere_and_no_detection():
    import cv2

    rclpy.init()
    node = RedColorDetector()
    try:
        frame = np.zeros((480, 640, 3), np.uint8)
        assert node.detect(frame) is None
        cv2.circle(frame, (320, 240), 35, (0, 0, 255), -1)
        box = node.detect(frame)
        assert abs(box[0] - 0.5) < 0.01 and abs(box[1] - 0.5) < 0.01
        assert box[4] > 0.55
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_airborne_not_emitted_on_arm_only():
    rclpy.init()
    node = KeyboardControl()
    try:
        node.ready = lambda: True
        node.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED)
        node.state = "flying"
        node.takeoff_ground_z = 0.0
        node.position = SimpleNamespace(z=0.0)
        node.check_airborne()
        assert node.airborne_since is None
        node.position.z = -0.6

        class TestClock:
            value = 1.0

            def now(self):
                return SimpleNamespace(nanoseconds=int(self.value * 1e9))

        clock = TestClock()
        with patch.object(node, "get_clock", return_value=clock):
            node.check_airborne()
            assert not node.airborne_announced
            clock.value = 1.4
            node.check_airborne()
            assert node.airborne_announced
    finally:
        node.destroy_node()
        rclpy.shutdown()
