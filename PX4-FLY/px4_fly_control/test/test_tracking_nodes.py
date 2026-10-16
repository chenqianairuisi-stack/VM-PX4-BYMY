"""ROS-node boundary tests; no PX4 process or arming commands required."""

import math
import time
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
from px4_fly_control.visual_tracking import Pose, ExitObservation, wrap
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
    assert n.track_available(10.7) is not None
    assert n.track_available(11.0) is None


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


def test_direction_yaw_uses_ned_without_calibrating_async_heading(controller):
    n = controller
    n.position = SimpleNamespace(heading=-math.pi / 2)
    pose = n.history.at(10.1)
    pose.q = np.array([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])
    # Deliberately stale local_position heading must not rotate a NED direction.
    # EKF2 publishes heading = Euler(odometry.q).psi(), not its negative.
    assert n.setpoint_yaw_for_direction(math.pi / 2, pose) == pytest.approx(math.pi / 2)
    assert n.setpoint_yaw_for_direction(0.0, pose) == pytest.approx(0.0)


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


def test_search_to_tracking_preserves_velocity_and_yaw_reference(controller):
    n = controller
    n.position = SimpleNamespace(x=0.0, y=0.0, z=0.0, heading=0.0,
                                 vx=0.0, vy=0.0, vz=0.0)
    n.ready = lambda: True
    n.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED,
                              nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    n.home = (0.0, 0.0, 2.5)
    n.state = "SEARCH"
    n.last_tick = 10.18
    n.smoother.reset([0.5, 0.0, 0.0])
    n.smoother.acceleration = np.array([0.3, 0.0, 0.0])
    n.yaw_smoother.reset(0.1)
    n.yaw = 0.1
    assert n.estimator.update(10.2, [4, 0, -0.02], np.eye(3) * 0.01)
    n.last_valid_wall = time.monotonic()
    n.last_frame_detected = True
    n.track_locked = True
    before_velocity = n.smoother.velocity.copy()
    before_acceleration = n.smoother.acceleration.copy()
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
    assert n.state == "APPROACH"
    assert np.linalg.norm(n.smoother.velocity - before_velocity) <= n.cfg["command_acceleration"] * 0.02
    assert np.linalg.norm(n.smoother.acceleration - before_acceleration) <= n.cfg["command_jerk"] * 0.02 + 1e-9
    assert abs(n.yaw - 0.1) <= n.cfg["yaw_rate_limit"] * 0.02
    msg = publish.call_args.args[0]
    assert all(math.isnan(v) for v in msg.position)
    assert math.isfinite(msg.yawspeed)


def test_initial_search_holds_position_instead_of_flying_waypoints(controller):
    n = controller
    n.position = SimpleNamespace(x=0.0, y=0.0, z=0.0, heading=0.0,
                                 vx=0.0, vy=0.0, vz=0.0)
    n.ready = lambda: True
    n.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED,
                              nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    n.home = (0.0, 0.0, 2.5)
    n.state = "SEARCH"
    n.last_tick = 10.18
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
    msg = publish.call_args.args[0]
    np.testing.assert_allclose(msg.position, np.zeros(3))
    assert all(math.isnan(v) for v in msg.velocity)
    np.testing.assert_allclose(n.search_position, np.zeros(3))


def test_keyboard_yaw_keys_publish_a_ramp_and_freeze_with_sim_clock():
    rclpy.init()
    n = KeyboardControl()
    try:
        n.ready = lambda: True
        n.position = SimpleNamespace(x=0.0, y=0.0, z=-3.0, heading=0.0)
        n.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED,
                                  nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        n.state = "flying"
        n.capture()
        for _ in range(9):
            n.handle_key('l')
        assert n.yaw == pytest.approx(math.pi / 2)
        n.last_tick_sim = 10.0
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_050_000_000))
        with patch.object(n, "get_clock", return_value=clock), patch.object(n.setpoint_pub, "publish") as publish:
            n.tick()
            first = publish.call_args.args[0]
            assert 0 < first.yaw < 0.45 * 0.05
            assert 0 < first.yawspeed <= 0.7 * 0.05 + 1e-9
            n.tick()
            assert publish.call_args.args[0].yaw == first.yaw
        for _ in range(50):
            n.handle_key('w')
        assert math.hypot(*n.target[:2]) <= n.get_parameter('manual_lead_limit').value + 1e-9
    finally:
        n.destroy_node()
        rclpy.shutdown()


def prepare_flying(n):
    n.position = SimpleNamespace(x=0.0, y=0.0, z=0.0, heading=0.0,
                                 vx=0.0, vy=0.0, vz=0.0)
    n.ready = lambda: True
    n.status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED,
                              nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    n.home = (0.0, 0.0, 2.5)
    n.state = "APPROACH"
    n.last_tick = 10.18


def test_short_miss_keeps_velocity_mode_then_recovery_discards_stale_velocity(controller):
    n = controller
    prepare_flying(n)
    n.cfg["lock_frames"] = 1
    n.on_detection(detection(10.1))
    n.process_detection(10.2)
    n.smoother.reset([5.0, 0.0, 0.0])
    n.on_detection(detection(10.2, False))
    n.history.at = lambda _: n.history.samples[-1]
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
        assert n.state == "APPROACH"  # One frame miss does not stop pursuit.
        n.sim_now = lambda: 10.42
        n.tick()
        assert n.state == "APPROACH"
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)
        n.sim_now = lambda: 11.0
        n.tick()
        assert n.state == "REACQUIRE" and not n.track_locked
        np.testing.assert_allclose(publish.call_args.args[0].position, np.zeros(3))
        n.on_detection(detection(11.01))
        n.sim_now = lambda: 11.02
        n.tick()
        assert n.state == "APPROACH"
        assert np.linalg.norm(publish.call_args.args[0].velocity) < 0.1
        assert n.reacquire_position is None


def test_loss_brakes_in_velocity_mode_then_latches_stopped_position(controller):
    n = controller
    prepare_flying(n)
    own = n.history.samples[-1]
    own.velocity[0] = 2.0
    n.history.at = lambda _: own
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
        assert n.state == "BRAKING"
        assert n.reacquire_position is None
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)
        assert publish.call_args.args[0].velocity[0] > 1.9
        own.position[0] += 0.3
        own.velocity[0] = 1.0
        n.sim_now = lambda: 10.4
        n.tick()
        assert n.state == "BRAKING"
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)
        own.position[0] += 0.2
        own.velocity[:] = 0
        n.sim_now = lambda: 11.0
        n.tick()
        anchor = n.reacquire_position.copy()
        np.testing.assert_allclose(anchor, [0.5, 0, 0])
        np.testing.assert_allclose(publish.call_args.args[0].position, anchor)
        assert n.state == "REACQUIRE"
        assert n.reacquire_scan_started == 11.0


def test_scan_completes_full_circle_across_wrap_and_keeps_holding(controller):
    from px4_fly_control.visual_tracking import wrap
    n = controller
    prepare_flying(n)
    own = n.history.samples[-1]
    start_yaw = math.radians(175)
    own.q = np.array([math.cos(start_yaw/2), 0, 0, math.sin(start_yaw/2)])
    n.history.at = lambda _: own
    traveled = 0.0
    previous = start_yaw
    with patch.object(n.setpoint_pub, "publish") as publish:
        for i in range(1800):
            n.sim_now = lambda i=i: 10.2 + i * 0.02
            n.tick()
            msg = publish.call_args.args[0]
            np.testing.assert_allclose(msg.position, np.zeros(3))
            traveled += wrap(msg.yaw - previous)
            previous = msg.yaw
            # Ideal attitude tracking isolates the scan logic from PX4 dynamics.
            own.q = np.array([math.cos(msg.yaw/2), 0, 0, math.sin(msg.yaw/2)])
            assert n.state == "REACQUIRE"
        assert n.reacquire_progress == pytest.approx(2 * math.pi)
        assert traveled == pytest.approx(2 * math.pi, abs=1e-5)
        assert abs(msg.yawspeed) < 1e-5
        assert abs(wrap(msg.yaw - start_yaw)) < 1e-5


def test_tracker_camera_extrinsics_match_model_and_mount_contacts_mesh(controller):
    from pathlib import Path
    import struct
    import xml.etree.ElementTree as ET
    root = Path(__file__).resolve().parents[2]
    model = ET.parse(root / "models/tracker_iris/tracker_iris.sdf").getroot().find("model")
    link = model.find("link[@name='front_camera_link']")
    offset = np.fromstring(link.findtext("pose"), sep=" ")[:3]
    sensor = np.fromstring(link.find("sensor").findtext("pose"), sep=" ")[:3]
    np.testing.assert_allclose(controller.camera.offset, (offset + sensor) * [1, -1, -1])
    joint = model.find("joint[@name='front_camera_fixed_joint']")
    assert joint.attrib["type"] == "fixed" and joint.findtext("parent") == "base_link"
    mount = link.find("visual[@name='front_camera_mount_visual']")
    center = offset + np.fromstring(mount.findtext("pose"), sep=" ")[:3]
    half = np.fromstring(mount.findtext("geometry/box/size"), sep=" ") / 2
    data = (root / "models/iris/meshes/iris.stl").read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    vertices = np.array([struct.unpack_from('<9f', data, 84 + i*50 + 12)
                         for i in range(count)]).reshape(-1, 3)
    assert np.any(np.all(abs(vertices - center) < half, axis=1))
    housing_half = np.fromstring(link.findtext("visual[@name='front_camera_visual']/geometry/box/size"), sep=" ") / 2
    assert np.all(abs(center - offset) < half + housing_half)


def test_tracker_accelerates_faster_without_losing_jerk_limit(controller):
    from px4_fly_control.visual_tracking import VelocitySmoother
    n = controller
    previous = VelocitySmoother(acceleration=1.5, jerk=3.0)
    last_acceleration = n.smoother.acceleration.copy()
    for _ in range(50):
        command = n.smoother.step([4.5, 0, 0], 0.02)
        baseline = previous.step([4.5, 0, 0], 0.02)
        assert np.linalg.norm(n.smoother.acceleration) <= n.cfg["command_acceleration"] + 1e-9
        assert np.linalg.norm(n.smoother.acceleration - last_acceleration) <= n.cfg["command_jerk"] * 0.02 + 1e-9
        last_acceleration = n.smoother.acceleration.copy()
    assert command[0] >= 2.0
    assert command[0] > baseline[0] * 1.8


def test_initial_search_rotates_multiple_turns_at_one_fixed_position(controller):
    n = controller
    prepare_flying(n)
    n.state = "SEARCH"
    own = n.history.samples[-1]
    n.history.at = lambda _: own
    turn = 0.0
    previous = 0.0
    with patch.object(n.setpoint_pub, "publish") as publish:
        for i in range(1250):
            n.sim_now = lambda i=i: 10.2 + 0.02 * i
            n.tick()
            msg = publish.call_args.args[0]
            np.testing.assert_allclose(msg.position, [0, 0, 0])
            assert all(math.isnan(v) for v in msg.velocity)
            turn += wrap(msg.yaw - previous)
            previous = msg.yaw
            own.q[:] = [math.cos(msg.yaw/2), 0, 0, math.sin(msg.yaw/2)]
        assert turn > 2 * math.pi
        assert n.state == "SEARCH"


def test_initial_scan_waits_for_settling_and_turns_slower_than_reacquisition(controller):
    n = controller
    own = Pose(10.0, np.zeros(3), np.array([0.18, 0, 0]),
               np.array([1.0, 0, 0, 0]), np.zeros(3))
    n.begin_search(own)
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.hold_and_scan(own, 10.0, 0.02, searching=True)
        assert n.search_scan_started is None
        own.velocity[:] = 0
        n.hold_and_scan(own, 10.1, 0.02, searching=True)
        n.hold_and_scan(own, 10.5, 0.02, searching=True)
        assert n.search_scan_started is None
        n.hold_and_scan(own, 10.62, 0.02, searching=True)
        assert n.search_scan_started == pytest.approx(10.62)
        assert n.search_progress == pytest.approx(n.cfg["search_yaw_rate"] * 0.02)
        np.testing.assert_allclose(publish.call_args.args[0].position, np.zeros(3))
        n.begin_reacquire(own, 11.0)
        n.hold_and_scan(own, 11.0, 0.02)
        assert n.reacquire_progress == pytest.approx(n.cfg["reacquire_yaw_rate"] * 0.02)
        assert n.reacquire_progress > n.search_progress


def test_initial_search_starts_slow_closure_on_first_detection(controller):
    n = controller
    prepare_flying(n)
    n.state = "SEARCH"
    for stamp in (10.25, 10.3):
        n.history.add(Pose(stamp, np.zeros(3), np.zeros(3),
                           np.array([1.0, 0, 0, 0]), np.zeros(3)))
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.on_detection(detection(10.1))
        n.tick()
        assert n.state == "LOCKING" and n.detection_streak == 1
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)
        assert publish.call_args.args[0].velocity[0] > 0
        n.on_detection(detection(10.2, False))
        n.sim_now = lambda: 10.25
        n.tick()
        assert n.state == "LOCKING" and n.detection_streak == 1
        n.on_detection(detection(10.25))
        n.sim_now = lambda: 10.3
        n.tick()
        assert n.state == "APPROACH" and n.track_locked
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)


def test_far_visible_balloon_can_start_approach_despite_depth_error(controller):
    n = controller
    prepare_flying(n)
    n.state = "SEARCH"
    far = detection(10.1)
    far.width = 0.016
    far.height = far.width * 640 / 480
    n.on_detection(far)
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
        assert n.state == "LOCKING"
        assert n.track_available(10.2) is not None
        assert np.linalg.norm(n.estimator.x[:3]) > 13.0
        assert publish.call_args.args[0].velocity[0] > 0


def test_five_fps_with_a_dropped_frame_does_not_restart_acceleration(controller):
    n = controller
    prepare_flying(n)
    n.state = "SEARCH"
    n.history.at = lambda stamp: Pose(
        stamp, np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3)
    )
    previous_acceleration = n.smoother.acceleration.copy()
    with patch.object(n.setpoint_pub, "publish") as publish:
        for i in range(100):
            now = 10.2 + i * 0.02
            n.sim_now = lambda now=now: now
            if i % 10 == 0:
                n.on_detection(detection(now, valid=i != 40))
            n.tick()
            assert n.state in ("LOCKING", "APPROACH")
            assert all(math.isnan(v) for v in publish.call_args.args[0].position)
            assert np.linalg.norm(n.smoother.acceleration - previous_acceleration) <= n.cfg["command_jerk"] * 0.02 + 1e-8
            previous_acceleration = n.smoother.acceleration.copy()
        assert n.track_locked
        assert publish.call_args.args[0].velocity[0] > 2.5


def test_stale_observation_caps_closure_without_a_jerk_discontinuity(controller):
    n = controller
    prepare_flying(n)
    n.last_tick = 10.28
    n.sim_now = lambda: 10.3
    n.history.at = lambda stamp: Pose(
        stamp, np.zeros(3), np.array([1.5, 0, 0]),
        np.array([1.0, 0, 0, 0]), np.zeros(3)
    )
    assert n.estimator.update(10.0, [7.0, 0, -0.02], np.eye(3) * 0.04)
    n.last_valid_wall = time.monotonic()
    n.track_locked = True
    n.smoother.reset([1.5, 0, 0])
    n.smoother.acceleration[0] = 2.0
    original_send = n.send_velocity
    desired = []

    def capture(command, dt, scale=1.0):
        desired.append(np.array(command, float))
        original_send(command, dt, scale)

    n.send_velocity = capture
    with patch.object(n.setpoint_pub, "publish"):
        n.tick()
    assert n.state == "APPROACH"
    assert 0 < np.linalg.norm(desired[0][:2]) <= 1.5 + 1e-9
    assert np.linalg.norm(n.smoother.acceleration - [2, 0, 0]) <= n.cfg["command_jerk"] * 0.02 + 1e-8


def test_partial_nearby_edge_exit_does_not_wait_for_prediction_timeout(controller):
    n = controller
    prepare_flying(n)
    first = detection(10.1)
    first.width = 0.2
    first.height = first.width * 640 / 480
    n.on_detection(first)
    n.process_detection(10.2)
    last = detection(10.2)
    last.center_x = 0.93
    last.clipped = True
    n.on_detection(last)
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
        assert n.state == "EDGE_RECOVERY"
        assert publish.call_args.args[0].position[1] > 0.3


@pytest.mark.parametrize("side", (-1, 1))
def test_nearby_exit_edge_commands_short_side_step_then_fixed_hold(controller, side):
    n = controller
    prepare_flying(n)
    own = n.history.samples[-1]
    n.last_target = np.array([1.0, 0, 0])
    n.last_seen = ExitObservation(
        10.1, .5 + .4 * side, .5, .1, .1, .55 * side, 0,
        np.array([0., 1., 0.]), True,
    )
    with patch.object(n.setpoint_pub, "publish") as publish:
        n.tick()
        assert n.state == "EDGE_RECOVERY"
        first = publish.call_args.args[0]
        assert side * first.position[1] > .3
        assert np.linalg.norm(first.position) <= .65 + 1e-9
        target = first.position.copy()
        assert side * n.recovery_yaw > 0
        n.sim_now = lambda: 11.0
        n.history.at = lambda _: own
        n.tick()
        assert n.state == "REACQUIRE"
        np.testing.assert_allclose(publish.call_args.args[0].position, target)
        n.estimator.update(11.01, [1, .4 * side, 0], np.eye(3) * .01)
        n.last_valid_wall = time.monotonic()
        n.sim_now = lambda: 11.02
        n.tick()
        assert n.state == "LOCKING"
        assert all(math.isnan(v) for v in publish.call_args.args[0].position)


def test_clipped_last_frame_retains_exit_bearing_without_using_bad_range(controller):
    n = controller
    n.on_detection(detection(10.1))
    n.process_detection(10.2)
    accepted = n.estimator.count
    last = detection(10.2)
    last.clipped = True
    last.center_x = .92
    last.width = .12
    n.on_detection(last)
    n.process_detection(10.2)
    assert n.estimator.count == accepted
    assert n.last_seen.center_x == pytest.approx(.92)
    assert n.last_seen.clipped


def test_target_balloon_is_visual_only_and_cannot_strike_ground():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[2] / "models/fpv_target/fpv_target.sdf"
    model = ET.parse(path).getroot().find("model")
    body = model.find("link[@name='base_link']")
    assert body.find("visual[@name='tracking_tether_visual']") is not None
    assert body.find("visual[@name='tracking_balloon_visual']") is not None
    assert model.find("link[@name='tracking_tether']") is None
    assert model.find("link[@name='tracking_balloon']") is None
    assert not any("balloon" in c.attrib["name"] or "tether" in c.attrib["name"]
                   for c in model.iter("collision"))


def test_drone_mode_ignores_balloon_contact(controller):
    n = controller
    prepare_flying(n)
    n.cfg['target_geometry'] = 'drone'
    n.cfg['stand_off'] = 4.0
    message = Bool()
    message.data = True
    n.on_burst(message)
    assert not n.burst_confirmed


def test_virtual_contact_accepts_event_while_active_in_virtual_mode(controller):
    n = controller
    prepare_flying(n)
    n.cfg['target_geometry'] = 'drone'
    n.cfg['stand_off'] = 3.0
    event = Bool(data=True)
    n.on_virtual_contact(event)
    assert not n.virtual_contact_confirmed
    n.cfg['virtual_contact_mode'] = True
    n.on_virtual_contact(event)
    assert n.virtual_contact_confirmed  # Gazebo confirms distance even without pixels.
    n.virtual_contact_confirmed = False
    n.track_locked = True
    n.estimator.update(9.0, [4, 0, 0], np.eye(3)*.1)
    n.on_virtual_contact(event)
    assert n.virtual_contact_confirmed
    n.virtual_contact_confirmed = False
    n.estimator.clear()
    n.estimator.update(10.1, [4, 0, 0], np.eye(3)*.1)
    n.on_virtual_contact(Bool(data=False))
    assert not n.virtual_contact_confirmed
    n.on_virtual_contact(event)
    assert n.virtual_contact_confirmed
    with patch.object(n, 'request_land') as land:
        n.tick()
    land.assert_called_once_with('simulator confirmed virtual drone contact')


def test_drone_mode_passes_standoff_to_guidance(controller):
    n = controller
    prepare_flying(n)
    n.cfg['target_geometry'] = 'drone'
    n.cfg['stand_off'] = 4.0
    assert n.estimator.update(10.2, [4.21,0,-.02], np.eye(3)*.001)
    n.last_valid_wall = time.monotonic()
    n.track_locked = True
    with patch.object(n.setpoint_pub, 'publish') as publish:
        n.tick()
    np.testing.assert_allclose(publish.call_args.args[0].velocity, [0,0,0], atol=1e-8)


def prepare_near_contact(n):
    prepare_flying(n)
    n.cfg.update(target_geometry='drone', virtual_contact_mode=True,
                 stand_off=0.0, virtual_contact_threshold=.5,
                 near_prediction_distance=1.6)
    n.estimator.update(10.2, [1.4, 0, 0], np.eye(3)*.001)
    n.track_locked = n.last_frame_detected = True
    n.last_valid_wall = time.monotonic()
    n.last_seen = ExitObservation(10.2, .5, .5, .2, .2, 0., 0.,
                                  np.array([0., 1., 0.]), False)
    n.history.at = lambda _: n.history.samples[-1]


@pytest.mark.parametrize('distance', [.3, .5, 1.0, 2.0])
def test_virtual_contact_has_no_standoff_retreat_or_braking_radius(controller, distance):
    n = controller
    prepare_near_contact(n)
    n.track_locked = False  # Exercise visual approach before prediction entry.
    own = n.history.samples[-1]
    own.velocity[0] = 2.0  # Positive stand_off would reserve braking distance.
    n.estimator.x[:3] = [distance + n.camera.offset[0], 0, n.camera.offset[2]]
    with patch.object(n, 'send_velocity') as send:
        n.tick()
    assert send.call_args.args[0][0] > 0


def test_near_contact_freezes_visual_track_and_accepts_event_without_fresh_pixels(controller):
    n = controller
    prepare_near_contact(n)
    with patch.object(n.setpoint_pub, 'publish'):
        n.tick()
        assert n.state == 'PREDICT_CONTACT'
        snapshot = n.near_target.copy()
        n.sim_now = lambda: 11.0
        n.on_detection(detection(11.0))
        n.tick()
    assert n.pending_detection is None
    assert n.estimator.stamp == pytest.approx(10.2)
    np.testing.assert_array_equal(n.near_target, snapshot)
    n.on_virtual_contact(Bool(data=True))
    assert n.virtual_contact_confirmed
    with patch.object(n, 'request_land') as land:
        n.tick()
    land.assert_called_once_with('simulator confirmed virtual drone contact')


def test_near_contact_timeout_brakes_then_repeats_scan_with_vision_restored(controller):
    n = controller
    prepare_near_contact(n)
    own = n.history.samples[-1]
    with patch.object(n.setpoint_pub, 'publish') as publish:
        n.tick()
        own.velocity[0] = 1.0
        n.sim_now = lambda: 11.71
        n.tick()
        assert n.state == 'BRAKING'
        assert n.estimator.stamp is None and not n.track_locked
        n.on_detection(detection(11.71))
        assert n.pending_detection is None
        own.velocity[:] = 0
        n.sim_now = lambda: 11.73
        n.tick()
        assert n.state == 'REACQUIRE' and not n.near_ignore_vision
        anchor = n.reacquire_position.copy()
        traveled, previous = 0.0, 0.0
        for i in range(1, 900):
            n.sim_now = lambda i=i: 11.73 + i*.02
            n.tick()
            msg = publish.call_args.args[0]
            np.testing.assert_allclose(msg.position, anchor)
            traveled += wrap(msg.yaw-previous)
            previous = msg.yaw
            own.q = np.array([math.cos(msg.yaw/2), 0., 0., math.sin(msg.yaw/2)])
        assert traveled > 2*math.pi
        assert n.state == 'REACQUIRE'
        n.on_detection(detection(n.sim_now()))
        assert n.pending_detection is not None
    n.on_virtual_contact(Bool(data=True))
    assert n.virtual_contact_confirmed  # A real near pass during search also lands.


@pytest.mark.parametrize('invalid', ['clipped', 'stale', 'uncertain', 'unlocked'])
def test_near_prediction_requires_reliable_entry_track(controller, invalid):
    n = controller
    prepare_near_contact(n)
    if invalid == 'clipped':
        n.last_seen = SimpleNamespace(clipped=True)
    elif invalid == 'stale':
        n.estimator.stamp = 9.0
    elif invalid == 'uncertain':
        n.estimator.P[:3, :3] = np.eye(3)*4.0
    else:
        n.track_locked = False
    assert not n.begin_near_prediction(10.2, n.history.samples[-1],
                                      (n.estimator.x, n.estimator.P))


def test_near_prediction_advances_frozen_target_velocity_and_limits_command(controller):
    n = controller
    prepare_near_contact(n)
    n.estimator.x[4] = 1.0
    assert n.begin_near_prediction(10.2, n.history.samples[-1],
                                   (n.estimator.x, n.estimator.P))
    with patch.object(n, 'send_velocity') as send:
        n.follow_near_prediction(n.history.samples[-1], 10.8, .02)
    desired = send.call_args.args[0]
    assert desired[0] > 0 and desired[1] > 1.0
    assert np.linalg.norm(desired[:2]) <= n.cfg['near_prediction_speed'] + 1e-9


def test_near_prediction_keeps_closing_inside_half_metre_until_event(controller):
    n = controller
    prepare_near_contact(n)
    n.estimator.x[:3] = [.3, 0, 0]
    assert n.begin_near_prediction(10.2, n.history.samples[-1],
                                   (n.estimator.x, n.estimator.P))
    with patch.object(n, 'send_velocity') as send:
        n.follow_near_prediction(n.history.samples[-1], 10.3, .02)
    assert send.call_args.args[0][0] >= .8


@pytest.mark.parametrize('filename', ['drone_follow.yaml', 'drone_virtual_contact.yaml'])
def test_checked_in_drone_configs_start_controller(filename):
    from pathlib import Path
    path = Path(__file__).resolve().parents[2]/'config'/filename
    rclpy.init(args=['--ros-args', '--params-file', str(path)])
    node = None
    try:
        node = SafeVisualFollow()
        assert node.cfg['target_geometry'] == 'drone'
        if node.cfg['virtual_contact_mode']:
            assert node.cfg['virtual_contact_threshold'] == .5
            assert node.cfg['stand_off'] == 0.0
            assert node.cfg['near_prediction_distance'] > .5
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_detector_accepts_numpy_box_and_publishes_clipped_flag():
    from px4_fly_control.detector_base import LatestFrameDetector
    rclpy.init()
    node = LatestFrameDetector('numpy_box_test')
    try:
        node.detect = lambda frame: tuple(np.float64(v) for v in [.95,.5,.1,.1,.9])
        node.on_image(node.bridge.cv2_to_imgmsg(np.zeros((360,480,3),np.uint8),'bgr8'))
        with patch.object(node.pub,'publish') as publish:
            node.process_latest()
        message = publish.call_args.args[0]
        assert message.detected and message.clipped
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_drone_large_clipped_box_updates_bearing_without_triggering_edge_recovery(controller):
    n = controller
    prepare_flying(n)
    n.cfg['target_geometry'] = 'drone'
    n.cfg['stand_off'] = 6.0
    n.track_locked = True
    n.estimator.update(10.1, [1.21, .25, -.02], np.eye(3)*.1)
    m = detection(10.2)
    m.width, m.height, m.center_x, m.clipped = .6, .2, .72, True
    n.on_detection(m)
    with patch.object(n.setpoint_pub, 'publish') as publish:
        n.tick()
    assert n.estimator.stamp == pytest.approx(10.2)
    assert n.last_seen.clipped and n.last_frame_detected
    assert n.state == 'FINAL'
    assert all(math.isnan(v) for v in publish.call_args.args[0].position)


def test_clipped_box_cannot_initialise_a_drone_track(controller):
    n = controller
    n.cfg['target_geometry'] = 'drone'
    m = detection(10.2)
    m.width, m.clipped = .8, True
    n.on_detection(m)
    n.process_detection(10.2)
    assert n.estimator.stamp is None
    assert not n.track_locked


def test_drone_partial_edge_frame_keeps_a_recent_associated_track(controller):
    n = controller
    n.cfg['target_geometry'] = 'drone'
    n.estimator.update(10.1, [4, -2.8, -.02], np.eye(3)*.1)
    m = detection(10.2)
    m.center_x, m.width, m.clipped = .06, .12, True
    n.on_detection(m)
    n.process_detection(10.2)
    assert n.estimator.stamp == pytest.approx(10.2)
    assert n.bearing_frames == 1
    assert n.last_frame_detected


def test_steady_delayed_frames_are_predicted_instead_of_treated_as_dropout(controller):
    n = controller
    prepare_flying(n)
    n.state = 'SEARCH'
    n.history.at = lambda stamp: Pose(
        stamp, np.zeros(3), np.zeros(3), np.array([1., 0, 0, 0]), np.zeros(3)
    )
    with patch.object(n.setpoint_pub, 'publish'):
        for i in range(100):
            now = 10.5 + i * .02
            n.sim_now = lambda now=now: now
            if i % 10 == 0:
                n.on_detection(detection(now - .28))
            n.tick()
        assert n.track_locked
        assert n.last_valid_delay == pytest.approx(.28)
        assert n.smoother.velocity[0] > 2.0
        n.sim_now = lambda: now + .9
        n.tick()
        assert n.state in ('BRAKING', 'REACQUIRE')


def test_low_score_detection_requires_locked_track_and_prediction_association(controller):
    n = controller
    n.cfg['tracking_confidence_threshold'] = .35
    low = detection(10.0)
    low.confidence = .4
    n.on_detection(low)
    n.process_detection(10.0)
    assert n.estimator.stamp is None
    n.on_detection(detection(10.1))
    n.process_detection(10.1)
    n.track_locked = True
    low = detection(10.2)
    low.confidence = .4
    n.on_detection(low)
    n.process_detection(10.2)
    assert n.estimator.stamp == pytest.approx(10.2)
    n.history.at = lambda stamp: Pose(
        stamp, np.zeros(3), np.zeros(3), np.array([1., 0, 0, 0]), np.zeros(3)
    )
    unrelated = detection(10.3)
    unrelated.confidence, unrelated.center_x = .4, .85
    n.on_detection(unrelated)
    n.process_detection(10.3)
    assert n.estimator.stamp == pytest.approx(10.2)
