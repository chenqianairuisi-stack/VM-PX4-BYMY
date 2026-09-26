"""Deterministic geometry/delay/control regression tests (no simulator)."""

import math
import numpy as np
from px4_fly_control.visual_tracking import (
    CameraGeometry,
    Pose,
    PoseHistory,
    Px4Clock,
    TargetFilter,
    VelocitySmoother,
    rotation,
    tracking_command,
)


def pose(
    stamp=1.0, position=(0, 0, 0), q=(1, 0, 0, 0), velocity=(0, 0, 0), rates=(0, 0, 0)
):
    return Pose(
        stamp,
        np.array(position, float),
        np.array(velocity, float),
        np.array(q, float),
        np.array(rates, float),
    )


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def test_rotation_and_camera_conventions():
    camera = CameraGeometry()
    level = pose()
    np.testing.assert_allclose(
        camera.project(np.array([10, 0.0, -0.02]), level), [0.5, 0.5]
    )
    assert camera.project(np.array([10, 1, -0.02]), level)[0] > 0.5
    assert camera.project(np.array([10, 0, 1]), level)[1] > 0.5
    assert camera.project(np.array([-1, 0, 0]), level) is None
    np.testing.assert_allclose(
        rotation(quaternion(0, 0, math.pi / 2)) @ [1, 0, 0], [0, 1, 0], atol=1e-12
    )


def test_pose_history_interpolation_sign_and_extrapolation():
    history = PoseHistory()
    history.add(pose(1, position=(0, 0, 0), velocity=(2, 0, 0)))
    history.add(pose(1.1, position=(0.2, 0, 0), q=(-1, 0, 0, 0), velocity=(2, 0, 0)))
    np.testing.assert_allclose(history.at(1.05).position, [0.1, 0, 0])
    np.testing.assert_allclose(rotation(history.at(1.05).q), np.eye(3))
    np.testing.assert_allclose(history.at(1.15).position, [0.3, 0, 0])
    assert history.at(0.9) is None and history.at(1.4) is None
    assert not history.add(pose(1.09))


def test_pose_history_integrates_own_rotation():
    history = PoseHistory()
    history.add(pose(rates=(0, 0, 1.0)))
    expected = rotation(quaternion(0, 0, 0.05))
    np.testing.assert_allclose(rotation(history.at(1.05).q), expected, atol=1e-12)


def test_clock_recovers_gazebo_time_without_receipt_timestamp():
    bridge = Px4Clock()
    assert bridge.sample_time(100, 20, 30) is None
    offset = -1700000000000000
    bridge.set_offset(offset, 30)
    stamp_us = 20100000 - offset
    assert abs(bridge.sample_time(stamp_us, 20.12, 30.1) - 20.1) < 1e-9
    assert bridge.sample_time(stamp_us, 50.0, 30.1) is None
    assert bridge.sample_time(stamp_us, 20.12, 34) is None
    assert Px4Clock("sim").sample_time(20100000, 20.12, 30) == 20.099999999999998


def test_static_balloon_stays_static_through_camera_rotation():
    camera = CameraGeometry()
    point = np.array([8.0, 0.4, -0.3])
    reconstructed = []
    for angle in np.linspace(-0.3, 0.3, 31):
        own = pose(q=quaternion(angle / 2, angle, angle / 3))
        uv = camera.project(point, own)
        # Reconstruct direction independent of own rotation, with known range
        # (size/depth observation is tested separately).
        unit = camera.ray(*uv)
        unit /= np.linalg.norm(unit)
        origin = own.position + rotation(own.q) @ camera.offset
        reconstructed.append(
            origin
            + rotation(own.q)
            @ camera.body_from_camera
            @ unit
            * np.linalg.norm(point - origin)
        )
    np.testing.assert_allclose(reconstructed, np.tile(point, (31, 1)), atol=1e-10)


def test_known_sphere_size_recovers_range_and_camera_offset():
    camera = CameraGeometry()
    for distance in [1.0, 3.0, 10.0]:
        radius_px = camera.fx * math.tan(math.asin(0.22 / distance))
        box = [0.5, 0.5, 2 * radius_px / camera.width, 2 * radius_px / camera.height]
        point, cov = camera.observe(box, pose(), 0.44)
        np.testing.assert_allclose(point, [distance + 0.28, 0, -0.02], atol=1e-10)
        assert np.linalg.eigvalsh(cov).min() > 0


def test_delay_updates_historical_state_then_predicts_current_motion():
    tracker = TargetFilter(acceleration_noise=0.1)
    velocity = np.array([2.0, -0.4, 0.1])
    start = np.array([4.0, 0.0, 0.0])
    covariance = np.eye(3) * 0.0025
    for stamp in np.arange(0, 3.0, 0.05):
        assert tracker.update(float(stamp), start + velocity * stamp, covariance)
        tracker.predict(float(stamp + 0.12))  # Queries must not change filter time.
        assert tracker.stamp == stamp
    state, _ = tracker.predict(3.1)
    np.testing.assert_allclose(state[:3], start + velocity * 3.1, atol=0.03)
    np.testing.assert_allclose(state[3:], velocity, atol=0.03)
    assert not tracker.update(2.8, [99, 99, 99], covariance)
    assert not tracker.update(3.0, [99, 99, 99], covariance)


def test_missing_frames_grow_uncertainty_and_do_not_fake_observations():
    tracker = TargetFilter()
    tracker.update(1, np.array([5.0, 0, 0]), np.eye(3) * 0.01)
    stamp = tracker.stamp
    _, early = tracker.predict(1.1)
    _, late = tracker.predict(1.5)
    assert np.trace(late[:3, :3]) > np.trace(early[:3, :3])
    assert tracker.stamp == stamp and tracker.count == 1


def test_close_tracking_uses_target_velocity_feedforward():
    camera = CameraGeometry()
    target = np.array([1.5, 0, -0.02, 3.0, 0, 0])
    desired, _, _, visible = tracking_command(
        camera, pose(), target, np.eye(6) * 0.001, 1.2, 0.12
    )
    assert visible and desired[0] > 3.0  # No absolute 0.8 m/s cap.


def test_fov_edges_reduce_closure_and_acceleration():
    camera = CameraGeometry()
    P = np.eye(6) * 0.001
    center = np.array([5.0, 0.0, -0.02, 0, 0, 0])
    edge = np.array([5.0, 3.9, -0.02, 0, 0, 0])
    v0, _, a0, _ = tracking_command(camera, pose(), center, P, 3.0, 0.1)
    v1, _, a1, _ = tracking_command(camera, pose(), edge, P, 3.0, 0.1)
    assert np.linalg.norm(v1[:2]) < np.linalg.norm(v0[:2])
    assert a1 < a0
    v2, _, _, visible = tracking_command(
        camera, pose(), np.array([-5.0, 0, 0, 0, 0, 0]), P, 3.0, 0.1
    )
    assert not visible and np.linalg.norm(v2) < 0.1


def test_vector_acceleration_and_jerk_limits_survive_direction_reversal():
    smoother = VelocitySmoother(acceleration=3.0, jerk=6.0)
    smoother.reset([2.0, 1.0, 0.0])
    previous_velocity = smoother.velocity.copy()
    previous_acceleration = np.zeros(3)
    for i in range(600):
        desired = [4.0, -2.0, 1.0] if i < 200 else [-2.0, 2.0, 0.0]
        velocity = smoother.step(desired, 0.02)
        acceleration = (velocity - previous_velocity) / 0.02
        assert np.linalg.norm(acceleration) <= 3.0 + 1e-9
        assert np.linalg.norm(acceleration - previous_acceleration) / 0.02 <= 6.0 + 1e-8
        previous_velocity = velocity.copy()
        previous_acceleration = acceleration
    np.testing.assert_allclose(velocity, [-2.0, 2.0, 0.0], atol=0.03)


def test_falling_acceleration_cap_does_not_break_jerk_limit():
    smoother = VelocitySmoother()
    for _ in range(30):
        smoother.step([10.0, 0, 0], 0.02)
    a = smoother.acceleration.copy()
    smoother.step([0, 0, 0], 0.02, acceleration_limit=0.3)
    assert np.linalg.norm(smoother.acceleration - a) <= 0.12000001
