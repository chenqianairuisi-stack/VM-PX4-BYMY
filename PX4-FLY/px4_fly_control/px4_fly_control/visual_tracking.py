"""ROS-independent estimation and constrained visual tracking primitives.

World = PX4 local NED; body = FRD; optical = right, down, forward.
This is an engineering adaptation (CV target KF + historical own odometry),
not the papers' SO(3) controller or full inertial delayed EKF.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def limit_norm(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    return vector * min(1.0, maximum / max(float(np.linalg.norm(vector)), 1e-12))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-6:
        raise ValueError("Invalid attitude quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def slerp(q0, q1, weight):
    a, b = np.asarray(q0, float), np.asarray(q1, float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        value = a + weight * (b - a)
        return value / np.linalg.norm(value)
    angle = math.acos(min(1.0, dot))
    return (
        math.sin((1 - weight) * angle) * a + math.sin(weight * angle) * b
    ) / math.sin(angle)


@dataclass
class Pose:
    stamp: float
    position: np.ndarray
    velocity: np.ndarray
    q: np.ndarray
    rates: np.ndarray


class PoseHistory:
    def __init__(self, duration=3.0, extrapolation=0.08):
        self.samples = deque()
        self.duration, self.extrapolation = duration, extrapolation

    def clear(self):
        self.samples.clear()

    def add(self, pose):
        if self.samples and pose.stamp <= self.samples[-1].stamp:
            return False
        self.samples.append(pose)
        while self.samples and pose.stamp - self.samples[0].stamp > self.duration:
            self.samples.popleft()
        return True

    def at(self, stamp):
        if not self.samples or stamp < self.samples[0].stamp - 1e-6:
            return None
        for a, b in zip(self.samples, list(self.samples)[1:]):
            if a.stamp <= stamp <= b.stamp:
                if b.stamp - a.stamp > 0.15:
                    return None  # Do not interpolate across a telemetry outage.
                w = (stamp - a.stamp) / (b.stamp - a.stamp)
                return Pose(
                    stamp,
                    a.position + w * (b.position - a.position),
                    a.velocity + w * (b.velocity - a.velocity),
                    slerp(a.q, b.q, w),
                    a.rates + w * (b.rates - a.rates),
                )
        last = self.samples[-1]
        dt = stamp - last.stamp
        if dt < -1e-6 or dt > self.extrapolation:
            return None
        rate = np.linalg.norm(last.rates)
        if rate * dt > 0.2:
            return None
        # Right-multiply the attitude by a body-rate increment.
        if rate > 1e-8:
            dq = np.r_[
                math.cos(rate * dt / 2), last.rates / rate * math.sin(rate * dt / 2)
            ]
            w, x, y, z = last.q
            a, b, c, d = dq
            q = np.array(
                [
                    w * a - x * b - y * c - z * d,
                    w * b + x * a + y * d - z * c,
                    w * c - x * d + y * a + z * b,
                    w * d + x * c - y * b + z * a,
                ]
            )
        else:
            q = last.q.copy()
        return Pose(
            stamp,
            last.position + last.velocity * dt,
            last.velocity.copy(),
            q,
            last.rates.copy(),
        )


class Px4Clock:
    """Undo PX4 1.14 DDS timestamp offset for lockstep Gazebo simulation.

    The 1.14 serializer adds -TimesyncStatus.estimated_offset to HRT samples.
    Classic SITL HRT is driven by Gazebo simulation time. Reject mismatched domains
    instead of inventing a reception-time alignment. Raw mode is explicit only.
    """

    def __init__(self, mode="dds"):
        if mode not in ("dds", "sim"):
            raise ValueError("px4_timestamp_mode must be dds or sim")
        self.mode = mode
        self.offset_us = None
        self.received_wall = None

    def set_offset(self, offset_us, wall):
        self.offset_us, self.received_wall = int(offset_us), wall

    def sample_time(self, sample_us, now_sim, now_wall):
        if self.mode == "dds":
            if self.offset_us is None or now_wall - self.received_wall > 3:
                return None
            sample_us = int(sample_us) + self.offset_us
        stamp = sample_us * 1e-6
        if stamp <= 0 or stamp > now_sim + 0.04 or now_sim - stamp > 0.3:
            return None
        return stamp


class CameraGeometry:
    def __init__(
        self,
        width=640,
        height=480,
        hfov=math.radians(80),
        offset=(0.28, 0, -0.02),
        pitch=0.0,
    ):
        self.width, self.height = width, height
        self.fx = self.fy = 0.5 * width / math.tan(hfov / 2)
        self.cx, self.cy = width / 2, height / 2
        self.offset = np.array(offset, float)
        # Optical (right,down,forward) -> FRD, then camera mounting pitch.
        c, s = math.cos(pitch), math.sin(pitch)
        self.body_from_camera = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ np.array(
            [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        )

    def set_intrinsics(self, width, height, k):
        if (
            width <= 0
            or height <= 0
            or k[0] <= 0
            or k[4] <= 0
            or not np.isfinite(k).all()
        ):
            raise ValueError("Invalid CameraInfo intrinsics")
        self.width, self.height = width, height
        self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

    def ray(self, cx, cy):
        return np.array(
            [
                (cx * self.width - self.cx) / self.fx,
                (cy * self.height - self.cy) / self.fy,
                1.0,
            ]
        )

    def observe(self, box, pose, diameter):
        cx, cy, width, height = box
        ray = self.ray(cx, cy)
        unit = ray / np.linalg.norm(ray)
        # Angular radius of a known sphere, rather than a planar depth guess.
        left = self.ray(cx - width / 2, cy)
        right = self.ray(cx + width / 2, cy)
        up = self.ray(cx, cy - height / 2)
        down = self.ray(cx, cy + height / 2)
        angles = [
            math.acos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))
            / 2
            for a, b in ((left, right), (up, down))
        ]
        radius_angle = min(angles)
        if radius_angle <= 1e-5:
            raise ValueError("Degenerate bounding box")
        distance = 0.5 * diameter / math.sin(radius_angle)
        R = rotation(pose.q)
        direction = R @ self.body_from_camera @ unit
        point = pose.position + R @ self.offset + direction * distance
        # Size errors primarily affect depth; pixel errors affect transverse axes.
        sigma_depth = max(0.04, 0.10 * distance)
        sigma_side = max(0.025, 2.0 * distance / min(self.fx, self.fy))
        covariance = np.eye(3) * sigma_side**2 + np.outer(direction, direction) * (
            sigma_depth**2 - sigma_side**2
        )
        return point, covariance

    def project(self, point, pose):
        optical = self.body_from_camera.T @ (
            rotation(pose.q).T @ (point - pose.position) - self.offset
        )
        if optical[2] <= 0.02:
            return None
        return np.array(
            [
                (self.fx * optical[0] / optical[2] + self.cx) / self.width,
                (self.fy * optical[1] / optical[2] + self.cy) / self.height,
            ]
        )


class TargetFilter:
    """CV Kalman filter kept at last exposure time, never at query time.

    Delayed observations update their own historical state. predict(now) then
    propagates a copy, so old pixels are never repeatedly used as new information.
    Out-of-order/duplicate observations are rejected explicitly.
    """

    def __init__(self, acceleration_noise=3.0, gate=16.3):
        self.noise, self.gate = acceleration_noise, gate
        self.clear()

    def clear(self):
        self.x = self.P = self.stamp = None
        self.count = 0

    def predict(self, stamp):
        if self.stamp is None or stamp < self.stamp - 1e-6:
            return None
        dt = max(0.0, stamp - self.stamp)
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        # Continuous white acceleration spectral density.
        Q = (
            np.block(
                [
                    [np.eye(3) * dt**3 / 3, np.eye(3) * dt**2 / 2],
                    [np.eye(3) * dt**2 / 2, np.eye(3) * dt],
                ]
            )
            * self.noise**2
        )
        return F @ self.x, F @ self.P @ F.T + Q

    def update(self, stamp, point, covariance):
        point = np.asarray(point, float)
        covariance = np.asarray(covariance, float)
        if not np.isfinite(point).all() or not np.isfinite(covariance).all():
            return False
        if self.stamp is None:
            self.x = np.r_[point, np.zeros(3)]
            self.P = np.zeros((6, 6))
            self.P[:3, :3] = covariance
            self.P[3:, 3:] = np.eye(3) * 9
        else:
            if stamp <= self.stamp:
                return False
            x, P = self.predict(stamp)
            innovation = point - x[:3]
            S = P[:3, :3] + covariance
            if float(innovation @ np.linalg.solve(S, innovation)) > self.gate:
                return False
            K = np.linalg.solve(S, P[:3, :]).T
            self.x = x + K @ innovation
            A = np.eye(6)
            A[:, :3] -= K
            self.P = A @ P @ A.T + K @ covariance @ K.T  # Joseph form.
        self.stamp = stamp
        self.count += 1
        return True


class VelocitySmoother:
    """Vector acceleration and jerk limits, initialized from measured velocity."""

    def __init__(self, acceleration=3.0, jerk=6.0):
        self.max_acceleration, self.max_jerk = acceleration, jerk
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

    def reset(self, velocity):
        self.velocity = np.asarray(velocity, float).copy()
        self.acceleration = np.zeros(3)

    def step(self, desired, dt, acceleration_limit=None):
        if dt <= 0:
            return self.velocity.copy()
        dt = min(dt, 0.1)
        cap = (
            self.max_acceleration
            if acceleration_limit is None
            else min(self.max_acceleration, acceleration_limit)
        )
        # Braking envelope avoids a bang-bang velocity servo near the setpoint.
        error = np.asarray(desired) - self.velocity
        requested = limit_norm(error / max(0.25, dt), cap)
        self.acceleration += limit_norm(
            requested - self.acceleration, self.max_jerk * dt
        )
        # Do not abruptly clip the existing acceleration when the FOV cap falls:
        # jerk limiting brings it down over subsequent cycles.
        self.velocity += self.acceleration * dt
        return self.velocity.copy()


def tracking_command(
    camera,
    pose,
    target_state,
    covariance,
    closure,
    horizon,
    stand_off=0.0,
    max_speed=5.0,
    max_vertical=2.0,
    edge_margin=0.12,
):
    """Relative-velocity feedforward with short-horizon/FOV-aware correction.

    This produces a velocity reference for existing PX4 inner loops, not a formal
    FOV-invariant controller. Covariance and angular errors suppress closure.
    """
    future = target_state[:3] + target_state[3:] * horizon
    # Align the front contact point, accounting for body translation at close range.
    relative = future - (pose.position + rotation(pose.q) @ camera.offset)
    distance = float(np.linalg.norm(relative))
    yaw = math.atan2(relative[1], relative[0])
    heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    along = float(relative @ heading)
    transverse = relative - heading * along
    uv = camera.project(future, pose)
    visible = (
        uv is not None and np.all(uv > edge_margin) and np.all(uv < 1 - edge_margin)
    )
    if uv is None:
        visibility = 0.0
    else:
        visibility = float(np.clip(min(*uv, *(1 - uv)) / edge_margin, 0, 1))
    sigma = math.sqrt(max(0, float(np.linalg.eigvalsh(covariance[:3, :3])[-1])))
    certainty = float(np.clip(1 - sigma / max(0.5, distance * 0.6), 0, 1))
    # Slow closure when below/above the target or when heading still lags.
    attitude_yaw = math.atan2(rotation(pose.q)[1, 0], rotation(pose.q)[0, 0])
    alignment = max(0.0, math.cos(wrap(yaw - attitude_yaw))) ** 4
    vertical_alignment = 1 / (1 + (abs(transverse[2]) / max(0.3, along * 0.25)) ** 2)
    closing = (
        min(closure, max(0.15, along - stand_off))
        * visibility
        * certainty
        * alignment
        * vertical_alignment
    )
    feedforward = limit_norm(target_state[3:], max_speed * 0.85)
    desired = feedforward + heading * closing + limit_norm(transverse * 0.8, 2.0)
    desired[:2] = limit_norm(desired[:2], max_speed)
    desired[2] = np.clip(desired[2], -max_vertical, max_vertical)
    # Reduce acceleration near a vertical edge so pitch does not remove target.
    acceleration_scale = 0.2 + 0.8 * visibility
    return desired, yaw, acceleration_scale, visible
