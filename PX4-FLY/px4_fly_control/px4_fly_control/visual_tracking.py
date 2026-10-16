"""ROS-independent estimation and constrained visual tracking primitives.

World = PX4 local NED; body = FRD; optical = right, down, forward.
The predictor adapts image-based visual-servoing concepts to this stack:
delayed measurements are time-aligned, transverse target motion is extrapolated
with bounded acceleration, and the command aims at a short
future image bearing instead of reacting only to the current oversized box.
It is still a receding velocity reference, not a full SO(3) controller.
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


@dataclass
class ExitObservation:
    """Last camera bearing, including a partially clipped final frame."""

    stamp: float
    center_x: float
    center_y: float
    width: float
    height: float
    bearing: float
    bearing_rate: float
    camera_right: np.ndarray
    clipped: bool


def edge_recovery_hint(observation, pose, target_position, max_distance=2.7):
    """Return a short body-side correction and yaw from the last image.

    This is only for a nearby target leaving a specific image edge. A centered,
    ambiguous or old observation does not authorize blind translation.
    """
    if observation is None or target_position is None:
        return None
    age = pose.stamp - observation.stamp
    if age < 0 or age > 0.55:
        return None
    origin = np.asarray(pose.position, float)
    if np.linalg.norm(np.asarray(target_position) - origin) > max_distance:
        return None
    side = observation.center_x - 0.5
    vertical = observation.center_y - 0.5
    horizontal_strength = float(np.clip((abs(side) - 0.18) / 0.27, 0, 1))
    vertical_strength = float(np.clip((abs(vertical) - 0.20) / 0.25, 0, 1))
    if max(horizontal_strength, vertical_strength) <= 0:
        return None
    # The final frame, not a possibly outdated inertial target velocity, sets
    # the side to cover. Keep the correction under one body length.
    right = np.asarray(observation.camera_right, float).copy()
    right[2] = 0
    right /= max(float(np.linalg.norm(right)), 1e-6)
    offset = right * math.copysign(0.25 + 0.40 * horizontal_strength, side) * horizontal_strength
    offset[2] = math.copysign(0.20 * vertical_strength, vertical)
    yaw = wrap(observation.bearing + float(np.clip(
        observation.bearing_rate * min(age, 0.25), -0.35, 0.35
    )))
    return offset, yaw


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
        offset=(0.21, 0, -0.02),
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

    def observe(self, box, pose, diameter, geometry="sphere"):
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
        if geometry == "drone":
            # The low, wide airframe is not a sphere. Use horizontal span only;
            # apparent span varies with yaw, so retain substantial depth noise.
            if width <= 0:
                raise ValueError("Degenerate drone width")
            distance = self.fx * diameter / (width * self.width) * np.linalg.norm(ray)
        elif geometry == "sphere":
            distance = 0.5 * diameter / math.sin(radius_angle)
        else:
            raise ValueError("Unknown target geometry")
        R = rotation(pose.q)
        direction = R @ self.body_from_camera @ unit
        point = pose.position + R @ self.offset + direction * distance
        # Size errors primarily affect depth; pixel errors affect transverse axes.
        sigma_depth = max(0.04, (0.25 if geometry == "drone" else 0.10) * distance)
        sigma_side = max(0.025, 2.0 * distance / min(self.fx, self.fy))
        covariance = np.eye(3) * sigma_side**2 + np.outer(direction, direction) * (
            sigma_depth**2 - sigma_side**2
        )
        return point, covariance

    def bearing(self, center, pose):
        """World camera origin and unit bearing; no size or range measurement."""
        ray = self.ray(*center)
        direction = rotation(pose.q) @ self.body_from_camera @ (ray / np.linalg.norm(ray))
        origin = pose.position + rotation(pose.q) @ self.offset
        return origin, direction

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
    """Six-state CV Kalman filter and optional bounded acceleration observer.

    Delayed observations update their own historical state. predict(now) then
    propagates a copy, so old pixels are never repeatedly used as new information.
    Out-of-order/duplicate observations are rejected explicitly.
    """

    def __init__(self, acceleration_noise=3.0, gate=16.3,
                 acceleration_limit=0.0, acceleration_decay=0.65):
        self.noise, self.gate = acceleration_noise, gate
        self.acceleration_limit = float(acceleration_limit)
        self.acceleration_decay = float(acceleration_decay)
        if not math.isfinite(self.acceleration_limit) or self.acceleration_limit < 0:
            raise ValueError("acceleration_limit must be finite and nonnegative")
        if not math.isfinite(self.acceleration_decay) or self.acceleration_decay <= 0:
            raise ValueError("acceleration_decay must be positive and finite")
        self.clear()

    def clear(self):
        self.x = self.P = self.stamp = None
        self.count = 0
        self.acceleration = np.zeros(3)

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
        # Depth comes from apparent airframe span and changes with pose. Keep
        # acceleration out of this depth prediction; guidance uses only its
        # bounded transverse component for short-horizon anticipation.
        return F @ self.x, F @ self.P @ F.T + Q

    def update_bearing(self, stamp, origin, direction):
        """Update only the two transverse constraints of an existing track.

        A partial box cannot initialise depth. The measurement matrix removes
        the ray direction, so repeated clipped frames add no direct depth
        information and cannot pretend their width measures distance.
        """
        if self.stamp is None or stamp <= self.stamp:
            return False
        origin, direction = np.asarray(origin, float), np.asarray(direction, float)
        if (not np.isfinite(origin).all() or not np.isfinite(direction).all()
                or np.linalg.norm(direction) < 1e-6):
            return False
        direction = direction / np.linalg.norm(direction)
        x, P = self.predict(stamp)
        transverse = np.eye(3) - np.outer(direction, direction)
        H = np.zeros((3, 6))
        H[:, :3] = transverse
        innovation = transverse @ (origin - x[:3])
        sigma = max(0.3, 0.18 * float(np.linalg.norm(x[:3] - origin)))
        noise = np.eye(3) * sigma**2
        S = H @ P @ H.T + noise
        if float(innovation @ np.linalg.solve(S, innovation)) > self.gate:
            return False
        K = np.linalg.solve(S, H @ P).T
        self.x = x + K @ innovation
        A = np.eye(6) - K @ H
        self.P = A @ P @ A.T + K @ noise @ K.T
        self.acceleration *= math.exp(-(stamp - self.stamp) / self.acceleration_decay)
        self.stamp = stamp
        self.count += 1
        return True

    def update(self, stamp, point, covariance):
        point = np.asarray(point, float)
        covariance = np.asarray(covariance, float)
        if not np.isfinite(point).all() or not np.isfinite(covariance).all():
            return False
        previous_stamp = self.stamp
        previous_velocity = None if self.x is None else self.x[3:].copy()
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
        if previous_velocity is not None and self.count >= 2 and self.acceleration_limit > 0:
            dt = stamp - previous_stamp
            if 0.04 <= dt <= 0.4:
                # A bounded, low-pass acceleration estimate improves short
                # horizon turns without allowing noisy box size to create a
                # long-horizon ballistic prediction.
                sample = limit_norm(
                    (self.x[3:] - previous_velocity) / dt,
                    self.acceleration_limit,
                )
                gain = min(0.3, dt / (0.35 + dt))
                self.acceleration += gain * (sample - self.acceleration)
            elif dt > 0.4:
                self.acceleration[:] = 0.0
        self.acceleration = limit_norm(self.acceleration, self.acceleration_limit)
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


class YawSmoother:
    """Shortest-path yaw reference with bounded rate and angular acceleration."""

    def __init__(self, rate_limit=0.45, acceleration_limit=0.7):
        if not all(math.isfinite(v) and v > 0 for v in (rate_limit, acceleration_limit)):
            raise ValueError("Yaw limits must be positive and finite")
        self.rate_limit = rate_limit
        self.acceleration_limit = acceleration_limit
        self.reset(0.0)

    def reset(self, yaw):
        self.yaw, self.rate = wrap(yaw), 0.0

    def step(self, target, dt):
        if dt <= 0:
            return self.yaw
        dt = min(dt, 0.1)
        error = wrap(target - self.yaw)
        # Proportional deceleration near the goal avoids a rate flip at arrival.
        requested = float(np.clip(2.0 * error, -self.rate_limit, self.rate_limit))
        previous = self.rate
        self.rate += float(np.clip(
            requested - self.rate,
            -self.acceleration_limit * dt,
            self.acceleration_limit * dt,
        ))
        self.yaw = wrap(self.yaw + 0.5 * (previous + self.rate) * dt)
        return self.yaw


def tracking_command(
    camera,
    pose,
    target_state,
    covariance,
    closure,
    horizon,
    stand_off=0.0,
    braking_acceleration=2.0,
    braking_latency=0.25,
    max_retreat_speed=1.0,
    max_speed=5.0,
    max_vertical=2.0,
    edge_margin=0.12,
    max_lookahead=0.8,
    prediction_weight=1.0,
    target_acceleration=None,
    prediction_yaw_weight=0.0,
    yaw_prediction_time=0.35,
    navigation_gain=0.0,
    turn_rate_budget=0.0,
    max_turn_standoff=12.0,
    image_box_width=None,
):
    """Relative-motion lookahead plus velocity feedforward in the NED frame.

    Lead relative motion across the line of sight over a bounded time-to-go;
    regulate radial closure by remaining distance without extrapolating past
    contact. The returned yaw is a blend toward the predicted future bearing,
    so a close target does not trigger a full-rate turn only after its current
    box has already filled the image. This is a receding velocity reference,
    not a full trajectory optimizer.
    """
    R = rotation(pose.q)
    origin = pose.position + R @ camera.offset
    relative = target_state[:3] - origin
    distance = float(np.linalg.norm(relative))
    current_yaw = math.atan2(relative[1], relative[0])
    uv = camera.project(target_state[:3], pose)
    visible = (
        uv is not None and np.all(uv > edge_margin) and np.all(uv < 1 - edge_margin)
    )
    if uv is None:
        visibility = 0.0
    else:
        visibility = float(np.clip(min(*uv, *(1 - uv)) / edge_margin, 0, 1))
        if image_box_width is not None:
            # A large box can touch the border while its centre is still near
            # the middle. Treat the nearest box edge as the available FOV.
            edge = min(float(uv[0]), 1.0 - float(uv[0])) - image_box_width / 2
            visibility = min(visibility, float(np.clip(edge / edge_margin, 0, 1)))
    sigma = math.sqrt(max(0, float(np.linalg.eigvalsh(covariance[:3, :3])[-1])))
    certainty = float(np.clip(1 - sigma / max(0.5, distance * 0.6), 0, 1))
    velocity_sigma = math.sqrt(max(0, float(np.linalg.eigvalsh(covariance[3:, 3:])[-1])))
    # Keep long extrapolations out of the command while velocity is unobserved.
    lead = min(max_lookahead, max(horizon, distance / max(closure, 0.3)))
    lead = min(lead, max(horizon, 0.3 / max(velocity_sigma, 1e-6)))
    weight = float(np.clip(prediction_weight, 0, 1))
    if target_acceleration is None:
        target_acceleration = np.zeros(3)
    target_acceleration = limit_norm(
        np.asarray(target_acceleration, float), max(0.1, max_speed * 1.2)
    )
    target_velocity_now = limit_norm(target_state[3:], max_speed * 0.85)
    origin_velocity = pose.velocity + R @ np.cross(pose.rates, camera.offset)
    # Only lead motion across the line of sight. Extrapolating radial closing
    # motion through the target flips the reference behind us at close range,
    # producing an unnecessary forward/reverse cycle. Radial speed is governed
    # separately by remaining distance below.
    line_of_sight = relative / max(distance, 1e-6)
    relative_velocity = target_velocity_now - origin_velocity
    transverse_velocity = relative_velocity - line_of_sight * float(
        relative_velocity @ line_of_sight
    )
    transverse_acceleration = target_acceleration - line_of_sight * float(
        target_acceleration @ line_of_sight
    )
    lead_offset = limit_norm(
        (transverse_velocity * lead +
         0.5 * transverse_acceleration * lead * lead) * weight,
        0.35 * distance,
    )
    predicted_relative = relative + lead_offset
    direction = predicted_relative / max(float(np.linalg.norm(predicted_relative)), 1e-6)
    # Slow closure when below/above the target or when heading still lags.
    attitude_yaw = math.atan2(R[1, 0], R[0, 0])
    yaw_time = min(max_lookahead, max(horizon, yaw_prediction_time)) * weight
    yaw_time *= float(np.clip(1 - velocity_sigma / 8.0, 0.25, 1.0))
    future_relative = relative + relative_velocity * yaw_time
    future_relative += 0.5 * transverse_acceleration * yaw_time * yaw_time
    future_yaw = math.atan2(future_relative[1], future_relative[0])
    yaw_lead = float(np.clip(wrap(future_yaw - current_yaw), -0.45, 0.45))
    yaw = wrap(current_yaw + float(np.clip(prediction_yaw_weight, 0.0, 1.0)) * yaw_lead)
    alignment = max(0.0, math.cos(wrap(yaw - attitude_yaw))) ** 4
    horizontal_distance = float(np.linalg.norm(relative[:2]))
    vertical_alignment = 1 / (1 + (abs(relative[2]) / max(0.3, horizontal_distance * 0.25)) ** 2)
    # A positive stand-off also commands retreat if the target comes too close.
    # Preserve the original contact experiment when stand_off is zero.
    # Leave room for the follower to stop if a moving target brakes suddenly.
    # Relative speed alone is zero when both vehicles match speed, and would
    # discard that stopping distance just before the target slows down.
    relative_closing_speed = max(
        0.0,
        float(origin_velocity @ line_of_sight),
        -float(relative_velocity @ line_of_sight),
    )
    if stand_off > 0 and turn_rate_budget > 0:
        # Required line-of-sight rate is v_perp / range. Open space before a
        # close lateral crossing demands more yaw than the camera can follow.
        horizontal_range = max(0.2, float(np.linalg.norm(relative[:2])))
        transverse_speed = abs(
            relative[0] * relative_velocity[1] -
            relative[1] * relative_velocity[0]
        ) / horizontal_range
        stand_off = max(
            stand_off,
            min(max_turn_standoff, transverse_speed / turn_rate_budget),
        )
    braking_distance = (
        relative_closing_speed**2 / (2 * max(braking_acceleration, 1e-6))
        + relative_closing_speed * braking_latency
    ) if stand_off > 0 else 0.0
    radial = (float(np.clip(distance - stand_off - braking_distance,
                            -max_retreat_speed, closure))
              if stand_off > 0 else min(closure, max(0.0, distance)))
    closing = (
        radial
        * (visibility if radial >= 0 else 1.0)
        * certainty
        * (alignment if radial >= 0 else 1.0)
        * (vertical_alignment if radial >= 0 else 1.0)
    )
    # Forward feedforward also fades at the image edge; otherwise an uncertain
    # target velocity can keep driving forward even after closure is suppressed.
    forward = np.array([math.cos(attitude_yaw), math.sin(attitude_yaw), 0.0])
    target_velocity = limit_norm(
        target_velocity_now + transverse_acceleration * lead,
        max_speed * 0.85,
    ) * weight
    feedforward = target_velocity.copy()
    advancing = max(0.0, float(feedforward @ forward))
    feedforward -= forward * advancing * (1 - visibility * alignment)
    # A bounded proportional-navigation normal velocity correction reduces
    # the line-of-sight angular rate during positive closure. At stand-off or
    # during retreat, do not apply interception guidance; keep lateral target
    # velocity matching and retreat directly away along the current LOS.
    if navigation_gain > 0 and closing > 0:
        los_rate = np.cross(relative, relative_velocity) / max(distance**2, 0.04)
        normal_acceleration = navigation_gain * closing * np.cross(
            los_rate, line_of_sight
        )
        navigation_velocity = limit_norm(normal_acceleration * lead, 0.35 * max_speed)
        desired = feedforward + line_of_sight * closing + navigation_velocity * weight
    else:
        desired = feedforward + (line_of_sight if radial < 0 else direction) * closing
    # Vertical centering remains available even while forward closure is gated.
    desired[2] = target_velocity[2] + weight * float(np.clip(predicted_relative[2] * 0.8, -2, 2))
    if stand_off > 0 and horizontal_distance > 0.1:
        # Constrain the TOTAL radial command, including velocity feedforward.
        # Otherwise a stale target velocity can overpower the retreat term
        # after the target stops. Solve v*latency + v^2/(2*a) <= free range.
        gap = max(0.0, horizontal_distance - stand_off)
        a = max(braking_acceleration, 1e-6)
        reserve = a * braking_latency
        approach_limit = math.sqrt(reserve**2 + 2 * a * gap) - reserve
        horizontal_los = relative[:2] / horizontal_distance
        approaching = float(desired[:2] @ horizontal_los)
        desired[:2] -= horizontal_los * max(0.0, approaching - approach_limit)
    desired[:2] = limit_norm(desired[:2], max_speed)
    desired[2] = np.clip(desired[2], -max_vertical, max_vertical)
    # Reduce acceleration near a vertical edge so pitch does not remove target.
    acceleration_scale = 1.0 if radial < 0 else 0.2 + 0.8 * visibility
    return desired, yaw, acceleration_scale, visible
