#!/usr/bin/env python3
"""Predictive visual tracking for the dual-UAV balloon simulation.

Only own PX4 telemetry and timestamped image observations enter tracking.
A separate simulator referee reports contact; no target truth enters guidance.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Time
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
    TimesyncStatus,
)
from px4_fly_interfaces.msg import VisualDetection
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from .visual_tracking import (
    CameraGeometry,
    Pose,
    PoseHistory,
    Px4Clock,
    TargetFilter,
    VelocitySmoother,
    tracking_command,
    rotation,
    wrap,
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SafeVisualFollow(Node):
    ACTIVE = (
        "STARTING",
        "TAKEOFF",
        "SEARCH",
        "LOCKING",
        "APPROACH",
        "FINAL",
        "PREDICT",
        "REACQUIRE",
    )

    def __init__(self):
        super().__init__(
            "safe_visual_follow",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        defaults = {
            "vehicle_namespace": "/px4_1",
            "target_system": 2,
            "camera_info_topic": "/tracker/front_camera/camera_info",
            "detection_topic": "/tracker/yolo/detection",
            "px4_timestamp_mode": "dds",
            "takeoff_altitude": 2.5,
            "target_hint_delay": 10.0,
            "search_north": 4.0,
            "search_east": 0.0,
            "search_size": 6.0,
            "search_altitude": 2.5,
            "search_latitude": math.nan,
            "search_longitude": math.nan,
            "search_radius": 6.0,
            "target_hint_north": math.nan,
            "target_hint_east": math.nan,
            "lock_frames": 4,
            "lock_speed": 0.7,
            "approach_speed": 3.0,
            "final_speed": 1.2,
            "final_distance": 1.5,
            "max_follow_speed": 5.0,
            "max_vertical_speed": 0.8,
            "control_rate": 50.0,
            "command_acceleration": 3.0,
            "command_jerk": 6.0,
            "yaw_rate_limit": 0.8,
            "max_image_age": 0.35,
            "fresh_timeout": 0.3,
            "prediction_timeout": 0.55,
            "prediction_sigma_limit": 1.0,
            "control_lookahead": 0.12,
            "reacquire_time": 6.0,
            "confidence_threshold": 0.55,
            "target_width_m": 0.44,
            "target_acceleration_noise": 3.0,
            "camera_forward": 0.28,
            "camera_right": 0.0,
            "camera_down": -0.02,
            "camera_pitch_deg": 0.0,
            "minimum_height": 0.35,
            "maximum_height": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in (
            "control_rate",
            "command_acceleration",
            "command_jerk",
            "max_follow_speed",
            "max_vertical_speed",
            "max_image_age",
            "prediction_timeout",
            "target_width_m",
        ):
            if not math.isfinite(float(self.cfg[name])) or self.cfg[name] <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not self.get_parameter("use_sim_time").value:
            raise ValueError(
                "This SITL controller requires use_sim_time:=true and Gazebo /clock"
            )
        self.vehicle_ns = "/" + str(self.cfg["vehicle_namespace"]).strip("/")
        self.target_system = int(self.cfg["target_system"])
        topic = lambda suffix: self.vehicle_ns + suffix
        self.mode_pub = self.create_publisher(
            OffboardControlMode, topic("/fmu/in/offboard_control_mode"), 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, topic("/fmu/in/trajectory_setpoint"), 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, topic("/fmu/in/vehicle_command"), 10
        )
        self.diagnostic_pub = self.create_publisher(
            String, "/tracker/tracking_status", 10
        )
        event_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Time, "/target/airborne_stamp", self.on_airborne, event_qos
        )
        self.create_subscription(
            Bool, "/target/takeoff", self.on_target_event, event_qos
        )
        self.create_subscription(
            Bool, "/tracker/balloon/burst", self.on_burst, event_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            topic("/fmu/out/vehicle_local_position"),
            self.on_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            topic("/fmu/out/vehicle_status"),
            self.on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            topic("/fmu/out/vehicle_global_position"),
            self.on_global_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            topic("/fmu/out/vehicle_odometry"),
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TimesyncStatus,
            topic("/fmu/out/timesync_status"),
            self.on_timesync,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.cfg["camera_info_topic"]),
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VisualDetection,
            str(self.cfg["detection_topic"]),
            self.on_detection,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.camera = CameraGeometry(
            offset=(
                self.cfg["camera_forward"],
                self.cfg["camera_right"],
                self.cfg["camera_down"],
            ),
            pitch=math.radians(self.cfg["camera_pitch_deg"]),
        )
        self.camera_ready = False
        self.camera_frame = None
        self.clock_bridge = Px4Clock(str(self.cfg["px4_timestamp_mode"]))
        self.history = PoseHistory()
        self.estimator = TargetFilter(float(self.cfg["target_acceleration_noise"]))
        self.smoother = VelocitySmoother(
            float(self.cfg["command_acceleration"]), float(self.cfg["command_jerk"])
        )
        self.position = self.status = self.global_position = None
        self.position_wall = self.status_wall = self.odom_wall = 0.0
        self.reset_counter = None
        self.home = None
        self.yaw = 0.0
        # Calibrated because this model's PX4 heading and attitude yaw use
        # opposite signs relative to the fixed camera optical axis.
        self.yaw_frame_offset = None
        self.target_takeoff_time = None
        self.burst_confirmed = False
        self.state = "WAIT_TARGET"
        self.stream_start = None
        self.last_command = 0.0
        self.request_time = 0.0
        self.last_tick = None
        self.last_frame_stamp = -1.0
        self.last_frame_detected = False
        self.last_valid_wall = 0.0
        self.last_delay = math.nan
        self.pending_detection = None
        self.detection_streak = 0
        self.search_index = 0
        self.search_center = None
        self.reacquire_started = None
        self.last_target = None
        self.rejected_frames = 0
        self.accepted_frames = 0
        self.shutdown_reason = ""
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / float(self.cfg["control_rate"]), self.tick, clock=steady)
        self.create_timer(1.0, self.report, clock=steady)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def armed(self):
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def on_position(self, msg):
        self.position = msg
        self.position_wall = time.monotonic()

    def on_status(self, msg):
        self.status = msg
        self.status_wall = time.monotonic()

    def on_global_position(self, msg):
        self.global_position = msg

    def on_timesync(self, msg):
        self.clock_bridge.set_offset(msg.estimated_offset, time.monotonic())

    def on_odometry(self, msg):
        now = self.sim_now()
        stamp = self.clock_bridge.sample_time(
            msg.timestamp_sample, now, time.monotonic()
        )
        if (
            stamp is None
            or msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
            or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED
        ):
            return
        values = np.r_[msg.position, msg.velocity, msg.q, msg.angular_velocity]
        if not np.isfinite(values).all() or np.linalg.norm(msg.q) < 0.5:
            return
        if self.reset_counter is not None and self.reset_counter != msg.reset_counter:
            self.reset_tracking()
            if self.state in self.ACTIVE and self.state not in ("STARTING", "TAKEOFF"):
                self.state = "REACQUIRE"
                self.reacquire_started = now
        self.reset_counter = msg.reset_counter
        pose = Pose(
            stamp,
            np.array(msg.position, float),
            np.array(msg.velocity, float),
            np.array(msg.q, float),
            np.array(msg.angular_velocity, float),
        )
        if self.history.add(pose):
            self.odom_wall = time.monotonic()

    def on_camera_info(self, msg):
        # The current Gazebo camera is undistorted. Do not silently apply pinhole
        # equations to a distorted replacement camera.
        if any(abs(v) > 1e-9 for v in msg.d):
            self.get_logger().error(
                "Rectified camera input is required.", throttle_duration_sec=5.0
            )
            self.camera_ready = False
            return
        try:
            self.camera.set_intrinsics(msg.width, msg.height, msg.k)
            self.camera_frame = msg.header.frame_id
            self.camera_ready = True
        except ValueError:
            self.camera_ready = False

    def on_airborne(self, msg):
        stamp = seconds(msg)
        if stamp <= 0:
            self.target_takeoff_time = None
        elif stamp <= self.sim_now() + 0.1 and self.state == "WAIT_TARGET":
            self.target_takeoff_time = stamp

    def on_target_event(self, msg):
        if not msg.data and self.state == "WAIT_TARGET":
            self.target_takeoff_time = None

    def on_burst(self, msg):
        if msg.data and self.state in self.ACTIVE and self.armed:
            self.burst_confirmed = True

    def reset_tracking(self):
        self.history.clear()
        self.estimator.clear()
        self.last_frame_stamp = -1.0
        self.pending_detection = None
        self.detection_streak = 0
        self.last_frame_detected = False
        self.last_target = None
        self.search_center = None
        self.yaw_frame_offset = None

    def setpoint_yaw_for_direction(self, direction_yaw, pose):
        """Map a NED direction to the yaw convention expected by PX4."""
        forward = rotation(pose.q) @ self.camera.body_from_camera @ np.array(
            [0.0, 0.0, 1.0]
        )
        camera_yaw = math.atan2(float(forward[1]), float(forward[0]))
        measured_heading = float(self.position.heading)
        if self.yaw_frame_offset is None or not math.isfinite(self.yaw_frame_offset):
            self.yaw_frame_offset = wrap(measured_heading - camera_yaw)
        return wrap(float(direction_yaw) + self.yaw_frame_offset)

    def on_detection(self, msg):
        if seconds(msg.header.stamp) > self.last_frame_stamp:
            self.pending_detection = msg

    def process_detection(self, now):
        msg = self.pending_detection
        if msg is None:
            return
        stamp = seconds(msg.header.stamp)
        age = now - stamp
        if (
            stamp <= self.last_frame_stamp
            or age > self.cfg["max_image_age"]
            or age < -0.04
            or stamp <= 0
        ):
            self.pending_detection = None
            self.rejected_frames += 1
            return
        if not self.camera_ready:
            return
        if (
            msg.image_width != self.camera.width
            or msg.image_height != self.camera.height
            or msg.header.frame_id != self.camera_frame
        ):
            self.pending_detection = None
            self.rejected_frames += 1
            return
        pose = self.history.at(stamp)
        if pose is None:
            # Allow telemetry to catch up without blocking the heartbeat callback.
            if age > 0.15:
                self.pending_detection = None
                self.rejected_frames += 1
            return
        self.pending_detection = None
        self.last_frame_stamp = stamp
        self.last_frame_detected = False
        self.last_delay = max(0.0, age)
        values = [msg.center_x, msg.center_y, msg.width, msg.height, msg.confidence]
        valid = (
            msg.detected
            and not msg.clipped
            and np.isfinite(values).all()
            and 0 <= msg.center_x <= 1
            and 0 <= msg.center_y <= 1
            and 0 < msg.width <= 1
            and 0 < msg.height <= 1
            and msg.confidence >= self.cfg["confidence_threshold"]
        )
        if not valid:
            self.detection_streak = 0
            return
        if (
            self.estimator.stamp is not None
            and stamp - self.estimator.stamp > self.cfg["prediction_timeout"]
        ):
            self.estimator.clear()
            self.detection_streak = 0
        try:
            point, covariance = self.camera.observe(
                values[:4], pose, float(self.cfg["target_width_m"])
            )
            accepted = self.estimator.update(stamp, point, covariance)
        except (ValueError, np.linalg.LinAlgError):
            accepted = False
        if not accepted:
            self.rejected_frames += 1
            self.detection_streak = 0
            return
        self.detection_streak += 1
        self.last_frame_detected = True
        self.last_valid_wall = time.monotonic()
        self.accepted_frames += 1
        self.last_target = point.copy()

    def ready(self):
        p = self.position
        wall = time.monotonic()
        return (
            p is not None
            and self.status is not None
            and p.xy_valid
            and p.z_valid
            and wall - self.position_wall < 2
            and wall - self.status_wall < 2
            and all(math.isfinite(v) for v in (p.x, p.y, p.z, p.heading))
        )

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = 0
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = command
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_land(self, reason):
        self.shutdown_reason = reason
        self.state = "LANDING"
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.last_command = time.monotonic()
        self.get_logger().info(f"Landing: {reason}")

    def heartbeat(self, velocity=False):
        mode = OffboardControlMode()
        mode.timestamp = 0
        mode.position = not velocity
        mode.velocity = velocity
        self.mode_pub.publish(mode)

    def send_position(self, point):
        self.heartbeat()
        sp = TrajectorySetpoint()
        sp.timestamp = 0
        sp.position = [float(v) for v in point]
        sp.velocity = sp.acceleration = sp.jerk = [math.nan] * 3
        sp.yaw = float(self.yaw)
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)

    def turn_towards(self, yaw, dt):
        step = float(self.cfg["yaw_rate_limit"]) * dt
        self.yaw = wrap(self.yaw + float(np.clip(wrap(yaw - self.yaw), -step, step)))

    def send_velocity(self, desired, dt, acceleration_scale=1.0):
        desired = np.array(desired, float)
        if self.home is not None:
            height = self.home[2] - self.position.z
            # Reserve stopping distance for the velocity loop and jerk ramp.
            up_clearance = max(0.0, self.cfg["maximum_height"] - height)
            down_clearance = max(0.0, height - self.cfg["minimum_height"])
            vz = float(self.position.vz)
            response = 0.6
            desired[2] = np.clip(
                desired[2],
                -min(
                    self.cfg["max_vertical_speed"],
                    max(0.0, up_clearance - max(0.0, -vz) * response),
                ),
                min(
                    self.cfg["max_vertical_speed"],
                    max(0.0, down_clearance - max(0.0, vz) * response),
                ),
            )
        velocity = self.smoother.step(
            desired, dt, float(self.cfg["command_acceleration"]) * acceleration_scale
        )
        self.heartbeat(velocity=True)
        sp = TrajectorySetpoint()
        sp.timestamp = 0
        sp.position = [math.nan] * 3
        sp.velocity = [float(v) for v in velocity]
        sp.acceleration = sp.jerk = [math.nan] * 3
        sp.yaw = float(self.yaw)
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)

    def search_points(self):
        center_n = float(self.cfg["target_hint_north"])
        center_e = float(self.cfg["target_hint_east"])
        if not math.isfinite(center_n):
            center_n = float(self.cfg["search_north"])
        if not math.isfinite(center_e):
            center_e = float(self.cfg["search_east"])
        half = max(1.0, float(self.cfg["search_size"]) / 2)
        own = self.global_position
        lat, lon = self.cfg["search_latitude"], self.cfg["search_longitude"]
        if math.isfinite(lat) and math.isfinite(lon) and own is not None:
            center_n = self.position.x + (lat - own.lat) * 111111.0
            center_e = self.position.y + (lon - own.lon) * 111111.0 * math.cos(
                math.radians(own.lat)
            )
            half = max(1.0, float(self.cfg["search_radius"]))
        altitude = self.home[2] - abs(float(self.cfg["search_altitude"]))
        if self.search_center is not None:
            center_n, center_e, altitude = self.search_center
            altitude = float(
                np.clip(
                    altitude,
                    self.home[2] - self.cfg["maximum_height"],
                    self.home[2] - self.cfg["minimum_height"],
                )
            )
        return [
            (center_n + dn, center_e + de, altitude)
            for dn, de in [
                (0, 0),
                (-half, -half),
                (-half, half),
                (0, half),
                (0, -half),
                (half, -half),
                (half, half),
            ]
        ]

    def track_available(self, now):
        if (
            self.estimator.stamp is None
            or now - self.estimator.stamp > self.cfg["prediction_timeout"]
        ):
            return None
        if time.monotonic() - self.last_valid_wall > 2.0:
            return None
        result = self.estimator.predict(now)
        if result is None:
            return None
        _, P = result
        sigma = math.sqrt(max(0.0, float(np.linalg.eigvalsh(P[:3, :3])[-1])))
        if sigma > self.cfg["prediction_sigma_limit"]:
            return None
        return result

    def tick(self):
        wall = time.monotonic()
        now = self.sim_now()
        if self.last_tick is not None and now < self.last_tick - 0.01:
            self.reset_tracking()
            self.target_takeoff_time = None
            self.home = None
            self.state = "WAIT_TARGET"
            self.stream_start = None
            self.last_tick = now
            return
        dt = 0.0 if self.last_tick is None else max(0.0, min(0.1, now - self.last_tick))
        self.last_tick = now
        if self.state == "LANDING":
            if not self.armed:
                self.state = "DONE"
            elif wall - self.last_command > 1.0:
                if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.last_command = wall
            return
        if self.state in ("DONE", "STOPPED"):
            return
        if not self.ready():
            self.stream_start = None
            if self.armed and self.state in self.ACTIVE:
                self.request_land(
                    "own telemetry stale/invalid (failsafe, not visual loss)"
                )
            return
        if self.home is None:
            self.home = (self.position.x, self.position.y, self.position.z)
            self.yaw = self.position.heading
        if self.stream_start is None:
            self.stream_start = wall
        self.process_detection(now)
        pose = self.history.at(now)
        if self.state == "WAIT_TARGET":
            self.send_position(self.home)
            if (
                self.target_takeoff_time is not None
                and now - self.target_takeoff_time >= self.cfg["target_hint_delay"]
                and pose is not None
                and self.camera_ready
                and wall - self.stream_start > 1.2
            ):
                self.state = "STARTING"
                self.request_time = wall
                self.get_logger().info(
                    "Target airborne head start completed; requesting tracker takeoff."
                )
            return
        if self.burst_confirmed and self.armed:
            self.request_land("simulator confirmed balloon contact")
            return
        if self.state == "STARTING":
            self.send_position(
                (
                    self.home[0],
                    self.home[1],
                    self.home[2] - self.cfg["takeoff_altitude"],
                )
            )
            if (
                self.armed
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.state = "TAKEOFF"
            elif wall - self.request_time > 20:
                self.request_land("takeoff handshake timed out")
            elif wall - self.last_command > 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_command = wall
            return
        if (
            not self.armed
            or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.state = "STOPPED"
            self.get_logger().warning(
                "PX4 left Offboard; yielding control, no automatic re-arm."
            )
            return
        if self.state == "TAKEOFF":
            altitude = self.home[2] - self.cfg["takeoff_altitude"]
            self.send_position((self.home[0], self.home[1], altitude))
            if abs(self.position.z - altitude) < 0.25:
                self.state = "SEARCH"
                self.smoother.reset(
                    [self.position.vx, self.position.vy, self.position.vz]
                )
            return
        if pose is None:
            # Coordinate/time alignment is unavailable; keep sending a bounded
            # braking command. Visual loss alone never starts automatic landing.
            self.send_velocity(np.zeros(3), dt)
            if wall - self.odom_wall > 2.0:
                self.request_land("own time-aligned odometry unavailable (failsafe)")
            return
        estimate = self.track_available(now)
        if estimate is not None:
            x, P = estimate
            if self.state in ("SEARCH", "REACQUIRE"):
                self.smoother.reset(pose.velocity)
                self.yaw = self.position.heading
            lost = (
                not self.last_frame_detected
                or now - self.estimator.stamp > self.cfg["fresh_timeout"]
            )
            if lost:
                self.state = "PREDICT"
                closure = self.cfg["lock_speed"] * max(
                    0.0,
                    1 - (now - self.estimator.stamp) / self.cfg["prediction_timeout"],
                )
            elif self.detection_streak < self.cfg["lock_frames"]:
                self.state = "LOCKING"
                closure = self.cfg["lock_speed"]
            elif np.linalg.norm(x[:3] - pose.position) < self.cfg["final_distance"]:
                self.state = "FINAL"
                closure = self.cfg["final_speed"]
            else:
                self.state = "APPROACH"
                closure = self.cfg["approach_speed"]
            desired, yaw, scale, _ = tracking_command(
                self.camera,
                pose,
                x,
                P,
                float(closure),
                float(self.cfg["control_lookahead"]),
                max_speed=float(self.cfg["max_follow_speed"]),
                max_vertical=float(self.cfg["max_vertical_speed"]),
            )
            self.turn_towards(self.setpoint_yaw_for_direction(yaw, pose), dt)
            self.send_velocity(desired, dt, scale)
            self.last_target = x[:3].copy()
            self.reacquire_started = None
            return
        if self.state not in ("SEARCH", "REACQUIRE"):
            self.state = "REACQUIRE"
            self.reacquire_started = now
            self.detection_streak = 0
        if self.state == "REACQUIRE":
            if self.reacquire_started is None:
                self.reacquire_started = now
            elapsed = now - self.reacquire_started
            center = self.last_target
            direction = (
                self.yaw
                if center is None
                else math.atan2(
                    center[1] - pose.position[1], center[0] - pose.position[0]
                )
            )
            direction += 0.6 * math.sin(1.2 * elapsed)
            if center is None:
                self.turn_towards(direction, dt)
            else:
                self.turn_towards(self.setpoint_yaw_for_direction(direction, pose), dt)
            self.send_velocity(np.zeros(3), dt)
            if (
                elapsed >= self.cfg["reacquire_time"]
                and np.linalg.norm(pose.velocity) < 0.3
            ):
                self.state = "SEARCH"
                self.search_center = None if center is None else center.copy()
                self.search_index = 0
            return
        points = self.search_points()
        point = points[self.search_index % len(points)]
        delta = np.array(point) - pose.position
        if np.linalg.norm(delta) < 0.7:
            self.search_index += 1
            point = points[self.search_index % len(points)]
            delta = np.array(point) - pose.position
        self.turn_towards(
            self.setpoint_yaw_for_direction(math.atan2(delta[1], delta[0]), pose), dt
        )
        self.send_position(point)

    def report(self):
        age = (
            math.nan
            if self.estimator.stamp is None
            else self.sim_now() - self.estimator.stamp
        )
        text = (
            f"{self.state}: camera={self.camera_ready} odometry={bool(self.history.samples)} "
            f"capture_age={age:.3f}s image_delay={self.last_delay:.3f}s "
            f"accepted={self.accepted_frames} rejected={self.rejected_frames}"
        )
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.diagnostic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SafeVisualFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.armed:
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
