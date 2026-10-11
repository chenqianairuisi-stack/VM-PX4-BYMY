# 追踪器
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
    ExitObservation,
    Pose,
    PoseHistory,
    Px4Clock,
    TargetFilter,
    VelocitySmoother,
    YawSmoother,
    edge_recovery_hint,
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
        "BRAKING",
        "EDGE_RECOVERY",
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
            "lock_frames": 2,
            "lock_speed": 1.4,
            "approach_speed": 4.5,
            "search_speed": 1.2,
            "final_speed": 1.5,
            "final_distance": 1.5,
            "max_follow_speed": 7.0,
            "max_vertical_speed": 1.2,
            "control_rate": 50.0,
            "command_acceleration": 3.0,
            "command_jerk": 6.0,
            "yaw_rate_limit": 0.7,
            "yaw_acceleration_limit": 1.2,
            "max_image_age": 0.45,
            "fresh_timeout": 0.25,
            "prediction_timeout": 0.85,
            "prediction_sigma_limit": 2.7,
            "prediction_relative_sigma_limit": 0.3,
            "control_lookahead": 0.12,
            "max_control_lookahead": 0.8,
            "search_yaw_rate": 0.35,
            "reacquire_yaw_rate": 0.6,
            "edge_recovery_duration": 0.7,
            "edge_recovery_distance": 2.7,
            "confidence_threshold": 0.55,
            "target_width_m": 0.44,
            "target_acceleration_noise": 3.0,
            "camera_forward": 0.21,
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
            "fresh_timeout",
            "prediction_timeout",
            "prediction_sigma_limit",
            "prediction_relative_sigma_limit",
            "lock_speed",
            "target_width_m",
            "yaw_rate_limit",
            "yaw_acceleration_limit",
            "search_speed",
            "search_yaw_rate",
            "reacquire_yaw_rate",
            "edge_recovery_duration",
            "edge_recovery_distance",
            "control_lookahead",
            "max_control_lookahead",
        ):
            if not math.isfinite(float(self.cfg[name])) or self.cfg[name] <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.cfg["max_control_lookahead"] < self.cfg["control_lookahead"]:
            raise ValueError("max_control_lookahead must be >= control_lookahead")
        if self.cfg["fresh_timeout"] >= self.cfg["prediction_timeout"]:
            raise ValueError("fresh_timeout must be less than prediction_timeout")
        if int(self.cfg["lock_frames"]) < 1:
            raise ValueError("lock_frames must be at least one")
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
        self.yaw_smoother = YawSmoother(
            float(self.cfg["yaw_rate_limit"]), float(self.cfg["yaw_acceleration_limit"])
        )
        self.position = self.status = self.global_position = None
        self.position_wall = self.status_wall = self.odom_wall = 0.0
        self.reset_counter = None
        self.home = None
        self.yaw = 0.0
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
        self.track_locked = False
        self.search_position = None
        self.search_scan_started = None
        self.search_ready_since = None
        self.search_yaw = 0.0
        self.search_progress = 0.0
        self.reacquire_started = None
        self.reacquire_scan_started = None
        self.reacquire_yaw = 0.0
        self.reacquire_position = None
        self.reacquire_progress = 0.0
        self.recovery_position = None
        self.recovery_yaw = 0.0
        self.recovery_started = None
        self.last_seen = None
        self.last_target = None
        self.rejected_frames = 0
        self.accepted_frames = 0
        self.processed_frames = 0
        self.last_report_wall = time.monotonic()
        self.last_report_accepted = 0
        self.last_report_processed = 0
        self.track_reason = "no observation"
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
        self.track_locked = False
        self.reacquire_scan_started = None
        self.reacquire_position = None
        self.reacquire_progress = 0.0
        self.search_position = None
        self.search_scan_started = None
        self.search_ready_since = None
        self.search_progress = 0.0
        self.recovery_position = None
        self.recovery_started = None
        self.last_seen = None
        self.last_frame_detected = False
        self.last_target = None
        if self.position is not None:
            self.yaw = float(self.position.heading)
            self.reacquire_yaw = self.yaw
            self.yaw_smoother.reset(self.yaw)
            self.smoother.reset([self.position.vx, self.position.vy, self.position.vz])

    def setpoint_yaw_for_direction(self, direction_yaw, pose):
        """Compensate camera mounting; odometry and setpoint both use NED yaw."""
        R = rotation(pose.q)
        forward = R @ self.camera.body_from_camera @ np.array(
            [0.0, 0.0, 1.0]
        )
        camera_yaw = math.atan2(float(forward[1]), float(forward[0]))
        body_yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        return wrap(float(direction_yaw) + wrap(body_yaw - camera_yaw))

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
        self.processed_frames += 1
        self.last_frame_detected = False
        self.last_delay = max(0.0, age)
        values = [msg.center_x, msg.center_y, msg.width, msg.height, msg.confidence]
        image_detection = (
            msg.detected
            and np.isfinite(values).all()
            and 0 <= msg.center_x <= 1
            and 0 <= msg.center_y <= 1
            and 0 < msg.width <= 1
            and 0 < msg.height <= 1
            and msg.confidence >= self.cfg["confidence_threshold"]
        )
        observation = None
        if image_detection:
            R = rotation(pose.q)
            ray = self.camera.ray(msg.center_x, msg.center_y)
            direction = R @ self.camera.body_from_camera @ ray
            bearing = math.atan2(float(direction[1]), float(direction[0]))
            bearing_rate = 0.0
            if self.last_seen is not None:
                elapsed = stamp - self.last_seen.stamp
                if 0.03 <= elapsed <= 0.35:
                    bearing_rate = float(np.clip(
                        wrap(bearing - self.last_seen.bearing) / elapsed, -1.5, 1.5
                    ))
            observation = ExitObservation(
                stamp, msg.center_x, msg.center_y, msg.width, msg.height,
                bearing, bearing_rate,
                R @ self.camera.body_from_camera @ np.array([1.0, 0.0, 0.0]),
                bool(msg.clipped),
            )
        if not image_detection or msg.clipped:
            # A clipped sphere has unreliable size/range, but its last image
            # bearing still tells us which side it exited. Only associate it
            # with a recent existing track, never create one from a partial box.
            if observation is not None and self.estimator.stamp is not None:
                if stamp - self.estimator.stamp <= self.cfg["max_image_age"]:
                    result = self.estimator.predict(stamp)
                    uv = None if result is None else self.camera.project(result[0][:3], pose)
                    if uv is not None and np.linalg.norm(
                        uv - [msg.center_x, msg.center_y]
                    ) < 0.55:
                        self.last_seen = observation
            # One skipped or clipped camera frame must not restart acquisition.
            # The prediction timeout below expires the candidate instead.
            if (
                self.estimator.stamp is not None
                and stamp - self.estimator.stamp > self.cfg["prediction_timeout"]
            ):
                self.estimator.clear()
                self.detection_streak = 0
                self.track_locked = False
            return
        if (
            self.estimator.stamp is not None
            and stamp - self.estimator.stamp > self.cfg["prediction_timeout"]
        ):
            self.estimator.clear()
            self.detection_streak = 0
            self.track_locked = False
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
        self.detection_streak = min(self.detection_streak + 1, int(self.cfg["lock_frames"]))
        if self.detection_streak >= self.cfg["lock_frames"]:
            self.track_locked = True
        self.last_frame_detected = True
        self.last_valid_wall = time.monotonic()
        self.accepted_frames += 1
        self.last_target = point.copy()
        self.last_seen = observation

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

    def send_position(self, point, yaw_rate=math.nan):
        self.heartbeat()
        sp = TrajectorySetpoint()
        sp.timestamp = 0
        sp.position = [float(v) for v in point]
        sp.velocity = sp.acceleration = sp.jerk = [math.nan] * 3
        sp.yaw = float(self.yaw)
        sp.yawspeed = float(yaw_rate)
        self.setpoint_pub.publish(sp)

    def turn_towards(self, yaw, dt):
        self.yaw = self.yaw_smoother.step(yaw, dt)

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
        sp.yawspeed = float(self.yaw_smoother.rate)
        self.setpoint_pub.publish(sp)

    def track_available(self, now):
        if self.estimator.stamp is None:
            self.track_reason = "no observation"
            return None
        if now - self.estimator.stamp > self.cfg["prediction_timeout"]:
            self.track_reason = "observation expired"
            return None
        if time.monotonic() - self.last_valid_wall > 2.0:
            self.track_reason = "detector stalled"
            return None
        result = self.estimator.predict(now)
        if result is None:
            self.track_reason = "prediction unavailable"
            return None
        x, P = result
        sigma = math.sqrt(max(0.0, float(np.linalg.eigvalsh(P[:3, :3])[-1])))
        # Camera range error grows with distance. A fixed 1 m cap rejected a
        # perfectly visible balloon beyond ~10 m and left SEARCH turning in
        # place. Bound both absolute and range-relative uncertainty instead.
        origin = self.history.samples[-1].position if self.history.samples else np.zeros(3)
        distance = float(np.linalg.norm(x[:3] - origin))
        limit = max(
            float(self.cfg["prediction_sigma_limit"]),
            float(self.cfg["prediction_relative_sigma_limit"]) * distance,
        )
        if sigma > limit:
            self.track_reason = f"uncertain {sigma:.2f}/{limit:.2f}m"
            return None
        self.track_reason = "tracking"
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
            self.yaw_smoother.reset(self.yaw)
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
                if pose is not None:
                    self.begin_search(pose)
                self.smoother.reset(
                    [self.position.vx, self.position.vy, self.position.vz]
                )
            return
        if pose is None:
            # Preserve a latched hold point even through a short odometry gap.
            self.turn_towards(self.yaw, dt)
            point = {
                "SEARCH": self.search_position,
                "EDGE_RECOVERY": self.recovery_position,
                "REACQUIRE": self.reacquire_position,
            }.get(self.state)
            if point is not None:
                self.send_position(point, self.yaw_smoother.rate)
            else:
                self.send_velocity(np.zeros(3), dt)
            if wall - self.odom_wall > 2.0:
                self.request_land("own time-aligned odometry unavailable (failsafe)")
            return
        estimate = self.track_available(now)
        # A visible edge exit is stronger evidence than a delayed camera frame.
        # React while the last bearing is still recent; the longer prediction
        # grace period must not disable the existing close-range side recovery.
        if (
            estimate is not None
            and self.last_seen is not None
            and self.last_seen.stamp >= self.estimator.stamp - 1e-6
            and (
                self.last_seen.clipped
                or now - self.estimator.stamp > self.cfg["fresh_timeout"]
            )
            and edge_recovery_hint(
                self.last_seen, pose, self.last_target,
                float(self.cfg["edge_recovery_distance"]),
            ) is not None
        ):
            estimate = None
            self.track_reason = "nearby image edge exit"
        if estimate is not None:
            if self.state in ("BRAKING", "REACQUIRE", "EDGE_RECOVERY"):
                # Return from PX4 position hold with a fresh, measured velocity
                # reference, not the stale pre-loss pursuit command.
                self.smoother.reset(pose.velocity)
                self.yaw = math.atan2(rotation(pose.q)[1, 0], rotation(pose.q)[0, 0])
                self.yaw_smoother.reset(self.yaw)
            x, P = estimate
            distance = np.linalg.norm(x[:3] - pose.position)
            # Blend closure continuously while observations remain valid.
            blend = float(np.clip(
                (distance - self.cfg["final_distance"])
                / max(0.1, self.cfg["final_distance"]), 0.0, 1.0,
            ))
            blend = blend * blend * (3 - 2 * blend)
            closure = self.cfg["final_speed"] + blend * (
                self.cfg["approach_speed"] - self.cfg["final_speed"]
            ) if self.track_locked else self.cfg["lock_speed"]
            if not self.track_locked:
                self.state = "LOCKING"
            else:
                self.state = "FINAL" if distance < self.cfg["final_distance"] else "APPROACH"
            # Start a bounded approach on the first reliable observation. New
            # frames can then refine the path while the aircraft accelerates.
            # On a gap, reduce closure continuously instead of switching PX4
            # straight into position hold and re-accelerating each frame.
            age = max(0.0, now - self.estimator.stamp)
            coasting = age > self.cfg["fresh_timeout"]
            fade = float(np.clip(
                (self.cfg["prediction_timeout"] - age)
                / (self.cfg["prediction_timeout"] - self.cfg["fresh_timeout"]),
                0.0, 1.0,
            ))
            fade = fade * fade * (3.0 - 2.0 * fade)
            desired, yaw, scale, _ = tracking_command(
                self.camera,
                pose,
                x,
                P,
                float(closure) * fade,
                float(self.cfg["control_lookahead"]),
                max_speed=float(self.cfg["max_follow_speed"]),
                max_vertical=float(self.cfg["max_vertical_speed"]),
                max_lookahead=float(self.cfg["max_control_lookahead"]),
                prediction_weight=fade,
            )
            if coasting:
                # Use the predicted bearing to steer without asking for more
                # horizontal speed. Existing acceleration winds down under
                # the jerk limit rather than jumping to zero.
                limit = float(np.linalg.norm(pose.velocity[:2])) * fade
                speed = float(np.linalg.norm(desired[:2]))
                if speed > limit:
                    desired[:2] *= limit / max(speed, 1e-6)
            self.turn_towards(self.setpoint_yaw_for_direction(yaw, pose), dt)
            self.send_velocity(desired, dt, scale)
            self.last_target = x[:3].copy()
            self.reacquire_started = None
            self.reacquire_scan_started = None
            self.reacquire_position = None
            self.recovery_position = None
            return
        if self.state == "SEARCH":
            self.hold_and_scan(pose, now, dt, searching=True)
            return
        if self.state not in ("BRAKING", "EDGE_RECOVERY", "REACQUIRE"):
            hint = edge_recovery_hint(
                self.last_seen, pose, self.last_target,
                float(self.cfg["edge_recovery_distance"]),
            )
            self.detection_streak = 0
            self.track_locked = False
            self.estimator.clear()
            if hint is not None:
                offset, self.recovery_yaw = hint
                self.recovery_position = pose.position + offset
                self.recovery_position[2] = float(np.clip(
                    self.recovery_position[2],
                    self.home[2] - self.cfg["maximum_height"],
                    self.home[2] - self.cfg["minimum_height"],
                ))
                self.recovery_started = now
                self.state = "EDGE_RECOVERY"
            else:
                if np.linalg.norm(pose.velocity) > 0.2:
                    # Stay in velocity mode until the aircraft has actually
                    # slowed; a position setpoint at cruise speed is an abrupt
                    # brake. Match the first braking command to measured speed.
                    self.state = "BRAKING"
                    self.smoother.reset(pose.velocity)
                else:
                    self.state = "REACQUIRE"
                    self.begin_reacquire(pose, now)
        if self.state == "BRAKING":
            if np.linalg.norm(pose.velocity) < 0.2:
                self.state = "REACQUIRE"
                self.begin_reacquire(pose, now)
            else:
                self.turn_towards(self.yaw, dt)
                self.send_velocity(np.zeros(3), dt)
                return
        if self.state == "EDGE_RECOVERY":
            if now - self.recovery_started >= self.cfg["edge_recovery_duration"]:
                self.state = "REACQUIRE"
                self.begin_reacquire(pose, now, self.recovery_position)
            else:
                self.turn_towards(self.recovery_yaw, dt)
                self.send_position(self.recovery_position, self.yaw_smoother.rate)
                return
        if self.state == "REACQUIRE":
            self.hold_and_scan(pose, now, dt)
            return

    def begin_search(self, pose):
        """Start scanning at one position; no waypoint search is permitted."""
        yaw = math.atan2(rotation(pose.q)[1, 0], rotation(pose.q)[0, 0])
        self.search_position = pose.position.copy()
        self.search_scan_started = None
        self.search_ready_since = None
        self.search_progress = 0.0
        self.search_yaw = yaw
        self.yaw_smoother.reset(yaw)
        self.yaw = yaw

    def begin_reacquire(self, pose, now, anchor=None):
        yaw = math.atan2(rotation(pose.q)[1, 0], rotation(pose.q)[0, 0])
        self.reacquire_position = (
            pose.position.copy() if anchor is None else np.asarray(anchor, float).copy()
        )
        self.reacquire_started = now
        self.reacquire_scan_started = None
        self.reacquire_progress = 0.0
        self.reacquire_yaw = yaw
        self.yaw_smoother.reset(yaw)
        self.yaw = yaw

    def hold_and_scan(self, pose, now, dt, searching=False):
        """Hold a fixed point while turning; SEARCH repeats, REACQUIRE stops."""
        measured_yaw = math.atan2(rotation(pose.q)[1, 0], rotation(pose.q)[0, 0])
        if searching:
            if self.search_position is None:
                self.begin_search(pose)
            anchor = self.search_position
            started = self.search_scan_started
            start_yaw = self.search_yaw
            progress = self.search_progress
        else:
            if self.reacquire_position is None:
                self.begin_reacquire(pose, now)
            anchor = self.reacquire_position
            started = self.reacquire_scan_started
            start_yaw = self.reacquire_yaw
            progress = self.reacquire_progress
        speed = float(np.linalg.norm(pose.velocity))
        error = float(np.linalg.norm(pose.position - anchor))
        if searching and started is None:
            # First settle at the takeoff hold point before beginning the
            # panorama. Yaw during the residual climb creates a visible arc.
            if speed < 0.12 and error < 0.15:
                if self.search_ready_since is None:
                    self.search_ready_since = now
                elif now - self.search_ready_since >= 0.5:
                    started = now
            else:
                self.search_ready_since = None
        elif started is None and speed < 0.2 and error < 0.25:
            started = now
        if started is not None and dt > 0:
            # Never let the yaw reference race ahead of the measured aircraft.
            if abs(wrap(start_yaw + progress - measured_yaw)) < 0.35:
                configured_rate = self.cfg["search_yaw_rate" if searching else "reacquire_yaw_rate"]
                rate = min(configured_rate, self.cfg["yaw_rate_limit"])
                progress += rate * dt
                if searching:
                    progress %= 2 * math.pi
                else:
                    progress = min(2 * math.pi, progress)
        if searching:
            self.search_scan_started, self.search_progress = started, progress
        else:
            self.reacquire_scan_started, self.reacquire_progress = started, progress
        self.turn_towards(wrap(start_yaw + progress), dt)
        self.send_position(anchor, self.yaw_smoother.rate)

    def report(self):
        wall = time.monotonic()
        interval = max(1e-3, wall - self.last_report_wall)
        detector_fps = (self.processed_frames - self.last_report_processed) / interval
        accepted_fps = (self.accepted_frames - self.last_report_accepted) / interval
        self.last_report_wall = wall
        self.last_report_processed = self.processed_frames
        self.last_report_accepted = self.accepted_frames
        age = (
            math.nan
            if self.estimator.stamp is None
            else self.sim_now() - self.estimator.stamp
        )
        text = (
            f"{self.state}: camera={self.camera_ready} odometry={bool(self.history.samples)} "
            f"capture_age={age:.3f}s image_delay={self.last_delay:.3f}s "
            f"frames={detector_fps:.1f}/s valid={accepted_fps:.1f}/s "
            f"cmd_xy={np.linalg.norm(self.smoother.velocity[:2]):.2f}m/s "
            f"lock={self.detection_streak}/{self.cfg['lock_frames']} "
            f"track={self.track_reason} accepted={self.accepted_frames} "
            f"rejected={self.rejected_frames}"
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
