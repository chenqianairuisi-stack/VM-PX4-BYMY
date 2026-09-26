#!/usr/bin/env python3
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                        qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleAttitude,
)
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('px4_fly_keyboard_control',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('takeoff_altitude', 1.5)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        event_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.takeoff_event_pub = self.create_publisher(Bool, '/target/takeoff', event_qos)
        self.airborne_stamp_pub = self.create_publisher(Time, '/target/airborne_stamp', event_qos)
        self.takeoff_ground_z = None
        self.airborne_since = None
        self.airborne_announced = False
        self.takeoff_event_pub.publish(Bool(data=False))
        self.airborne_stamp_pub.publish(Time())
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_position, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self.on_status, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.on_attitude, qos_profile_sensor_data)
        self.roll = self.pitch = math.nan
        self.position = None
        self.status = None
        self.position_time = self.status_time = 0.0
        self.target = None
        self.yaw = 0.0
        self.state = 'idle'
        self.stream_start = None
        self.request_time = self.last_command = 0.0
        self.lean = None
        self.lean_until = 0.0
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=steady)
        self.create_timer(1.0, self.print_status, clock=steady)

    def on_position(self, msg):
        self.position = msg
        self.position_time = time.monotonic()

    def on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def on_attitude(self, msg):
        w, x, y, z = msg.q
        self.roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x*x + y*y)))
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))))

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def ready(self):
        p = self.position
        now = time.monotonic()
        return (p is not None and self.status is not None and p.xy_valid and p.z_valid
                and now - self.position_time < 2.0 and now - self.status_time < 2.0
                and all(math.isfinite(v) for v in (p.x, p.y, p.z, p.heading)))

    def capture(self):
        self.lean = None
        self.target = [self.position.x, self.position.y, self.position.z]
        self.yaw = self.position.heading

    def timestamp(self):
        # PX4 1.14 uCDR assigns its own receive time when timestamp is zero.
        # This avoids wall-clock drift when Gazebo runs slower than real time.
        return 0

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        self.command_pub.publish(msg)

    def land(self):
        self.lean = None
        if self.armed or self.state == 'starting':
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.last_command = time.monotonic()
            self.state = 'landing'
            self.get_logger().info('Landing requested; waiting for PX4 AUTO_LAND and disarm.')

    def tick(self):
        now = time.monotonic()
        self.check_airborne()
        if self.state == 'landing':
            if self.status is not None and not self.armed and now - self.last_command > 2.0:
                self.state = 'idle'
                self.stream_start = None
            elif self.armed and now - self.last_command > 1.0:
                if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.last_command = now
            return
        if not self.ready():
            # A stale telemetry stream must not sustain Offboard indefinitely.
            self.stream_start = None
            if self.state in ('starting', 'flying'):
                self.land()
            return
        if self.state == 'idle':
            self.capture()
        if self.target is None:
            return
        if self.stream_start is None:
            self.stream_start = now
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp()
        mode.position = True
        self.mode_pub.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [float(v) for v in self.target]
        sp.velocity = sp.acceleration = sp.jerk = [math.nan, math.nan, math.nan]
        if self.lean is not None:
            if now > self.lean_until:
                self.target[0:2] = [self.position.x, self.position.y]
                sp.position = [float(v) for v in self.target]
                self.lean = None
            else:
                # PX4 closes the acceleration/attitude loops; keep its altitude loop active.
                forward, right = self.lean
                heading = self.position.heading
                scale = 9.8066 * math.tan(math.radians(5.0))
                sp.position = [math.nan, math.nan, float(self.target[2])]
                sp.acceleration = [scale * (forward * math.cos(heading) - right * math.sin(heading)),
                                   scale * (forward * math.sin(heading) + right * math.cos(heading)),
                                   math.nan]
        sp.yaw = float(self.yaw)
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)
        if self.state == 'starting':
            if self.armed and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.state = 'flying'
                self.get_logger().info('PX4 confirmed ARMED + OFFBOARD.')
            elif now - self.request_time > 12.0:
                self.get_logger().error('Takeoff not confirmed. Check PX4 preflight messages.')
                self.land()
                if not self.armed:
                    self.state = 'idle'
            elif now - self.last_command > 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_command = now
        elif self.state == 'flying' and (
                not self.armed or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().warning('PX4 left Offboard; keyboard movement disabled.')
            self.state = 'idle'

    def engage(self):
        if not self.ready() or self.stream_start is None:
            self.get_logger().warning('Waiting for valid PX4 telemetry.')
            return
        if self.armed or self.state != 'idle':
            return
        if time.monotonic() - self.stream_start < 1.2:
            self.get_logger().warning('Wait for one second of setpoints, then press T again.')
            return
        self.capture()
        self.takeoff_ground_z = self.position.z
        self.airborne_since = None
        self.airborne_announced = False
        self.takeoff_event_pub.publish(Bool(data=False))
        self.airborne_stamp_pub.publish(Time())
        altitude = max(1.0, float(self.get_parameter('takeoff_altitude').value))
        self.target[2] -= altitude
        self.state = 'starting'
        self.request_time = time.monotonic()
        self.last_command = 0.0
        self.get_logger().info(f'Takeoff requested: {altitude:.1f} m above the current position.')

    def check_airborne(self):
        """Start the head-start interval after verified lift-off, in sim time."""
        if self.airborne_announced:
            if not self.armed:
                self.takeoff_event_pub.publish(Bool(data=False))
                self.airborne_stamp_pub.publish(Time())
                self.airborne_announced = False
            return
        if (not self.ready() or not self.armed or self.takeoff_ground_z is None
                or self.state not in ('starting', 'flying')):
            self.airborne_since = None
            return
        now = self.get_clock().now().nanoseconds*1e-9
        if self.takeoff_ground_z-self.position.z < .5:
            self.airborne_since = None
            return
        if self.airborne_since is None:
            self.airborne_since = now
        if now-self.airborne_since >= .3:
            stamp = Time()
            stamp.sec = int(self.airborne_since)
            stamp.nanosec = int((self.airborne_since-stamp.sec)*1e9)
            self.airborne_stamp_pub.publish(stamp)
            self.takeoff_event_pub.publish(Bool(data=True))
            self.airborne_announced = True
            self.get_logger().info('Target airborne above 0.5 m; tracker head-start begins.')

    def print_status(self):
        if not self.ready():
            print('Waiting for fresh PX4 position/status...', flush=True)
            return
        p = self.position
        print(f'{self.state:8s} armed={self.armed} mode={self.status.nav_state} '
              f'ENU (m): E={p.y:6.2f} N={p.x:6.2f} U={-p.z:5.2f} '
              f'RPY={self.roll:.1f}/{self.pitch:.1f}/{math.degrees(p.heading):.1f} deg', flush=True)

    def handle_key(self, key):
        key = key.lower()
        if key in ('?', 'h'):
            print('T takeoff | W/S forward/back | A/D left/right | R/F up/down | '
                  'J/L yaw | I/K pitch | U/O bank | SPACE hold | X land | Ctrl-C land and exit', flush=True)
        elif key == 't':
            self.engage()
        elif key == 'x':
            self.land()
        elif self.state == 'flying' and self.ready():
            if key in ('i', 'k', 'u', 'o'):
                self.lean = {'i': (1, 0), 'k': (-1, 0), 'u': (0, -1), 'o': (0, 1)}[key]
                self.lean_until = time.monotonic() + 0.6
                return
            if key == ' ':
                self.capture()
                return
            forward, right, up = {
                'w': (0.25, 0, 0), 's': (-0.25, 0, 0),
                'a': (0, -0.25, 0), 'd': (0, 0.25, 0),
                'r': (0, 0, 0.25), 'f': (0, 0, -0.25),
            }.get(key, (0, 0, 0))
            heading = self.position.heading
            self.target[0] += forward * math.cos(heading) - right * math.sin(heading)
            self.target[1] += forward * math.sin(heading) + right * math.cos(heading)
            self.target[2] = min(-0.3, max(-20.0, self.target[2] - up))
            # Keep a generous lead so repeated key presses can use the higher
            # PX4 velocity limit without making the target sluggish.
            dx, dy = self.target[0] - self.position.x, self.target[1] - self.position.y
            distance = math.hypot(dx, dy)
            if distance > 5.0:
                self.target[0] = self.position.x + dx * 5.0 / distance
                self.target[1] = self.position.y + dy * 5.0 / distance
            if key == 'j':
                self.yaw -= math.radians(10)
            elif key == 'l':
                self.yaw += math.radians(10)
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))


def main(args=None):
    if not sys.stdin.isatty():
        sys.exit('Run keyboard_control in an interactive terminal.')
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardControl()
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.handle_key('?')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if not key or key == '\x03':
                    break
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        try:
            if node.armed:
                node.land()
                deadline = time.monotonic() + 40.0
                while node.armed and rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if node.armed:
                    node.get_logger().warning('Landing not yet confirmed; PX4 retains landing control.')
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
