#!/usr/bin/env python3
"""Exercise the real SITL controller and capture camera evidence. Start simulation first."""
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from px4_fly_control.keyboard_control import KeyboardControl


def main():
    rclpy.init()
    node = KeyboardControl()
    frames = []

    def on_image(msg):
        # Gazebo's RGB image includes a per-row byte stride.
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frames.append((time.monotonic(), frame))
        if len(frames) > 100:
            del frames[0]

    node.create_subscription(Image, '/front_camera/image_raw', on_image, qos_profile_sensor_data)
    output = Path(__file__).resolve().parents[1] / 'runtime' / 'verification'
    output.mkdir(parents=True, exist_ok=True)
    report = {}

    def wait_for(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                print('PASS:', label, flush=True)
                return
        raise RuntimeError('Timeout: ' + label)

    def settle(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        wait_for(lambda: node.ready() and len(frames) >= 5, 60, 'telemetry and camera')
        settle(2)
        z0 = node.position.z
        node.handle_key('t')
        wait_for(lambda: node.state == 'flying', 20, 'armed and Offboard')
        wait_for(lambda: abs(node.position.z - (z0 - 1.5)) < 0.25, 35, 'takeoff to 1.5 m')
        settle(2)
        cv2.imwrite(str(output / 'camera_hover.png'), frames[-1][1])
        report['hover_height_m'] = z0 - node.position.z
        report['image_shape'] = list(frames[-1][1].shape)
        report['image_stddev'] = float(frames[-1][1].std())
        assert report['image_stddev'] > 5, 'Camera frame is blank'
        report['image_wall_fps'] = (len(frames)-1) / (frames[-1][0]-frames[0][0])
        x0, y0 = node.position.x, node.position.y
        for _ in range(4):
            node.handle_key('w')
        tx, ty = node.target[:2]
        wait_for(lambda: math.hypot(node.position.x-tx, node.position.y-ty) < 0.25,
                 25, 'forward movement')
        report['horizontal_displacement_m'] = math.hypot(node.position.x-x0, node.position.y-y0)
        assert report['horizontal_displacement_m'] > 0.65
        for _ in range(3):
            node.handle_key('l')
        wait_for(lambda: abs(math.atan2(math.sin(node.position.heading-node.yaw),
                                       math.cos(node.position.heading-node.yaw))) < 0.1,
                 20, 'right yaw 30 degrees')
        settle(2)
        cv2.imwrite(str(output / 'camera_moved.png'), frames[-1][1])
        observed_pitch = []
        for _ in range(20):
            node.handle_key('i')
            settle(0.1)
            observed_pitch.append(node.pitch)
        report['forward_lean_min_pitch_deg'] = min(observed_pitch)
        assert min(observed_pitch) < -2.0, 'Forward lean did not pitch the aircraft'
        node.handle_key(' ')
        settle(2)
        node.handle_key('x')
        wait_for(lambda: not node.armed, 45, 'land and auto-disarm')
        report['landed_disarmed'] = True
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        if node.armed:
            node.land()
            wait_for(lambda: not node.armed, 45, 'cleanup landing')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
