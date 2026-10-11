#!/usr/bin/env python3
"""Measure target-model rebound in a running dual Gazebo/PX4 SITL."""

import json
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from px4_fly_control.keyboard_control import KeyboardControl


def main():
    rclpy.init()
    node = KeyboardControl()
    node.set_parameters([Parameter('takeoff_altitude', value=3.0)])
    samples = []
    model_z = None
    output = Path(__file__).resolve().parents[1] / 'runtime' / 'verification'
    output.mkdir(parents=True, exist_ok=True)

    def on_models(msg):
        nonlocal model_z
        if 'target' not in msg.name:
            return
        i = msg.name.index('target')
        model_z = msg.pose[i].position.z
        samples.append((node.get_clock().now().nanoseconds * 1e-9,
                        model_z, msg.twist[i].linear.z, node.state, node.armed))

    node.create_subscription(ModelStates, '/model_states', on_models,
                             qos_profile_sensor_data)

    def wait_until(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f'Timed out waiting for {label}')

    try:
        wait_until(lambda: node.ready() and model_z is not None and
                   node.stream_start is not None and
                   time.monotonic() - node.stream_start > 1.5,
                   60, 'target telemetry and Offboard heartbeat')
        ground_z = model_z
        node.handle_key('t')
        wait_until(lambda: node.state == 'flying', 25, 'takeoff confirmation')
        wait_until(lambda: model_z is not None and model_z > ground_z + 2.6,
                   40, 'target at 3 m')
        node.handle_key('x')
        wait_until(lambda: not node.armed, 55, 'automatic landing and disarm')
        landing = [row for row in samples if row[3] == 'landing']
        contact = next((i for i, row in enumerate(landing)
                        if row[1] <= ground_z + 0.012), None)
        if contact is None:
            raise RuntimeError('No physical ground contact in model trace')
        post = landing[contact:]
        result = {
            'touchdown_speed_m_s': abs(landing[contact][2]),
            'max_rebound_m': max(0.0, max(row[1] for row in post) - landing[contact][1]),
            'ground_z_m': ground_z,
            'final_z_m': model_z,
            'disarmed': not node.armed,
            'samples': len(samples),
        }
        (output / 'target_landing_result.json').write_text(
            json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        if result['max_rebound_m'] > 0.03:
            raise AssertionError('Target rebounded more than 3 cm after touchdown')
    finally:
        if node.armed:
            node.land()
            try:
                wait_until(lambda: not node.armed, 55, 'cleanup landing')
            except RuntimeError:
                node.get_logger().error('Target still armed after cleanup timeout')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
