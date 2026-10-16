#!/usr/bin/env python3
"""Capture Gazebo RGB frames and render-difference YOLO labels, without PX4.

Run via collect_drone_dataset.sh. No target truth enters the flight controller.
Labels include only visible vehicle pixels; target shadows are disabled during
capture. Paired empty frames provide background negatives and exact pixel boxes.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def make_world(destination):
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='drone_dataset')
    for name in ('sun', 'ground_plane'):
        ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://' + name
    ET.SubElement(world, 'gravity').text = '0 0 0'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.01'
    ET.SubElement(physics, 'real_time_update_rate').text = '100'
    ET.SubElement(world, 'plugin', name='state', filename='libgazebo_ros_state.so')
    model = ET.SubElement(world, 'model', name='capture_target')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0 0 -100 0 0 0'
    source = ET.parse(ROOT / 'models/fpv_target/fpv_target.sdf').getroot().find('model')
    for original in source.findall('link'):
        visuals = [v for v in original.findall('visual') if 'tracking_' not in v.get('name', '')]
        if not visuals:
            continue
        link = ET.SubElement(model, 'link', name=original.get('name'))
        if original.find('pose') is not None:
            link.append(copy.deepcopy(original.find('pose')))
        for visual in visuals:
            visual = copy.deepcopy(visual)
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            link.append(visual)
    camera = ET.SubElement(world, 'model', name='capture_camera')
    ET.SubElement(camera, 'static').text = 'true'
    link = ET.SubElement(camera, 'link', name='camera_link')
    sensor = ET.SubElement(link, 'sensor', name='camera', type='camera')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '20'
    settings = ET.SubElement(sensor, 'camera')
    ET.SubElement(settings, 'horizontal_fov').text = str(math.radians(80))
    image = ET.SubElement(settings, 'image')
    for key, value in [('width','640'), ('height','480'), ('format','R8G8B8')]:
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(settings, 'clip')
    ET.SubElement(clip, 'near').text = '0.05'
    ET.SubElement(clip, 'far').text = '200'
    plugin = ET.SubElement(sensor, 'plugin', name='camera_ros', filename='libgazebo_ros_camera.so')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/dataset'
    ET.SubElement(plugin, 'camera_name').text = 'camera'
    ET.SubElement(plugin, 'frame_name').text = 'capture_camera'
    # Background landmarks are far behind the sampled target volume.
    rng = np.random.default_rng(713)
    for i in range(28):
        angle = i * 2 * math.pi / 28
        obstacle = ET.SubElement(world, 'model', name=f'background_{i}')
        ET.SubElement(obstacle, 'static').text = 'true'
        h = float(rng.uniform(1, 8))
        ET.SubElement(obstacle, 'pose').text = f'{25*math.cos(angle)} {25*math.sin(angle)} {h/2} 0 0 {angle}'
        link = ET.SubElement(obstacle, 'link', name='body')
        visual = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = f'3 3 {h}'
        material = ET.SubElement(visual, 'material')
        color = ' '.join(map(str, rng.uniform(.15, .7, 3))) + ' 1'
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color
    ET.indent(sdf)
    ET.ElementTree(sdf).write(destination, encoding='unicode', xml_declaration=True)


def rotation(roll, pitch, yaw):
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler('xyz', [roll, pitch, yaw])


def collect(output, per_scene):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from gazebo_msgs.srv import SetEntityState
    from cv_bridge import CvBridge

    rclpy.init()
    node = Node('drone_dataset_capture')
    bridge = CvBridge()
    latest = [None, 0, 0.0]

    def on_frame(msg):
        latest[:] = [bridge.imgmsg_to_cv2(msg, 'bgr8'), latest[1] + 1,
                     msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9]

    node.create_subscription(Image, '/dataset/camera/image_raw', on_frame, qos_profile_sensor_data)
    client = node.create_client(SetEntityState, '/set_entity_state')
    if not client.wait_for_service(timeout_sec=45):
        raise RuntimeError('Gazebo /set_entity_state unavailable')

    def move(name, position, rpy=(0, 0, 0)):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = map(float, position)
        q = rotation(*rpy).as_quat()
        (req.state.pose.orientation.x, req.state.pose.orientation.y,
         req.state.pose.orientation.z, req.state.pose.orientation.w) = map(float, q)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Cannot move ' + name)

    def frame():
        # Drain pre-service frames, then allow multiple complete render updates.
        start = latest[1]
        deadline = time.monotonic() + 20
        while latest[1] < start + 4:
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh camera frames')
            rclpy.spin_once(node, timeout_sec=.1)
        return latest[0].copy()

    for split in ('train', 'val', 'test'):
        for kind in ('images', 'labels'):
            (output / kind / split).mkdir(parents=True, exist_ok=True)
    records = []
    rng = np.random.default_rng(20261012)
    # Whole camera-heading scenes are held out (no adjacent-frame split).
    split_scenes = {'train': [0,1,2,4,5,6,8,9,10,12], 'val': [3,11], 'test': [7,13]}
    try:
        for split, scenes in split_scenes.items():
            for scene in scenes:
                for index in range(per_scene):
                    stem = f'scene{scene:02d}_{index:04d}'
                    if (output / 'labels' / split / (stem + '.txt')).exists():
                        raise RuntimeError('Output already contains captures; use a new output directory')
                    yaw = (scene + rng.uniform(.05, .95)) * 2 * math.pi / 14
                    pitch = rng.uniform(-.18, .30)
                    roll = rng.uniform(-.12, .12)
                    cam = np.array([rng.uniform(-2,2), rng.uniform(-2,2), rng.uniform(1.7,6)])
                    rot = rotation(roll, pitch, yaw)
                    distance = float(np.exp(rng.uniform(math.log(2), math.log(14))))
                    relative = np.array([distance, rng.uniform(-.56,.56)*distance, rng.uniform(-.32,.32)*distance])
                    target = cam + rot.apply(relative)
                    if target[2] < .4:
                        cam[2] += .4 - target[2]
                        target = cam + rot.apply(relative)
                    attitude = [rng.uniform(-.4,.4), rng.uniform(-.4,.4), rng.uniform(-math.pi,math.pi)]
                    move('capture_target', (0,0,-100))
                    move('capture_camera', cam, (roll,pitch,yaw))
                    empty = frame()
                    move('capture_target', target, attitude)
                    rgb = frame()
                    delta = np.max(np.abs(rgb.astype(np.int16) - empty.astype(np.int16)), axis=2)
                    mask = (delta > 12).astype(np.uint8)
                    count, components, stats, _ = cv2.connectedComponentsWithStats(mask)
                    keep = np.zeros(mask.shape, np.uint8)
                    for label in range(1, count):
                        if stats[label, cv2.CC_STAT_AREA] >= 3:
                            keep[components == label] = 255
                    ys, xs = np.nonzero(keep)
                    if len(xs) < 12:
                        raise RuntimeError(f'Invisible target {stem}: {len(xs)} pixels')
                    x1,y1,x2,y2 = int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)
                    if x2-x1 < 5 or y2-y1 < 3:
                        raise RuntimeError('Target too small: ' + stem)
                    box = [(x1+x2)/1280, (y1+y2)/960, (x2-x1)/640, (y2-y1)/480]
                    cv2.imwrite(str(output/'images'/split/(stem+'.jpg')), rgb, [cv2.IMWRITE_JPEG_QUALITY,95])
                    (output/'labels'/split/(stem+'.txt')).write_text('0 '+' '.join(f'{v:.7f}' for v in box)+'\n')
                    records.append(dict(file=f'images/{split}/{stem}.jpg', scene=scene, camera=cam.tolist(),
                                        camera_rpy=[roll,pitch,yaw], target=target.tolist(), target_rpy=attitude,
                                        box_xyxy=[x1,y1,x2,y2], mask_pixels=len(xs), stamp=latest[2]))
                    if index % 8 == 0:
                        neg = stem + '_empty'
                        cv2.imwrite(str(output/'images'/split/(neg+'.jpg')), empty, [cv2.IMWRITE_JPEG_QUALITY,95])
                        (output/'labels'/split/(neg+'.txt')).write_text('')
                    if index % 10 == 0:
                        print(f'{split} scene={scene:02d} frame={index}/{per_scene} captured={len(records)}', flush=True)
                    (output/'capture_manifest.json').write_text(json.dumps(records, indent=2))
        (output/'data.yaml').write_text('path: .\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: drone\n')
        print(f'Completed {len(records)} labelled target frames plus empty backgrounds', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path.home()/'Desktop/drone_yolo_dataset')
    parser.add_argument('--per-scene', type=int, default=50)
    parser.add_argument('--world', type=Path)
    args = parser.parse_args()
    if args.world:
        make_world(args.world)
    else:
        collect(args.output.resolve(), args.per_scene)
