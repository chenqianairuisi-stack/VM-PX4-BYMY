#!/usr/bin/env python3
"""Exercise drone following in dual SITL. Model truth is evaluation-only.

Start the drone world and YOLO detector first; do not start keyboard/follow nodes.
"""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from px4_fly_control.keyboard_control import KeyboardControl
from px4_fly_control.safe_visual_follow import SafeVisualFollow
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    scenario = parser.add_mutually_exclusive_group()
    scenario.add_argument('--fast', action='store_true',
                        help='Sustain repeated target key inputs and measure both vehicle speeds')
    scenario.add_argument('--lateral', action='store_true',
                          help='Approach the camera, then cross laterally and reverse direction')
    parser.add_argument('--output', type=Path, default=ROOT/'runtime/drone_follow_verification')
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '--params-file', str(ROOT/'config/drone_follow.yaml')])
    target = KeyboardControl()
    target.set_parameters([Parameter('takeoff_altitude', value=3.0)])
    follower = SafeVisualFollow()
    executor = SingleThreadedExecutor()
    executor.add_node(target)
    executor.add_node(follower)
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    poses = {}
    speeds = {}
    yaw_rates = {}
    samples = []
    report = {'passed': False}
    phase = 'startup'
    last_sample = [0.0]
    last_image = [0.0]

    def on_models(msg):
        for name in ('target','tracker'):
            if name in msg.name:
                p = msg.pose[msg.name.index(name)].position
                poses[name] = np.array([p.x,p.y,p.z])
                v = msg.twist[msg.name.index(name)].linear
                speeds[name] = float(np.hypot(v.x, v.y))
                yaw_rates[name] = float(msg.twist[msg.name.index(name)].angular.z)
        now = time.monotonic()
        if len(poses) == 2 and now-last_sample[0] > .25:
            last_sample[0] = now
            own = follower.position
            estimated_range = (float(np.linalg.norm(
                follower.estimator.x[:3] - np.array([own.x, own.y, own.z])
            )) if own is not None and follower.estimator.stamp is not None else None)
            samples.append(dict(wall=now,sim=follower.sim_now(),phase=phase,state=follower.state,
                                distance=float(np.linalg.norm(poses['target']-poses['tracker'])),
                                target=poses['target'].tolist(),tracker=poses['tracker'].tolist(),
                                target_speed=speeds['target'],tracker_speed=speeds['tracker'],
                                tracker_yaw_rate=yaw_rates['tracker'],
                                detection_fresh=follower.last_frame_detected and
                                    follower.estimator.stamp is not None and
                                    follower.sim_now()-follower.estimator.stamp < .25,
                                estimated_range=estimated_range,
                                last_box_width=None if follower.last_seen is None else follower.last_seen.width,
                                accepted=follower.accepted_frames,reason=follower.track_reason))

    def on_image(msg):
        if time.monotonic()-last_image[0] > 5:
            last_image[0] = time.monotonic()
            cv2.imwrite(str(output/f'{phase}_{len(samples):04d}.jpg'),CvBridge().imgmsg_to_cv2(msg,'bgr8'))

    target.create_subscription(ModelStates,'/model_states',on_models,qos_profile_sensor_data)
    target.create_subscription(Image,'/tracker/yolo/annotated',on_image,qos_profile_sensor_data)

    def spin_until(predicate, timeout, label, guard=True):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if guard and follower.armed and len(poses)==2 and np.linalg.norm(poses['target']-poses['tracker'])<1.8:
                raise RuntimeError('Evaluation stopped: separation below 1.8 m')
            if predicate():
                print('PASS:',label,flush=True)
                return
        raise RuntimeError('Timeout: '+label)

    def observe(seconds):
        start=follower.sim_now()
        spin_until(lambda:follower.sim_now()-start>=seconds,seconds*4+10,'observe '+str(seconds)+' simulation seconds')

    def press_for(key, seconds):
        start = follower.sim_now()
        next_press = start
        def moved_long_enough():
            nonlocal next_press
            now = follower.sim_now()
            if now >= next_press:
                target.handle_key(key)
                next_press = now + .08
            return now-start >= seconds
        spin_until(moved_long_enough, seconds*5+15, f'key {key} for {seconds} simulation seconds')

    try:
        spin_until(lambda:target.ready() and follower.ready() and len(poses)==2 and target.stream_start is not None
                   and time.monotonic()-target.stream_start>1.5,60,'telemetry')
        phase='takeoff'
        target.handle_key('t')
        spin_until(lambda:target.state=='flying' and poses['target'][2]>2.7,60,'target takeoff')
        spin_until(lambda:follower.track_locked and follower.state in ('APPROACH','FINAL'),100,'YOLO lock')
        phase='hover_follow'
        observe(6 if args.fast or args.lateral else 12)
        phase='moving_target'
        if args.lateral:
            press_for('w', 1.0)
            press_for('a', 2.5)
            press_for('d', 3.5)
            observe(8)
        elif args.fast:
            press_for('s', 5)
            observe(8)
        else:
            for _ in range(6):
                target.handle_key('s')
            observe(18)
        rows=[s for s in samples if s['phase'] in ('hover_follow','moving_target')]
        tracked=[s for s in rows if s['state'] in ('APPROACH','FINAL')]
        report.update(tracked_fraction=len(tracked)/max(1,len(rows)),
                      min_separation_m=min(s['distance'] for s in rows),
                      final_separation_m=rows[-1]['distance'],accepted_frames=follower.accepted_frames)
        moving = [s for s in rows if s['phase']=='moving_target']
        report['target_displacement_m'] = float(np.linalg.norm(np.array(moving[-1]['target'])-moving[0]['target']))
        report['tracker_displacement_m'] = float(np.linalg.norm(np.array(moving[-1]['tracker'])-moving[0]['tracker']))
        report['target_peak_xy_speed_m_s'] = max(s['target_speed'] for s in moving)
        report['tracker_peak_xy_speed_m_s'] = max(s['tracker_speed'] for s in moving)
        report['fast_trial'] = args.fast
        report['lateral_trial'] = args.lateral
        report['bearing_only_frames'] = follower.bearing_frames
        report['moving_tracked_fraction'] = sum(s['state'] in ('APPROACH','FINAL') for s in moving) / len(moving)
        report['fresh_detection_fraction'] = sum(s['detection_fresh'] for s in rows) / len(rows)
        report['tracker_peak_yaw_rate_deg_s'] = float(np.degrees(max(abs(s['tracker_yaw_rate']) for s in moving)))
        report['max_box_width_fraction'] = max(s['last_box_width'] or 0 for s in moving)
        longest_loss = 0.0
        loss_start = None
        for row in rows:
            if row['state'] not in ('APPROACH','FINAL'):
                if loss_start is None:
                    loss_start = row['sim']
                longest_loss = max(longest_loss, row['sim']-loss_start)
            else:
                loss_start = None
        report['longest_untracked_sim_s'] = longest_loss
        report['mean_abs_range_error_m'] = float(np.mean([
            abs(s['estimated_range']-s['distance']) for s in moving
            if s['estimated_range'] is not None
        ]))
        if min(report['target_displacement_m'],report['tracker_displacement_m']) < .7:
            raise RuntimeError('Target or follower moved less than 0.7 m')
        if args.fast and min(report['target_peak_xy_speed_m_s'], report['tracker_peak_xy_speed_m_s']) < 1.8:
            raise RuntimeError('At least one vehicle did not reach 1.8 m/s in the fast trial')
        if args.fast and report['min_separation_m'] < 2.5:
            raise RuntimeError('Fast trial separation fell below 2.5 m')
        if args.fast and not 3.5 <= report['final_separation_m'] <= 8.0:
            raise RuntimeError('Fast trial did not settle to a stable 3.5-8 m separation')
        if args.lateral and (report['moving_tracked_fraction'] < .85 or longest_loss > 1.0):
            raise RuntimeError('Lateral trial lost tracking for too long')
        if args.lateral and report['min_separation_m'] < 2.5:
            raise RuntimeError('Lateral trial separation fell below 2.5 m')
        if report['tracked_fraction']<.7:
            raise RuntimeError('Tracking lost for more than 30% of observation')
        if not 2<=report['final_separation_m']<=7:
            raise RuntimeError('Final separation outside 2-7 m')
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        print('FAIL:',exc,flush=True)
    finally:
        phase='landing'
        if follower.armed:follower.request_land('drone verification complete')
        if target.armed:target.land()
        try:
            spin_until(lambda:not follower.armed and not target.armed,70,'both vehicles landed/disarmed',guard=False)
            report['both_disarmed']=True
        except Exception as exc:
            report['both_disarmed']=False
            report['cleanup_error']=str(exc)
            report['passed']=False
        (output/'result.json').write_text(json.dumps(report,indent=2))
        (output/'samples.json').write_text(json.dumps(samples,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        executor.shutdown()
        target.destroy_node();follower.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
