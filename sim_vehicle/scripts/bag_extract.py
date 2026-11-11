#!/usr/bin/env python3
"""bag -> pickle. offline_sim 보정과 실주행 대조에 쓰는 공통 추출기.

컨테이너 /tmp 는 세션이 끝나면 지워지므로 이 파일은 레포에 둔다.
사용법: python3 bag_extract.py <bag> <out.pkl>
"""
import rosbag, pickle, math, sys

def yawof(o): return math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z))

def main(bagpath, outpath):
    d = {k: [] for k in ['odom','pose','cpose','wodom','ready','cmds','cmdg','cmdst',
                          'aspd','agear','enc','estop','mode','wp','gps','imu','status']}
    with rosbag.Bag(bagpath) as b:
        for tp, m, t in b.read_messages():
            ts = t.to_sec()
            if tp == '/odom_fused':
                p = m.pose.pose.position
                d['odom'].append((ts, p.x, p.y, yawof(m.pose.pose.orientation), m.twist.twist.linear.x))
            elif tp == '/localization/pose':
                p = m.pose.pose.position
                d['pose'].append((ts, p.x, p.y, yawof(m.pose.pose.orientation)))
            elif tp == '/vehicle1/c099/pose':
                p = m.pose.pose.position
                d['cpose'].append((ts, p.x, p.y, yawof(m.pose.pose.orientation)))
            elif tp == '/wheel/odom':
                d['wodom'].append((ts, m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == '/localization/ready':
                d['ready'].append((ts, int(m.data)))
            elif tp == '/vehicle/cmd_speed':
                d['cmds'].append((ts, m.data))
            elif tp == '/vehicle/cmd_gear':
                d['cmdg'].append((ts, m.data))
            elif tp == '/vehicle/cmd_steer':
                d['cmdst'].append((ts, m.data))
            elif tp == '/vehicle/applied_speed':
                d['aspd'].append((ts, m.data))
            elif tp == '/vehicle/applied_gear':
                d['agear'].append((ts, m.data))
            elif tp == '/vehicle/encoder':
                d['enc'].append((ts, m.data))
            elif tp == '/vehicle/estop':
                d['estop'].append((ts, int(m.data)))
            elif tp == '/vehicle/mode':
                d['mode'].append((ts, m.data))
            elif tp == '/final_waypoints':
                d['wp'].append((ts, [(w.pose.pose.position.x, w.pose.pose.position.y,
                                       w.mission_state, w.rk, w.ryaw) for w in m.waypoints]))
            elif tp == '/gps_state':
                d['gps'].append((ts, {f: getattr(m, f) for f in m.__slots__
                                       if isinstance(getattr(m, f), (int, float, bool))}))
            elif tp == '/imu/data':
                d['imu'].append((ts, m.linear_acceleration.y, m.angular_velocity.z))
            elif tp == '/vehicle1/c099/status':
                d['status'].append((ts, float(m.horizontal_accuracy_m), float(m.heading_accuracy_deg),
                                     int(m.localization_ready)))
    pickle.dump(d, open(outpath, 'wb'))
    print({k: len(v) for k, v in d.items() if v})

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
