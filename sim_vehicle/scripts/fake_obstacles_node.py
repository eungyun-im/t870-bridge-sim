#!/usr/bin/env python3
"""map 좌표에 놓아둔 장애물을 실제 라이다처럼 보이게 발행한다.

obstacle_detector 가 내는 것과 같은 obstacle_detector/Obstacles 를
frame "laser" 로 낸다. 즉 map 에 고정해 둔 콘을 매 주기 자차 기준
좌표로 바꿔서, 라이다 범위/시야 안에 들어온 것만 발행한다.
소비 노드(lane_avoid_selector, obstacle_path_bridge)가 mount_x/y/yaw 와
자차 pose 로 다시 map 으로 되돌리므로, 실제 파이프라인과 동일한 경로를 탄다.

장애물 목록은 ~obstacles 파라미터로 준다: [x, y, r, x, y, r, ...] (map 좌표)
"""
import math

import rospy
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles, CircleObstacle


class FakeObstacles:
    def __init__(self):
        flat = rospy.get_param("~obstacles", [])
        if len(flat) % 3 != 0:
            rospy.logwarn("fake_obstacles: ~obstacles 길이가 3의 배수가 아님. 뒤를 버립니다.")
            flat = flat[: len(flat) - (len(flat) % 3)]
        self.obstacles = [(flat[i], flat[i + 1], flat[i + 2])
                          for i in range(0, len(flat), 3)]

        # 라이다 사양 (RPLIDAR 급 기본값)
        self.max_range = rospy.get_param("~max_range", 12.0)
        self.min_range = rospy.get_param("~min_range", 0.15)
        self.fov_deg = rospy.get_param("~fov_deg", 360.0)

        # 소비 노드와 같은 값이어야 map 복원이 맞다
        self.mount_x = rospy.get_param("~mount_x", 0.0)
        self.mount_y = rospy.get_param("~mount_y", 0.0)
        self.mount_yaw = rospy.get_param("~mount_yaw", 0.0)

        self.rate_hz = rospy.get_param("~rate_hz", 10.0)
        self.frame_id = rospy.get_param("~frame_id", "laser")

        self.x = self.y = self.yaw = None
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odom_fused"),
                         Odometry, self._odom)
        self.pub = rospy.Publisher(rospy.get_param("~obstacles_topic", "/obstacles"),
                                   Obstacles, queue_size=1)

        rospy.loginfo("fake_obstacles: %d개, range %.1fm, fov %.0fdeg",
                      len(self.obstacles), self.max_range, self.fov_deg)
        for i, (ox, oy, orr) in enumerate(self.obstacles):
            rospy.loginfo("  [%d] map (%.2f, %.2f) r=%.2f", i, ox, oy, orr)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

    def _odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _tick(self, _evt):
        if self.x is None:
            return
        out = Obstacles()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cm, sm = math.cos(self.mount_yaw), math.sin(self.mount_yaw)

        for ox, oy, orr in self.obstacles:
            # map -> base_link
            dx, dy = ox - self.x, oy - self.y
            bx = dx * cy + dy * sy
            by = -dx * sy + dy * cy
            # base_link -> laser (소비 노드의 toGlobal 역변환)
            rx, ry = bx - self.mount_x, by - self.mount_y
            lx = rx * cm + ry * sm
            ly = -rx * sm + ry * cm

            d = math.hypot(lx, ly)
            if d > self.max_range or d < self.min_range:
                continue
            if self.fov_deg < 360.0:
                if abs(math.degrees(math.atan2(ly, lx))) > self.fov_deg * 0.5:
                    continue

            c = CircleObstacle()
            c.center.x = lx
            c.center.y = ly
            c.radius = orr
            c.true_radius = orr
            out.circles.append(c)

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("fake_obstacles_node")
    FakeObstacles()
    rospy.spin()
