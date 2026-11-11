#!/usr/bin/env python3
"""sim_vehicle 실행마다 자동으로 CTE / state 진행 / 미션 성공여부를 남긴다.

2026-09-14: 원래는 /tmp/eval_full.csv 하나에 매번 덮어써서, 돌릴 때마다
이전 결과가 사라졌다(A/B 비교 불가) -- 실행마다 타임스탬프 디렉토리를
새로 만들고, 컨테이너 /tmp 가 아니라 호스트에서도 보이는
catkin_ws/src 아래에 저장한다(도커 바인드마운트: catkin_ws/data/Downloads
만 호스트와 공유되고 /tmp 는 컨테이너 전용이라 예전엔 호스트에서
안 보였다).

저장물 (logs/<타임스탬프>_<label>/):
  timeseries.csv   t,x,y,mission_state,lane,cross_track,steer  (매 odom 틱)
  summary.txt      사람이 읽는 요약: 성공여부, state별 CTE, 조향반전수

"성공"의 정의: /global_waypoints(로더가 시작할 때 한 번 latched 로 내는
전체 경로)에 들어있는 mission_state 중 최댓값을 "완주 목표 state"로 보고,
그 state 를 실제로 밟았으면 성공이다. 그 중간에 /gps_state 가 10초 넘게
전혀 안 들어오면(멈췄거나 죽었다고 보고) 실패로 기록한다.
"""
import math
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from waypoint_maker.msg import Lane, State


class Eval:
    def __init__(self, outdir, label):
        self.label = label
        self.waypoints = None
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.log = open(os.path.join(outdir, "timeseries.csv"), "w")
        self.log.write("t,x,y,mission_state,lane,cross_track,steer\n")

        self.last_steer = None
        self.reversals = 0
        self.steer = 0.0
        self.start = rospy.Time.now()
        self.max_cte = 0.0
        self.cte_sum2 = 0.0
        self.n = 0

        self.cur_state = -1
        self.cur_lane = -1
        self.states_seen = set()
        # state -> [n, max_cte, cte_sum]
        self.per_state = {}
        self.state_enter_t = {}   # state -> 처음 그 state 를 본 sim 시각
        self.final_state_target = None
        self.last_gps_state_wall = time.time()
        self.stalled = False

        rospy.Subscriber("/final_waypoints", Lane, self.wp_cb)
        rospy.Subscriber("/global_waypoints", Lane, self.global_wp_cb)
        rospy.Subscriber("/odom_fused", Odometry, self.odom_cb)
        rospy.Subscriber("/vehicle/cmd_steer", Float32, self.steer_cb)
        rospy.Subscriber("gps_state", State, self.state_cb)

        rospy.Timer(rospy.Duration(1.0), self.watchdog_cb)

    def wp_cb(self, msg):
        self.waypoints = msg.waypoints

    def global_wp_cb(self, msg):
        if not msg.waypoints:
            return
        m = max(w.mission_state for w in msg.waypoints)
        if self.final_state_target is None:
            self.final_state_target = m
            rospy.loginfo("[eval] 완주 목표 mission_state = %d (global_waypoints 로 확인)", m)

    def state_cb(self, msg):
        self.cur_state = msg.current_state
        self.cur_lane = msg.lane_number
        self.states_seen.add(msg.current_state)
        self.last_gps_state_wall = time.time()
        t = (rospy.Time.now() - self.start).to_sec()
        if msg.current_state not in self.state_enter_t:
            self.state_enter_t[msg.current_state] = t

    def watchdog_cb(self, _evt):
        # gps_state 가 10초 넘게 안 들어오면(로더/차 죽음) 멈춘 걸로 본다.
        if self.cur_state >= 0 and time.time() - self.last_gps_state_wall > 10.0:
            self.stalled = True

    def steer_cb(self, msg):
        s = msg.data
        if self.last_steer is not None:
            if (s > 0.3 and self.last_steer < -0.3) or (s < -0.3 and self.last_steer > 0.3):
                self.reversals += 1
        self.last_steer = s
        self.steer = s

    def odom_cb(self, msg):
        if not self.waypoints:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        mind = min(
            math.hypot(x - w.pose.pose.position.x, y - w.pose.pose.position.y)
            for w in self.waypoints
        )
        t = (rospy.Time.now() - self.start).to_sec()
        self.log.write("%.2f,%.3f,%.3f,%d,%d,%.4f,%.3f\n" %
                        (t, x, y, self.cur_state, self.cur_lane, mind, self.steer))
        self.log.flush()
        self.n += 1
        self.cte_sum2 += mind * mind
        if mind > self.max_cte:
            self.max_cte = mind

        if self.cur_state >= 0:
            ps = self.per_state.setdefault(self.cur_state, [0, 0.0, 0.0])
            ps[0] += 1
            ps[1] = max(ps[1], mind)
            ps[2] += mind

    def report(self):
        rms = math.sqrt(self.cte_sum2 / self.n) if self.n else 0.0
        elapsed = (rospy.Time.now() - self.start).to_sec()

        success = (self.final_state_target is not None
                   and self.final_state_target in self.states_seen
                   and not self.stalled)
        reason = ""
        if self.final_state_target is None:
            reason = "global_waypoints 를 못 받아 목표 state 를 모름"
        elif self.stalled:
            reason = "gps_state 가 10초 넘게 끊긴 구간 있음(정지/사망 의심)"
        elif self.final_state_target not in self.states_seen:
            reason = "목표 state %d 를 못 밟음(최대로 밟은 state=%d)" % (
                self.final_state_target, max(self.states_seen) if self.states_seen else -1)

        lines = []
        lines.append("label=%s" % self.label)
        lines.append("결과: %s%s" % ("성공" if success else "실패",
                                    ("" if success else " (%s)" % reason)))
        lines.append("소요시간: %.1fs" % elapsed)
        lines.append("전체: N=%d maxCTE=%.3fm rmsCTE=%.3fm 조향반전=%d회" %
                      (self.n, self.max_cte, rms, self.reversals))
        lines.append("밟은 state: %s" % sorted(self.states_seen))
        if self.final_state_target is not None:
            lines.append("목표 state: %d" % self.final_state_target)
        lines.append("")
        lines.append("state별 CTE (진입시각, n, 평균, 최대):")
        for st in sorted(self.per_state):
            n, mx, s = self.per_state[st]
            avg = s / n if n else 0.0
            enter = self.state_enter_t.get(st, -1.0)
            lines.append("  state %2d  t=%7.1fs  n=%4d  평균=%.3fm  최대=%.3fm" %
                         (st, enter, n, avg, mx))

        summary = "\n".join(lines)
        rospy.loginfo("\n" + summary)
        with open(os.path.join(self.outdir, "summary.txt"), "w") as f:
            f.write(summary + "\n")
        self.log.close()
        print("\n[eval] 저장 위치: %s" % self.outdir)


if __name__ == "__main__":
    rospy.init_node("eval_metrics")
    # 2026-09-14: sys.argv 를 그대로 쓰면 roslaunch 가 자동으로 뒤에 붙이는
    # __name:=eval_metrics/__log:=... 같은 ROS 리매핑 인자까지 위치 인자로
    # 같이 읽힌다. args="라벨" 하나만 줬는데 sys.argv[2] 가 "__name:=eval_metrics"
    # 가 돼서 그게 그대로 저장 디렉토리가 돼버렸다(~/.ros/__name:=eval_metrics/...
    # 에 실제로 저장됨, 실측 확인됨). rospy.myargv() 로 리매핑 인자를 걷어낸
    # "진짜" 커맨드라인만 받는다.
    argv = rospy.myargv(argv=sys.argv)
    label = argv[1] if len(argv) > 1 else "run"
    base = argv[2] if len(argv) > 2 else os.path.expanduser(
        "~/catkin_ws/src/sim_vehicle/logs")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(base, "%s_%s" % (stamp, label))
    e = Eval(outdir, label)
    rospy.on_shutdown(e.report)
    rospy.spin()
