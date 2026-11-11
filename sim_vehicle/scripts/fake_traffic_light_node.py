#!/usr/bin/env python3
"""vision_traffic(std_msgs/Bool)을 흉내낸다.

new_process.cpp 에서 vision_traffic_ 로 신호등 정지-출발을 거는 곳은
(data_시흥 state 번호 기준) 3, 7, 15, 33 네 곳이다. 그 중 3/7/15 는
dist_<=6.0 부터 650->450->0 3단계 감속, 33 은 dist_<=3.7 에서 바로
멈춘다(별도 감속 없음). gps_state(current_state, dist)를 보고 있다가
목표 state 에 들어와 dist 가 그 state 의 trigger_dist 이하로 내려오면
빨간불(False)로 유지하다가 red_duration 초 뒤 파란불(True)로 바꾼다.
목표 state 를 벗어나면 다음 신호등을 위해 리셋한다.
"""
import rospy
from std_msgs.msg import Bool
from waypoint_maker.msg import State

# state -> dist_ 임계값. new_process.cpp 의 실제 판정 조건과 맞춘 값.
DEFAULT_TRIGGER_DIST = {3: 6.0, 7: 6.0, 15: 6.0, 33: 3.7}


class FakeTrafficLight:
    def __init__(self):
        target_states = rospy.get_param("~target_states", [3, 7, 15, 33])
        self.target_states = set(int(s) for s in target_states)

        trigger_dist_param = rospy.get_param("~trigger_dist", {})
        self.trigger_dist = dict(DEFAULT_TRIGGER_DIST)
        for k, v in trigger_dist_param.items():
            self.trigger_dist[int(k)] = float(v)

        self.red_duration = rospy.get_param("~red_duration", 5.0)

        self.armed_state = None      # 이번에 빨간불을 건 state (재진입 방지용)
        self.red_since = None        # 빨간불로 전환된 시각
        self.green = True            # 기본은 초록(감속 안 걸리게)

        self.pub = rospy.Publisher("vision_traffic", Bool, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~state_topic", "gps_state"),
                          State, self._state_cb)

        rospy.loginfo("fake_traffic_light: states=%s red_duration=%.1fs trigger_dist=%s",
                      sorted(self.target_states), self.red_duration, self.trigger_dist)

        rospy.Timer(rospy.Duration(0.1), self._tick)

    def _state_cb(self, msg):
        state = msg.current_state
        dist = msg.dist

        if state not in self.target_states:
            # 신호등 구간을 벗어남 -> 다음 신호등을 위해 리셋
            if self.armed_state is not None:
                rospy.loginfo("fake_traffic_light: state %d 이탈, 리셋", self.armed_state)
            self.armed_state = None
            self.red_since = None
            self.green = True
            return

        threshold = self.trigger_dist.get(state, 6.0)
        if self.armed_state != state and dist <= threshold:
            self.armed_state = state
            self.red_since = rospy.Time.now()
            self.green = False
            rospy.loginfo("fake_traffic_light: state %d dist=%.2f<=%.2f -> 빨간불",
                          state, dist, threshold)

    def _tick(self, _evt):
        if not self.green and self.red_since is not None:
            elapsed = (rospy.Time.now() - self.red_since).to_sec()
            if elapsed >= self.red_duration:
                self.green = True
                rospy.loginfo("fake_traffic_light: state %d %.1fs 경과 -> 파란불(출발)",
                              self.armed_state, elapsed)
        self.pub.publish(Bool(data=self.green))


if __name__ == "__main__":
    rospy.init_node("fake_traffic_light_node")
    FakeTrafficLight()
    rospy.spin()
