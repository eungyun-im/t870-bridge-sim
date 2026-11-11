#!/usr/bin/env python3
import csv
import math
import random
import threading

import rospy
import tf
from std_msgs.msg import Int32, Float32, Bool
from nav_msgs.msg import Odometry

FORWARD, NEUTRAL, BACKWARD = 0, 1, 2


def read_initial_pose_from_csv(path):
    with open(path, newline="") as f:
        row = next(csv.reader(f))
    return float(row[1]), float(row[2]), float(row[4])


# 2026-09-09: 국민대 실주행 7회(run3/4/5/6/7/8/9) bag 으로 확인한, GPS 드롭아웃과
# 무관한 EKF 지연. /odom_fused yaw 를 미분한 값과 IMU 자이로의 교차상관으로 잰
# "EKF 가 실제보다 얼마나 늦게 따라오는가" 가, RTK 는 계속 Fixed 인 채로도 속도
# 임계 근방에서 갑자기 커진다:
#   v<=1.86 m/s : 지연 20~160 ms (상관 0.84~0.95, 잘 따라옴)
#   v>=2.31 m/s : 지연 380~420 ms (상관 0.42~0.50, 사실상 발산)
# 계단형 로지스틱으로 피팅(scipy curve_fit, RMS 잔차 44.7 ms):
#   L0=80ms, L1=420ms, v0=2.274 m/s, w=0.020
# 원인은 EKF 의 프로세스 노이즈(proc_stddev_yaw_c/wz_c)가 실측 요레이트(최대
# 1.6 rad/s)에 비해 작아, GNSS 업데이트 사이(0.2s)에 P 가 못 커져서 정상적인
# 보정이 반복 거부되다 한 번에 흡수되는 것 - 위치가 여러 GNSS 샘플에 걸쳐
# 계단식으로 벌어지다 흡수되는 패턴을 직접 확인했다(공분산도 같이 부풀었다 꺼짐).
# 2026-09-09 오후에 autoware_ekf.yaml 의 proc_stddev_yaw_c(0.03->0.10) /
# proc_stddev_wz_c(0.2->0.60) 를 올려 대응했으나 재검증은 아직이다.
#
# 기존 gps_dropout_profile(RTK 드롭아웃->복구 점프)과는 별개 메커니즘이다 -
# 그쪽은 ready 플래그가 실제로 내려간 드문 경우(run2)만 다루고, 이건 GPS가
# 완벽해도(ready 100%) 일어나는, 오늘 데이터에서 훨씬 흔했던 쪽이다(run4/6/
# 10/12). 아래는 delay-line 으로 구현한다: 실제(x,y,yaw) 를 이력 버퍼에 쌓아
# 두고, 컨트롤러에는 lag_model(v) 만큼 과거 값을 낸다. 정확한 "한 번에
# 흡수되는 점프"보다는 매끄러운 지연으로 단순화했지만, 컨트롤러 CTE 에
# 미치는 영향(과거 위치 보고 조향)은 같은 방향이다.
POS_LAG_L0 = 0.080
POS_LAG_L1 = 0.420
POS_LAG_V0 = 2.274
POS_LAG_W = 0.020


def default_pos_lag(v):
    v = abs(v)
    return POS_LAG_L0 + (POS_LAG_L1 - POS_LAG_L0) / (1.0 + math.exp(-(v - POS_LAG_V0) / POS_LAG_W))


def load_gps_dropout_profile(path):
    events = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            events.append((float(row[0]), bool(int(row[1]))))
    return events


def lookup_ready(profile, elapsed_sec):
    # replay the recorded /localization/ready trace, looping once it ends.
    t = elapsed_sec % profile[-1][0]
    lo, hi = 0, len(profile) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if profile[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return profile[lo][1]


class FakeVehicle:
    def __init__(self):
        # HENES Broon T870 (개조 차량). 차체 제원 140 x 77.5 x 53.1 cm.
        # wheel_base / max_steer_deg 는 실차 설정과 같은 값을 쓴다
        # (waypoint_follower/config/t870_control.yaml: wheel_base 0.75,
        #  maximum_steering_angle_deg 20.0; vehicle_bridge.yaml: steer_cmd_max_deg 20.0).
        self.wheel_base = rospy.get_param("~wheel_base", 0.75)
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 20.0)

        # 2026-09-09 재보정: 15 km/h(0.0041667) 가정이 틀렸었다. speed_sweep.bag
        # (실내, raw 900 을 여러 초 유지)과 drift_0906 bag(raw 350) 두 점을
        # 엔코더(GPS 로 캘리브레이션된 0.01158 m/count)로 실측:
        #   raw 350 -> 1.088 m/s,  raw 900 -> 2.783 m/s
        # 두 점이 절편 0 인 한 직선에 정확히 놓인다(원점통과 -> 기동 문턱 없음,
        # 단 raw<350 은 미측정). v = raw * 0.003081.
        #   raw 1000 외삽 = 3.09 m/s = 11.1 km/h (스펙 15 km/h 와 다르다)
        # waypoint_follower/include/waypointFollower.h 의 RAW_TO_MPS 와
        # 반드시 같은 값을 써야 한다 - 다르면 controller 가 계산한
        # speed_curvature_alat 상한과 이 가짜 차량의 실제 속도가 어긋나서,
        # 컨트롤러 입장에서는 "속도를 다 맞춰 깎았는데" 시뮬 차량은 그보다
        # 빠르거나 느리게 움직이는 모순이 생긴다.
        self.speed_scale = rospy.get_param("~speed_scale", 0.003081)
        self.max_speed_mps = rospy.get_param("~max_speed_mps", 3.09)  # raw 1000 외삽
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)

        # 2026-09-08: 종방향 응답. 예전에는 v = raw * scale 을 그대로 써서
        # 명령 속도가 한 스텝에 그대로 반영됐다. drift_0906 bag 실측(500 raw/s
        # 램프 시절): 가속도 p05/p95 = -1.8 / +2.1 m/s^2, +2.1 은
        # drive_accel_raw_per_s(500)*speed_scale 과 일치 - 램프가 지배적이란 뜻.
        # 2026-09-09: drive_accel_raw_per_s 를 500->250 으로 낮췄다(정지 출발시
        # 경로 이탈 완화). max_accel_mps2 도 같은 비례로 낮춘다:
        #   250 raw/s * 0.003081 m/s/raw = 0.77 m/s^2
        # speed_lag_tau(추가 1차지연)는 이번에 다시 잰 적이 없어 그대로 둔다 -
        # 램프가 지배적이라던 원래 근거가 맞다면 영향이 작을 것이다.
        self.speed_lag_tau = rospy.get_param("~speed_lag_tau", 0.86)
        self.max_accel_mps2 = rospy.get_param("~max_accel_mps2", 0.77)
        self._actual_speed = 0.0
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        # 2026-09-07: replicate the real T870's steering-servo miscalibration
        # seen in drift_0906_12xx.bag (drift_analysis.png): pot_center was
        # configured at 450 but the measured straight-ahead ADC was ~560, a
        # ~110-count / ~6deg standing error, plus a noisy/oscillating actual-
        # vs-commanded steer angle. All default to 0 (no error) unless a
        # "drift" test launch overrides them.
        self.steer_bias_deg = rospy.get_param("~steer_bias_deg", 0.0)
        self.steer_gain = rospy.get_param("~steer_gain", 1.0)
        self.steer_noise_std_deg = rospy.get_param("~steer_noise_std_deg", 0.0)
        # 2026-09-08: 조향 응답 지연은 결함이 아니라 이 차의 물리 특성이라
        # 기본값으로 켠다. drift_0906 bag 의 cmd_steer 스텝 대비
        # steer_adc_raw 응답에서 tau = 0.55~0.58 s (n=25). 50 Hz 기준
        # alpha = 1 - exp(-dt/tau) = 0.036. 예전 값 0.35 는 tau 0.046 s 로
        # 실제보다 12 배 빨랐다. steer_pwm_slew_per_cycle(30) 이 20 Hz 로
        # 걸려 PWM 이 최대치에 닿는 데만 0.33 s 걸리는 것과도 부합한다.
        self.steer_lag_alpha = rospy.get_param("~steer_lag_alpha", 0.036)
        self._actual_steer_deg = 0.0

        # 2026-09-07: optional replay of a recorded /localization/ready trace
        # (e.g. exported from drift_0906_1302.bag -- real, flickering RTK
        # dropouts) instead of always publishing True. gnss_ready is left
        # constant True since require_raw_gnss_ready defaults to false and
        # only /localization/ready gates driving.
        gps_profile_path = rospy.get_param("~gps_dropout_profile", "")
        self.gps_profile = load_gps_dropout_profile(gps_profile_path) if gps_profile_path else None
        self._start_time = rospy.Time.now()

        # 2026-09-08: RTK 가 돌아오는 순간 EKF 가 그동안 쌓인 추측항법 오차를
        # 한 번에 보정하면서 위치가 튄다. 크기는 드롭아웃이 얼마나 길었는지에
        # 비례한다(그동안 쌓인 오차니까). drift_0906 bag 의 복구 엣지 68 개를
        # 재보면:
        #   드롭아웃 < 1 s (63 개)  -> 점프 중앙 ~1 cm (50 Hz 정상 이동 2.6 cm
        #                             보다도 작아 사실상 없음), 최대 17 cm
        #   드롭아웃 > 3 s ( 2 개)  -> 점프 33 cm
        #   점프/드롭아웃 길이 중앙값 = 0.009~0.016 m/s
        # 그래서 엣지마다 큰 값을 넣지 않고 지속시간에 비례시키되, 짧은
        # 깜빡임은 아예 건너뛴다. 드롭아웃 트레이스를 재생할 때만 동작한다.
        self.loc_drift_rate = rospy.get_param("~loc_recovery_drift_rate", 0.05)
        self.loc_jump_max = rospy.get_param("~loc_recovery_jump_max", 0.65)
        self.loc_min_dropout = rospy.get_param("~loc_recovery_min_dropout", 1.0)
        self._prev_ready = True
        self._dropout_start = None

        # 2026-09-09: GPS 드롭아웃과 무관한 EKF 지연(위 POS_LAG_* 주석 참고).
        #
        # 2026-09-14: 기본값 True -> False. 이 지연의 원인으로 지목했던
        # autoware_ekf.yaml 의 proc_stddev_yaw_c(0.03->0.10)/proc_stddev_wz_c
        # (0.2->0.60)가 그날 오후 이미 실제로 올라갔는데, 이 모델은 그 전
        # 데이터로 피팅한 채 안 고쳐진 상태로 계속 켜져 있었다. drive_
        # 20260911_235552.bag(수정 이후 실주행, state18)로 직접 재검증:
        #   v<2.274 m/s (n=196): CTE 평균 0.108m
        #   v>=2.274 m/s (n=684): CTE 평균 0.117m  (거의 최대속도 3.18m/s 포함)
        # 모델이 예측하는 "문턱 넘으면 지연 0.08s->0.42s 로 급증, CTE 악화"가
        # 실측에서 전혀 안 보였다 - 두 구간 CTE가 오차범위 안에서 같다.
        # 즉 이 모델이 시뮬레이션하던 버그 자체가 이미 고쳐졌다. 그런데도
        # 켜둔 채로 쓰면 이미 고친 구간에서 sim CTE 만 실차보다 부풀어
        # 보이는 역효과가 난다(2026-09-14 사용자 관측: "state18 고속
        # 직진에서 sim이 실차보다 CTE 큼" - 바로 이 모델 때문이었다).
        # proc_stddev 를 낮추는 회귀가 다시 생기면 새로 딴 bag 으로
        # 재피팅해서 pos_lag_enable:=true 로 켤 것 - 상수를 지우지 않고
        # 기본값만 끈다.
        self.pos_lag_enable = rospy.get_param("~pos_lag_enable", False)
        self.pos_lag_l0 = rospy.get_param("~pos_lag_l0", POS_LAG_L0)
        self.pos_lag_l1 = rospy.get_param("~pos_lag_l1", POS_LAG_L1)
        self.pos_lag_v0 = rospy.get_param("~pos_lag_v0", POS_LAG_V0)
        self.pos_lag_w = rospy.get_param("~pos_lag_w", POS_LAG_W)
        self._hist = []   # (t, x, y, yaw), rate_hz 로 채워짐
        self._sim_t = 0.0

        init_from_csv = rospy.get_param("~init_from_csv", "")
        if init_from_csv:
            x0, y0, yaw0 = read_initial_pose_from_csv(init_from_csv)
            rospy.loginfo("sim_vehicle: initial pose from %s -> (%.3f, %.3f, %.3f rad)",
                          init_from_csv, x0, y0, yaw0)
        else:
            x0 = rospy.get_param("~init_x", 0.0)
            y0 = rospy.get_param("~init_y", 0.0)
            yaw0 = rospy.get_param("~init_yaw", 0.0)

        self.lock = threading.Lock()
        self.x, self.y, self.yaw = x0, y0, yaw0
        self.cmd_speed_raw = 0
        self.cmd_steer_deg = 0.0
        self.cmd_gear = NEUTRAL

        rospy.Subscriber("/vehicle/cmd_speed", Int32, self._on_speed)
        rospy.Subscriber("/vehicle/cmd_steer", Float32, self._on_steer)
        rospy.Subscriber("/vehicle/cmd_gear", Int32, self._on_gear)

        self.odom_pub = rospy.Publisher("/odom_fused", Odometry, queue_size=10)
        self.gnss_ready_pub = rospy.Publisher(
            rospy.get_param("~gnss_ready_topic", "/vehicle1/c099/localization_ready"),
            Bool, queue_size=1)
        self.loc_ready_pub = rospy.Publisher(
            rospy.get_param("~localization_ready_topic", "/localization/ready"),
            Bool, queue_size=1)
        self.tf_bcaster = tf.TransformBroadcaster() if self.publish_tf else None

        self.last_t = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._step)
        rospy.Timer(rospy.Duration(0.1), self._publish_ready)

    def _pos_lag_sec(self, v):
        v = abs(v)
        return self.pos_lag_l0 + (self.pos_lag_l1 - self.pos_lag_l0) / (
            1.0 + math.exp(-(v - self.pos_lag_v0) / self.pos_lag_w))

    def _observed_pose(self):
        """실제(x,y,yaw) 대신 lag 만큼 과거 값을 컨트롤러용으로 낸다."""
        if not self.pos_lag_enable:
            return self.x, self.y, self.yaw
        lag = self._pos_lag_sec(self._actual_speed)
        target_t = self._sim_t - lag
        h = self._hist
        if not h or target_t <= h[0][0]:
            return self.x, self.y, self.yaw
        if target_t >= h[-1][0]:
            return h[-1][1], h[-1][2], h[-1][3]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= target_t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, yw0 = h[lo]
        t1, x1, y1, yw1 = h[hi]
        f = 0.0 if t1 <= t0 else (target_t - t0) / (t1 - t0)
        yaw = yw0 + f * math.atan2(math.sin(yw1 - yw0), math.cos(yw1 - yw0))
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw

    def _on_speed(self, msg):
        with self.lock:
            self.cmd_speed_raw = msg.data

    def _on_steer(self, msg):
        with self.lock:
            self.cmd_steer_deg = msg.data

    def _on_gear(self, msg):
        with self.lock:
            self.cmd_gear = msg.data

    def _publish_ready(self, _event):
        self.gnss_ready_pub.publish(Bool(data=True))
        if self.gps_profile is None:
            self.loc_ready_pub.publish(Bool(data=True))
            return

        elapsed = (rospy.Time.now() - self._start_time).to_sec()
        ready = lookup_ready(self.gps_profile, elapsed)

        # RTK 복구 순간(false -> true)에 EKF 보정 점프를 넣는다. 크기는 직전
        # 드롭아웃 길이에 비례하고, 방향은 그동안 쌓인 추측항법 오차가 어느
        # 쪽으로 났는지에 달렸으므로 무작위.
        if self._prev_ready and not ready:
            self._dropout_start = elapsed
        elif ready and not self._prev_ready and self._dropout_start is not None:
            outage = elapsed - self._dropout_start
            self._dropout_start = None
            if outage >= self.loc_min_dropout:
                mag = min(self.loc_jump_max, self.loc_drift_rate * outage)
                ang = random.uniform(-math.pi, math.pi)
                with self.lock:
                    self.x += mag * math.cos(ang)
                    self.y += mag * math.sin(ang)
                rospy.loginfo("sim_vehicle: RTK recovery jump %.2f m after %.1f s outage",
                              mag, outage)
        self._prev_ready = ready

        self.loc_ready_pub.publish(Bool(data=ready))

    def _step(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last_t).to_sec()
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate_hz

        with self.lock:
            raw_speed, steer_deg, gear = self.cmd_speed_raw, self.cmd_steer_deg, self.cmd_gear

        sign = {FORWARD: 1.0, BACKWARD: -1.0}.get(gear, 0.0)
        v_target = sign * abs(raw_speed) * self.speed_scale
        v_target = max(-self.max_speed_mps, min(self.max_speed_mps, v_target))

        # 종방향: 1차 지연으로 목표에 붙되, 그 과정의 가속도는 슬루 한계로
        # 자른다. 작은 스텝은 지연이, 큰 스텝은 슬루가 지배한다.
        if self.speed_lag_tau > 1e-6:
            alpha = 1.0 - math.exp(-dt / self.speed_lag_tau)
        else:
            alpha = 1.0
        dv = alpha * (v_target - self._actual_speed)
        dv_max = self.max_accel_mps2 * dt
        dv = max(-dv_max, min(dv_max, dv))
        self._actual_speed += dv
        v = self._actual_speed

        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))

        # steering-servo error model (see drift_bias params above): the real
        # actuator doesn't hit the commanded angle exactly -- it's biased,
        # scaled, noisy, and lags behind the command.
        target_actual = steer_deg * self.steer_gain + self.steer_bias_deg
        if self.steer_noise_std_deg > 0.0:
            target_actual += random.gauss(0.0, self.steer_noise_std_deg)
        self._actual_steer_deg += self.steer_lag_alpha * (target_actual - self._actual_steer_deg)
        actual_steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, self._actual_steer_deg))

        # new_path_tracking.cpp computes alpha as (cur_course - target_course), the
        # mirror of the textbook pure-pursuit sign, matching the LLC's hardware
        # convention (vehicle_bridge_node.py: "negative = left"). Negate here so
        # the bicycle model turns the way waypoint_follower actually intends.
        delta = -math.radians(actual_steer_deg)

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += (v / self.wheel_base) * math.tan(delta) * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self._sim_t += dt
        self._hist.append((self._sim_t, self.x, self.y, self.yaw))
        cutoff = self._sim_t - max(self.pos_lag_l1, 0.5) - 0.2
        while len(self._hist) > 2 and self._hist[1][0] < cutoff:
            self._hist.pop(0)
        ox, oy, oyaw = self._observed_pose()

        quat = tf.transformations.quaternion_from_euler(0, 0, oyaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = (v / self.wheel_base) * math.tan(delta)
        self.odom_pub.publish(odom)

        if self.tf_bcaster is not None:
            self.tf_bcaster.sendTransform(
                (ox, oy, 0.0), quat, now, self.base_frame, self.map_frame)


if __name__ == "__main__":
    rospy.init_node("fake_vehicle_node")
    FakeVehicle()
    rospy.spin()
