#!/usr/bin/env python3
"""ROS 없이 도는 폐루프 시뮬레이터. Ld/alat 스윕 + 실주행 대조용.

2026-09-09 전면 재작성. 이전 버전(SPEED_TAU=0.86, MAX_ACCEL=2.1 1차지연 모델,
완벽한 위치 피드백)은 검증 없이 쓰이다가 실측과 크게 어긋났다 - 예측 max CTE
0.419m, 실주행(run3) 2.84m. 원인은 두 가지였다:
  1) raw->m/s 가정이 틀렸다 (15km/h 가정, 실측은 raw 900=2.77 m/s=10km/h)
  2) 가장 큰 문제: 컨트롤러에 "완벽한" 위치를 그대로 먹였다. 실제로는
     GNSS(5Hz)+EKF 결합 출력이 진짜 위치보다 뒤처지는데, 그 지연이 속도에
     따라 급격히 커진다(국민대 실주행 7회 실측):
         v <= ~2.0 m/s  ->  EKF 지연  20~160 ms  (IMU 대비 교차상관 0.84~0.95)
         v >= ~2.3 m/s  ->  EKF 지연 380~420 ms  (교차상관 0.42~0.50, 사실상 발산)
     이 임계를 안 넣으면 시뮬은 "빠를수록 코너를 못 돈다"만 볼 뿐, 실제로
     CTE 를 터뜨리는 지배적 원인(고속에서 위치 추정 자체가 늦게 온다)을
     놓친다. 아래 OBS_LAG_* 가 이 관계를 흉내낸 것이다 - run3/4/5/6/7/8/9
     7개 bag, 6개 유효점(run3 는 기어 NEUTRAL 33% 라 정상 순항이 아니라 제외)
     으로 맞춘 계단형 로지스틱이다. 점이 6개뿐이고 그중 두 점(run8,9)은
     pose_smoothing_steps 가 나머지(5)와 다른 값(2)일 때 찍혀서 확인되지
     않은 교란요인이 섞여 있다. "이 alat 이 CTE 0.09m 을 낸다" 처럼 절대값을
     믿지 말고, "이 alat 이 임계 2.27 m/s 를 넘기는 구간 비율" 같은 상대
     비교/스크리닝 용도로 쓸 것. 실측 없이 alat 를 확정하지 말 것.

옮긴 대상(원본, 이 파일을 고칠 때 반드시 같이 볼 것):
  waypoint_maker/src/new_waypoint_loader.cpp
      getClosestWaypoint()  : 순차 탐색창 + state 차 0/1 후보 조건
      getFinalWaypoint()    : 최근접점부터 100점 롤링 창, 10 Hz
  waypoint_follower/src/new_process.cpp
      곡률 기반 속도 상한(2026-09-09 추가): v_limit=sqrt(alat/|rk|),
      preview 구간 |rk| 최대, speed_multiplier, speed_multiplier_cap
  waypoint_follower/src/new_path_tracking.cpp
      pure_pursuit() : 목표 탐색, 종점 가드, alpha, final_ld 클램프(2~20),
                       조향식. Ld = lookahead_speed_base + gain*|v|
  vehicle_bridge/scripts/vehicle_bridge_node.py
      _rate_limit() : raw 를 drive_accel/decel_raw_per_s 로 램프, 20 Hz
                       (control_rate). RAW_TO_MPS 는 실측(2026-09-09):
                       raw 350 -> 1.088 m/s, raw 900 -> 2.783 m/s.

주의: 조향 응답(steer_lag_alpha)은 이번 실측 대상이 아니었다(포텐셔미터
피드백을 안 남긴 bag 이 많았다). 기본값은 눈대중이다. 다음 실주행에
/vehicle/steer_adc_raw 를 녹화하면 그걸로 캘리브레이션할 것.
"""
import argparse
import csv
import math
import sys

WHEEL_BASE = 0.75
MAX_STEER = 20.0
RAW_TO_MPS = 0.003081     # 실측: v = raw * 0.003081 (raw 350->1.088, raw 900->2.783)
FINAL_SIZE = 100          # 로더 롤링 창
MAX_INDEX_STEP = 15
MAX_SEARCH_DIST = 10.0
MIN_TARGET_DIST = 1.5     # 종점 가드 가상 목표
TARGET_STATE_LOCK = {12, 13, 28, 29, 30, 31}

DT = 0.005                # 200Hz 기준 클럭. 50/20/10 Hz 가 정수배로 떨어진다.
N_VEH, N_FOL, N_LOAD = 4, 10, 20   # 50 / 20 / 10 Hz

# 2026-09-09 실측 위치-지연 모델. lag(v) = L0 + (L1-L0)/(1+exp(-(v-v0)/w))
# 데이터: run9(1.40,20) run8(1.50,160) run7(1.60,100) run5(1.86,40)
#         run6(2.31,380) run4(2.57,420)  [run3(0.70,340) 는 기어 NEUTRAL 33% 라 제외]
# scipy curve_fit 결과: L0=80, L1=420, v0=2.274, w=0.020 (RMS 잔차 44.7 ms)
OBS_LAG_L0 = 0.080
OBS_LAG_L1 = 0.420
OBS_LAG_V0 = 2.274
OBS_LAG_W = 0.020


def default_lag_model(v):
    """|v| -> 관측 지연 [s]. 실측 6점 로지스틱 피팅. --no-lag 로 끌 수 있다."""
    v = abs(v)
    return OBS_LAG_L0 + (OBS_LAG_L1 - OBS_LAG_L0) / (1.0 + math.exp(-(v - OBS_LAG_V0) / OBS_LAG_W))


def load_csv(path):
    """idx,x,y,state,ryaw,rk  6열. rk 없는 구형 파일은 0으로 채운다."""
    pts = []
    for r in csv.reader(open(path)):
        rk = float(r[5]) if len(r) > 5 else 0.0
        pts.append((float(r[1]), float(r[2]), int(r[3]), float(r[4]), rk))
    return pts


class Sim:
    def __init__(self, pts, start_idx, state_inspection,
                 steer_lag_alpha=0.15, ld_base=2.0, ld_gain=0.4,
                 ld_min=2.0, ld_max=5.0,
                 spd_state=850.0, speed_multiplier=1.0, speed_cap=900.0,
                 curvature_alat=0.0, curvature_preview=3.0, curvature_min_raw=150.0,
                 accel_raw_per_s=250.0, decel_raw_per_s=2000.0,
                 lag_model=default_lag_model,
                 legacy_table=None, legacy_default=(4.5, 450.0)):
        """legacy_table: {mission_state: (ld, spd_raw)} - new_process.cpp 의 case
        switch 를 그대로 옮긴 것. 주어지면 lookahead_speed_base/gain, spd_state,
        speed_curvature_* 를 전부 무시하고 이 표만 쓴다(구 로직 재현).
        legacy_default: 표에 없는 state(주차 등, 별도 오버라이드 경로) 에 쓸 값.
        """
        self.legacy_table = legacy_table
        self.legacy_default = legacy_default
        self.p = pts
        self.n = len(pts)
        self.s = self._arclength()
        self.true_x, self.true_y = pts[start_idx][0], pts[start_idx][1]
        self.true_yaw = pts[start_idx][3]
        self.true_v = 0.0
        self.actual_steer = 0.0
        self.cmd_steer = 0.0
        self.steer_lag = steer_lag_alpha
        self.ld_base, self.ld_gain = ld_base, ld_gain
        self.ld_min, self.ld_max = ld_min, ld_max

        self.spd_state = spd_state
        self.speed_multiplier = speed_multiplier
        self.speed_cap = speed_cap
        self.curvature_alat = curvature_alat
        self.curvature_preview = curvature_preview
        self.curvature_min_raw = curvature_min_raw
        self.accel_raw = accel_raw_per_s
        self.decel_raw = decel_raw_per_s
        self.current_raw = 0.0

        self.lag_model = lag_model
        self.hist = []            # (t, x, y, yaw), 50Hz 로 채워짐
        self.t = 0.0

        # observed(EKF 가 낸다고 가정하는) 상태 - 컨트롤러는 이것만 본다
        self.x, self.y, self.yaw = self.true_x, self.true_y, self.true_yaw

        # loader state
        self.closest = 0
        self.has_closest = False
        self.mission_state = state_inspection
        self.window = []
        self.state_index = self._build_state_index()

    def _arclength(self):
        s = [0.0]
        for i in range(1, self.n):
            s.append(s[-1] + math.hypot(self.p[i][0] - self.p[i - 1][0],
                                         self.p[i][1] - self.p[i - 1][1]))
        return s

    def _build_state_index(self):
        """state 값(0..max) -> 그 state 가 이 경로 배열에서 처음 나오는 인덱스.

        2026-09-09 버그 수정: 이전엔 '몇 번째 블록인가'로 표를 만들고
        loader_tick() 에서 그걸 '실제 state 값'으로 바로 인덱싱했다. 전체
        728m 코스(state 0부터 시작, 블록 순서 == state 값)에서는 우연히
        맞았지만, state 14부터 시작하는 부분 구간(seg_fwd2)처럼 첫 state 가
        0이 아니면 완전히 어긋나 loader 가 엉뚱한 인덱스에서 탐색을 시작해
        closest 가 영영 못 움직였다(state 진행 자체가 멈춤).

        real new_waypoint_loader.cpp 의 state_index_ 는 state 값으로 직접
        인덱싱되므로(주석의 "state 0 시작은 5"가 그 증거), 여기서도 값 기준
        표를 만든다. 이 경로에 없는 state 는 직전에 확인된 인덱스로 채운다
        (실제로 그 state 를 찾을 일이 없으면 안 쓰이는 값이라 문제없다).
        """
        max_state = int(max(q[2] for q in self.p))
        tbl = [0] * (max_state + 1)
        cur = -1
        for i, q in enumerate(self.p):
            s = int(q[2])
            if s != cur:
                for j in range(cur + 1, s + 1):
                    if j <= max_state:
                        tbl[j] = i
                cur = s
        if 0 <= max_state:
            tbl[0] = min(5, len(self.p) - 1)  # 원 코드의 "state 0 은 5부터" 예외
        return tbl

    # ---- 관측 지연: history 버퍼에서 (now - lag) 시점 값을 보간 --------------
    def _update_observed(self):
        lag = self.lag_model(self.true_v)
        target_t = self.t - lag
        h = self.hist
        if not h or target_t <= h[0][0]:
            self.x, self.y, self.yaw = self.true_x, self.true_y, self.true_yaw
            return
        if target_t >= h[-1][0]:
            self.x, self.y, self.yaw = h[-1][1], h[-1][2], h[-1][3]
            return
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
        self.x = x0 + f * (x1 - x0)
        self.y = y0 + f * (y1 - y0)
        dyaw = math.atan2(math.sin(yw1 - yw0), math.cos(yw1 - yw0))
        self.yaw = yw0 + f * dyaw

    # ---- loader (10 Hz) --------------------------------------------------
    def loader_tick(self):
        self._update_observed()
        tbl = self.state_index
        si = min(max(self.mission_state, 0), len(tbl) - 1)
        t_index = max(0, tbl[si] - 4)

        if self.has_closest:
            begin, end = self.closest, min(self.closest + MAX_INDEX_STEP + 1, self.n)
        else:
            begin, end = t_index, self.n

        for attempt in range(2):
            best_d, best_i = MAX_SEARCH_DIST, -1
            for i in range(begin, end):
                dx = self.x - self.p[i][0]
                dy = self.y - self.p[i][1]
                d = math.hypot(dx, dy)
                ds = self.p[i][2] - self.mission_state
                if d < MAX_SEARCH_DIST and ds in (0, 1):
                    if d < best_d:
                        best_d, best_i = d, i
                if d > MAX_SEARCH_DIST and ds > 2:
                    break
            if best_i >= 0 or not self.has_closest:
                break
            begin, end = t_index, self.n

        if best_i >= 0:
            if best_i < self.closest:
                best_i = self.closest
            self.closest = best_i
            self.has_closest = True

        end_w = min(self.closest + FINAL_SIZE, self.n)
        self.window = self.p[self.closest:end_w]
        if self.window:
            self.mission_state = self.window[0][2]

    # ---- 곡률 기반 속도 상한 + 배율/cap (new_process.cpp, 20 Hz) -----------
    def speed_tick(self):
        if self.legacy_table is not None:
            ld, eff = self.legacy_table.get(self.mission_state, self.legacy_default)
            self._legacy_ld = ld
            self._target_raw = eff
            return

        eff = self.spd_state
        if self.speed_multiplier > 0.0 and self.speed_multiplier != 1.0:
            eff = min(eff * self.speed_multiplier, self.speed_cap)
        else:
            eff = min(eff, self.speed_cap)

        if self.curvature_alat > 0.0 and eff > 0.0 and self.window:
            travelled = 0.0
            max_rk = 0.0
            i0 = self.closest
            s0 = self.s[i0] if i0 < self.n else self.s[-1]
            for j, q in enumerate(self.window):
                idx = self.closest + j
                rk = abs(q[4])
                if rk > max_rk:
                    max_rk = rk
                if idx + 1 < self.n:
                    travelled = self.s[idx] - s0
                if travelled >= self.curvature_preview:
                    break
            if max_rk > 1e-6:
                v_limit = math.sqrt(self.curvature_alat / max_rk)
                raw_limit = v_limit / RAW_TO_MPS
                if raw_limit < self.curvature_min_raw:
                    raw_limit = self.curvature_min_raw
                if eff > raw_limit:
                    eff = raw_limit
        self._target_raw = eff

    # ---- follower (20 Hz) -------------------------------------------------
    def follower_tick(self):
        w = self.window
        if not w:
            self.cmd_steer = 0.0
            return
        if self.legacy_table is not None:
            ld = getattr(self, '_legacy_ld', self.legacy_default[0])
        else:
            ld = self.ld_base + self.ld_gain * abs(self.true_v)
            ld = max(self.ld_min, min(self.ld_max, ld))

        lock = self.mission_state in TARGET_STATE_LOCK
        state_last = -1
        target = -1
        final_ld = 0.0
        for i, q in enumerate(w):
            if lock:
                if q[2] != self.mission_state:
                    if state_last >= 0:
                        break
                    continue
                state_last = i
            d = math.hypot(self.x - q[0], self.y - q[1])
            if d > ld:
                target, final_ld = i, d
                break

        vx = vy = 0.0
        virtual = False
        if target < 0:
            target = state_last if (lock and state_last >= 0) else len(w) - 1
            final_ld = math.hypot(self.x - w[target][0], self.y - w[target][1])
            if final_ld < MIN_TARGET_DIST:
                ext = MIN_TARGET_DIST - final_ld
                th = w[target][3]
                vx = w[target][0] + math.cos(th) * ext
                vy = w[target][1] + math.sin(th) * ext
                final_ld = math.hypot(vx - self.x, vy - self.y)
                virtual = True

        tx = vx if virtual else w[target][0]
        ty = vy if virtual else w[target][1]
        temp_theta = math.degrees(math.atan2(ty - self.y, tx - self.x))
        deg_alpha = math.degrees(self.yaw) - temp_theta
        while deg_alpha > 180.0:
            deg_alpha -= 360.0
        while deg_alpha < -180.0:
            deg_alpha += 360.0
        alpha = math.radians(deg_alpha)

        if final_ld > 20.0:
            final_ld = 4.0
        if final_ld < 2.0:
            final_ld = 2.0

        steer = math.degrees(math.atan2(2.0 * WHEEL_BASE * math.sin(alpha) / final_ld, 1.0))
        self.cmd_steer = max(-MAX_STEER, min(MAX_STEER, steer))

    # ---- 구동 raw 램프 (vehicle_bridge _rate_limit, 20 Hz) -----------------
    def bridge_tick(self, dt):
        target = getattr(self, '_target_raw', 0.0)
        if target > self.current_raw:
            self.current_raw = min(target, self.current_raw + self.accel_raw * dt)
        else:
            self.current_raw = max(target, self.current_raw - self.decel_raw * dt)

    # ---- 차량 물리 (50 Hz) --------------------------------------------------
    def vehicle_tick(self, dt):
        self.true_v = self.current_raw * RAW_TO_MPS

        cmd = max(-MAX_STEER, min(MAX_STEER, self.cmd_steer))
        self.actual_steer += self.steer_lag * (cmd - self.actual_steer)
        act = max(-MAX_STEER, min(MAX_STEER, self.actual_steer))
        delta = -math.radians(act)

        self.true_x += self.true_v * math.cos(self.true_yaw) * dt
        self.true_y += self.true_v * math.sin(self.true_yaw) * dt
        self.true_yaw += (self.true_v / WHEEL_BASE) * math.tan(delta) * dt
        self.true_yaw = math.atan2(math.sin(self.true_yaw), math.cos(self.true_yaw))

        self.t += dt
        self.hist.append((self.t, self.true_x, self.true_y, self.true_yaw))
        # 필요한 만큼만 남긴다 (최대 지연 0.5s + 여유)
        cutoff = self.t - 0.8
        while len(self.hist) > 2 and self.hist[1][0] < cutoff:
            self.hist.pop(0)

    def run(self, stop_idx, max_time=400.0):
        """closest 가 stop_idx 를 넘을 때까지 돌린다.
        (t, true_x, true_y, obs_x, obs_y, cmd_steer, true_v, closest) 로그.
        """
        log = []
        k = 0
        t = 0.0
        while t < max_time:
            if k % N_LOAD == 0:
                self.loader_tick()
            if k % N_FOL == 0:
                self.speed_tick()
                self.follower_tick()
                self.bridge_tick(DT * N_FOL)
            if k % N_VEH == 0:
                self.vehicle_tick(DT * N_VEH)
                log.append((t, self.true_x, self.true_y, self.x, self.y,
                            self.cmd_steer, self.true_v, self.closest))
            if self.closest >= stop_idx:
                break
            k += 1
            t += DT
        return log


def cte_stats(log, pts, tail_margin=10):
    """CTE 통계. 경로 끝 tail_margin 점 구간은 뺀다.

    run5_path.csv 처럼 bag 에서 뽑은 경로는 대개 닫힌 루프가 아니라(주행이
    끝난 시점까지만 기록됨) 열린 구간이다. 시뮬은 마지막 점까지 달리므로,
    실제로는 존재하지 않는 "경로 끝"에서 종점 가드가 가상 목표를 만들다
    차가 지나쳐 CTE 가 크게 튄다 - 실차 성능과 무관한 경계 인공물이다.
    """
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cutoff = max(0, len(pts) - 1 - tail_margin)
    errs = []
    for _, tx, ty, _ox, _oy, _st, _v, closest in log:
        if closest >= cutoff:
            continue
        best = min(math.hypot(tx - x, ty - y) for x, y in zip(xs, ys))
        errs.append(best)
    errs.sort()
    n = len(errs)
    def pct(q):
        return errs[min(int(q * n), n - 1)] if n else float('nan')
    return dict(n=n, median=pct(0.5), p75=pct(0.75), p90=pct(0.90), p95=pct(0.95),
                max=errs[-1] if errs else float('nan'))


LEGACY_TABLE = {   # new_process.cpp switch(mission_state) 를 그대로 옮김 (2026-09-09 채집)
    0: (6.0, 850), 1: (4.5, 750), 2: (6.5, 850), 3: (4.5, 450), 4: (5.5, 550),
    5: (4.5, 450), 6: (6.0, 600), 7: (7.5, 750), 8: (4.5, 450), 9: (4.5, 450),
    10: (7.5, 750), 11: (6.5, 650), 15: (6.5, 650), 16: (4.5, 450), 17: (4.5, 450),
    18: (6.5, 650), 19: (4.5, 450), 20: (6.5, 350), 21: (7.5, 750), 22: (3.5, 350),
    23: (6.5, 650), 24: (2.5, 250), 25: (3.5, 350), 26: (8.0, 800), 27: (3.5, 350),
    28: (7.0, 700), 29: (5.0, 500), 33: (6.5, 650), 34: (4.5, 500),
}   # 12,13,14,30,31,32 (주차 진입/체크/탈출) 는 parking_speed_override_ 로 별도
    # 처리되어 이 switch 에 없다 - legacy_default 로 대체한다.


def make_sim(pts, args, no_lag=False):
    lag_model = (lambda v: 0.0) if no_lag else default_lag_model
    legacy = LEGACY_TABLE if getattr(args, 'legacy', False) else None
    return Sim(pts, 0, 0,
               steer_lag_alpha=args.steer_lag,
               ld_base=args.ld_base, ld_gain=args.ld_gain,
               ld_min=args.ld_min, ld_max=args.ld_max,
               spd_state=args.spd, speed_multiplier=args.mult, speed_cap=args.cap,
               curvature_alat=args.alat, curvature_preview=args.preview,
               curvature_min_raw=args.min_raw,
               accel_raw_per_s=args.accel_raw, decel_raw_per_s=args.decel_raw,
               lag_model=lag_model, legacy_table=legacy)


def cmd_sweep(args):
    pts = load_csv(args.path)
    L = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            for i in range(1, len(pts)))
    print(f"경로: {args.path}  {len(pts)}점  {L:.1f} m")
    print(f"{'alat':>6} {'랩s':>7} {'v평균':>7} | {'CTE중앙':>8} {'75%':>7} {'90%':>7} {'max':>7}")
    for alat in args.alat_list:
        args.alat = alat
        sim = make_sim(pts, args, no_lag=args.no_lag)
        stop = min(len(pts) - 1, args.loops and int(len(pts) * 0.98) or len(pts) - 1)
        log = sim.run(stop, max_time=args.max_time)
        if not log:
            print(f"{alat:6.2f}  (완주 못함)")
            continue
        st = cte_stats(log, pts)
        lap_t = log[-1][0]
        vavg = (L / lap_t) if lap_t > 0 else 0.0
        print(f"{alat:6.2f} {lap_t:7.1f} {vavg:7.2f} | {st['median']:8.2f} {st['p75']:7.2f}"
              f" {st['p90']:7.2f} {st['max']:7.2f}")


def cmd_validate(args):
    pts = load_csv(args.path)
    sim = make_sim(pts, args, no_lag=args.no_lag)
    stop = len(pts) - 1
    log = sim.run(stop, max_time=args.max_time)
    st = cte_stats(log, pts)
    print(f"시뮬 예측 (alat={args.alat}, spd={args.spd}, mult={args.mult}, "
          f"lag_model={'off' if args.no_lag else 'on'}):")
    print(f"  CTE 중앙 {st['median']:.2f}  75% {st['p75']:.2f}  90% {st['p90']:.2f}"
          f"  max {st['max']:.2f}  (n={st['n']})")
    print(f"  랩타임 {log[-1][0]:.1f} s" if log else "  (완주 못함)")
    if args.real:
        print(f"\n실측 대조({args.real}): CTE 중앙 {args.real[0]:.2f}  90% {args.real[1]:.2f}"
              f"  max {args.real[2]:.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ld-base', type=float, default=2.0)
    ap.add_argument('--ld-gain', type=float, default=0.4)
    ap.add_argument('--ld-min', type=float, default=2.0)
    ap.add_argument('--ld-max', type=float, default=5.0)
    ap.add_argument('--spd', type=float, default=850.0, help='spd_state_ 기본값(raw)')
    ap.add_argument('--mult', type=float, default=1.0, help='speed_multiplier')
    ap.add_argument('--cap', type=float, default=900.0, help='speed_multiplier_cap (raw)')
    ap.add_argument('--preview', type=float, default=3.0, help='speed_curvature_preview [m]')
    ap.add_argument('--min-raw', type=float, default=150.0, dest='min_raw')
    ap.add_argument('--accel-raw', type=float, default=250.0, dest='accel_raw')
    ap.add_argument('--decel-raw', type=float, default=2000.0, dest='decel_raw')
    ap.add_argument('--steer-lag', type=float, default=0.15, dest='steer_lag',
                     help='조향 1차지연 계수(50Hz틱당). 미검증 기본값 - 다음 실주행에서 '
                          '/vehicle/steer_adc_raw 녹화 후 다시 잡을 것')
    ap.add_argument('--no-lag', action='store_true',
                     help='위치 관측지연 모델을 끈다(구버전과 동일한 이상적 피드백)')
    ap.add_argument('--legacy', action='store_true',
                     help='new_process.cpp 의 state별 하드코딩 Ld/speed 표를 쓴다 '
                          '(적응형 lookahead_speed_*/speed_curvature_* 전부 무시)')
    ap.add_argument('--max-time', type=float, default=120.0, dest='max_time')

    sub = ap.add_subparsers(dest='cmd', required=True)

    sp = sub.add_parser('sweep', help='경로 하나에 대해 alat 여러 값을 스윕')
    sp.add_argument('path')
    sp.add_argument('--alat', dest='alat_list', type=float, nargs='+',
                     default=[0.3, 0.4, 0.5, 0.6, 0.75, 1.0])
    sp.add_argument('--loops', type=int, default=1)
    sp.set_defaults(func=cmd_sweep)

    vp = sub.add_parser('validate', help='실측 bag 대비 예측 CTE 를 비교')
    vp.add_argument('path')
    vp.add_argument('--alat', type=float, default=0.5)
    vp.add_argument('--real', type=float, nargs=3, metavar=('MED', 'P90', 'MAX'),
                     help='실측 CTE 중앙/90%%/max 를 넣으면 나란히 출력')
    vp.set_defaults(func=cmd_validate)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
