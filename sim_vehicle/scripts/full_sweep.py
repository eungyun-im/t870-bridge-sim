#!/usr/bin/env python3
"""2026-09-08: 전 구간 Ld 스윕. offline_sim.py 위에서 돈다.

speed_override 로 속도를 한 값에 묶으면 Ld = base + gain*v 에서 gain 과
base 가 서로 구분되지 않는다(v 가 상수라 gain*v 도 상수다). gain 을 제대로
보려면 state 마다 속도가 다른 실제 주행을 그대로 달려야 한다. 그래서
new_process.cpp 의 미션 switch 가 정한 state 별 spd_state_ / lookahead_dist_
를 여기에 옮겨 두고 그대로 쓴다.

세 가지 Ld 모드를 같은 코스에서 비교한다.
  table : 지금 실차가 쓰는 state 별 고정값 (baseline)
  fixed : 전 구간 한 값
  speed : base + gain * |v|

주차 state(12/13/14, 30/31/32)는 뺀다. 후진과 슬롯 판정 상태기계가 얽혀
있어 Ld 튜닝 대상이 아니고, target_state_lock 으로 목표점이 그 구간 안에
묶여 있어 Ld 를 바꿔도 거동이 거의 안 변한다. 그래서 코스를 주차 앞뒤로
세 토막 내어 각각 달린다.
"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offline_sim import Sim, load_csv, SPEED_SCALE, MAX_SPEED

CSV = '/home/kuuve/catkin_ws/src/data_시흥/0.csv'
PTS = load_csv(CSV)

# new_process.cpp 미션 switch 의 각 case 첫 지정값.
# 뒤따르는 조건부 값(신호등/정지 램프)은 추종 성능이 아니라 정지 동작이라 뺀다.
STATE_SPD = {0:850, 1:750, 2:850, 3:450, 4:550, 5:450, 6:600, 7:750, 8:450,
             9:450, 10:750, 11:650, 15:650, 16:450, 17:450, 18:650, 19:450,
             20:350, 21:750, 22:350, 23:650, 24:250, 25:350, 26:800, 27:350,
             28:700, 29:500, 33:650, 34:500}
STATE_LD  = {0:4.0, 1:4.5, 2:6.5, 3:4.5, 4:5.5, 5:4.5, 6:6.0, 7:7.5, 8:4.5,
             9:4.5, 10:7.5, 11:6.5, 15:6.5, 16:4.5, 17:4.5, 18:6.5, 19:4.5,
             20:6.5, 21:7.5, 22:3.5, 23:6.5, 24:2.5, 25:3.5, 26:8.0, 27:3.5,
             28:7.0, 29:5.0, 33:6.5, 34:4.5}

PARKING = {12, 13, 14, 30, 31, 32}


def state_start(s):
    for i, q in enumerate(PTS):
        if q[2] == s:
            return i
    return None


def segments():
    """주차 구간을 뺀 연속 토막들. (시작 idx, 끝 idx, 시작 state)"""
    segs, cur_start = [], None
    for i, q in enumerate(PTS):
        park = q[2] in PARKING
        if not park and cur_start is None:
            cur_start = i
        elif park and cur_start is not None:
            segs.append((cur_start, i - 1, PTS[cur_start][2]))
            cur_start = None
    if cur_start is not None:
        segs.append((cur_start, len(PTS) - 1, PTS[cur_start][2]))
    return segs


class RouteSim(Sim):
    """state 별 속도 테이블을 쓰고, Ld 모드를 고를 수 있는 Sim."""
    def __init__(self, *a, mode='speed', **kw):
        super().__init__(*a, **kw)
        self.mode = mode

    def follower_tick(self):
        st = self.mission_state
        # 속도 목표를 state 테이블에서 가져온다 (fake_vehicle 은 명령을 그대로 받는다)
        raw = STATE_SPD.get(st, 450)
        self.v_target = min(raw * SPEED_SCALE, MAX_SPEED)
        if self.mode == 'table':
            self.ld_base, self.ld_gain = STATE_LD.get(st, 4.5), 0.0
        super().follower_tick()


def run_route(alpha, mode, base, gain, settle_m=15.0):
    """모든 토막을 달리고 전체 CTE 통계를 낸다."""
    ctes, steers, worst = [], [], []
    for s0, s1, st0 in segments():
        sim = RouteSim(PTS, s0, st0, alpha, base, gain, mode=mode)
        log = sim.run(s1)
        skip = int(settle_m / 0.1)     # 토막 시작 정착 구간은 뺀다
        seg_c = []
        for t, x, y, sd, ci in log:
            if ci < s0 + skip or ci > s1:
                continue
            lo = max(s0, ci - 60)
            hi = min(s1, ci + 60)
            best = 1e9
            for i in range(lo, hi):
                ax, ay = PTS[i][0], PTS[i][1]
                bx, by = PTS[i+1][0], PTS[i+1][1]
                ex, ey = bx - ax, by - ay
                L2 = ex*ex + ey*ey
                u = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((x-ax)*ex + (y-ay)*ey)/L2))
                d = math.hypot(x - (ax + ex*u), y - (ay + ey*u))
                if d < best:
                    best = d
            seg_c.append(best)
            steers.append(sd)
        ctes += seg_c
        if seg_c:
            worst.append((max(seg_c), st0))
    if not ctes:
        return None
    s = sorted(ctes)
    act = 50.0*sum(abs(b-a) for a, b in zip(steers, steers[1:]))/max(1, len(steers)-1)
    return dict(n=len(ctes), p50=s[len(s)//2], p95=s[int(0.95*len(s))], mx=s[-1],
                rms=math.sqrt(sum(c*c for c in ctes)/len(ctes)), act=act,
                worst=max(worst))


if __name__ == '__main__':
    print('구간:', segments())
    for a, tau in ((0.0511, 0.38), (0.0357, 0.55), (0.0328, 0.60)):
        r = run_route(a, 'table', 0, 0)
        print('tau %.2f  table(baseline)  rms %.3f p95 %.3f max %.3f act %.1f  최악토막 %s'
              % (tau, r['rms'], r['p95'], r['mx'], r['act'], r['worst']))
