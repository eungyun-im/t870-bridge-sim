#!/usr/bin/env python3
"""2026-09-09: state 별 (Ld, 속도) 동시 최적화.

opt_speed.py 는 속도만 건드려서 한계 0.30 을 못 맞췄다. state 3/7/22/33 은
속도를 150 까지 내려도 max CTE 가 0.44 에서 안 내려간다. 그 오차는 지연이
아니라 Ld 에 의한 코너 컷팅(대략 Ld^2*kappa/8)이라 속도와 무관하기 때문이다.
속도로 지울 수 있는 건 지연에서 오는 발산 성분뿐이다.

방법: (Ld, 속도배율) 격자로 전 구간을 돌면서 state 별 max CTE 를 모은다.
각 state 에 대해 "한계를 지키는 것 중 가장 빠른" 칸을 고르면 그게 그
state 의 답이다. state 마다 조건이 섞이면 진입 상태가 달라지므로, 뽑은
표로 다시 달려 검증하고 어긋난 state 만 한 칸 물린다.
"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_sweep
from full_sweep import RouteSim, segments, PTS, STATE_SPD
from offline_sim import Sim

BASE_SPD = dict(STATE_SPD)
TAUS = [0.0511, 0.0357, 0.0328]
LDS = [2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
MULS = [0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.4]
STATES = sorted(BASE_SPD)


class TblSim(RouteSim):
    """state 별 Ld / 속도 표를 그대로 쓰는 Sim."""
    def __init__(self, *a, spd_tbl=None, ld_tbl=None, **kw):
        super().__init__(*a, mode='fixed', **kw)
        self.spd_tbl, self.ld_tbl = spd_tbl, ld_tbl

    def follower_tick(self):
        st = self.mission_state
        self.v_target = min(self.spd_tbl.get(st, 450) * 0.0041667, 4.1667)
        self.ld_base = self.ld_tbl.get(st, 3.5)
        self.ld_gain = 0.0
        Sim.follower_tick(self)


def measure(spd_tbl, ld_tbl):
    """state 별 max CTE (tau 최악) + 전체 통계"""
    ws, allc, steers = {}, [], []
    for a in TAUS:
        for s0, s1, st0 in segments():
            sim = TblSim(PTS, s0, st0, a, 3.5, 0.0, spd_tbl=spd_tbl, ld_tbl=ld_tbl)
            for t, x, y, sd, ci in sim.run(s1):
                if ci < s0 + 50 or ci > s1:
                    continue
                lo, hi = max(s0, ci-60), min(s1, ci+60)
                best = 1e9
                for i in range(lo, hi):
                    ax, ay = PTS[i][0], PTS[i][1]
                    ex, ey = PTS[i+1][0]-ax, PTS[i+1][1]-ay
                    L2 = ex*ex+ey*ey
                    u = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((x-ax)*ex+(y-ay)*ey)/L2))
                    d = math.hypot(x-(ax+ex*u), y-(ay+ey*u))
                    if d < best: best = d
                st = PTS[ci][2]
                if best > ws.get(st, 0): ws[st] = best
                allc.append(best); steers.append(sd)
    s = sorted(allc)
    return ws, dict(rms=math.sqrt(sum(c*c for c in allc)/len(allc)),
                    p95=s[int(0.95*len(s))], mx=s[-1],
                    act=50.0*sum(abs(b-a) for a, b in zip(steers, steers[1:]))/max(1, len(steers)-1))


def route_time(tbl):
    t = 0.0
    for s0, s1, _ in segments():
        for i in range(s0, s1):
            d = math.hypot(PTS[i+1][0]-PTS[i][0], PTS[i+1][1]-PTS[i][1])
            t += d / max(0.3, min(1000, tbl.get(PTS[i][2], 450))*0.0041667)
    return t


if __name__ == '__main__':
    # 1) (Ld, mul) 격자 조사
    grid = {}   # (ld, mul) -> {state: maxcte}
    for ld in LDS:
        for mul in MULS:
            spd = {k: min(1000, int(round(v*mul))) for k, v in BASE_SPD.items()}
            ldt = {k: ld for k in BASE_SPD}
            ws, _ = measure(spd, ldt)
            grid[(ld, mul)] = (ws, spd)
            print('  scan Ld %.1f mul %.1f  max %.3f' % (ld, mul, max(ws.values())), flush=True)

    for limit in (0.25, 0.30, 0.35, 0.40, 0.50):
        # 2) state 마다 한계를 지키는 것 중 가장 빠른 칸
        spd_tbl, ld_tbl = {}, {}
        for st in STATES:
            best = None
            for (ld, mul), (ws, spd) in grid.items():
                if ws.get(st, 9) <= limit:
                    v = spd[st]
                    if best is None or v > best[0]:
                        best = (v, ld)
            if best is None:   # 어떤 칸도 한계를 못 지키면 가장 작은 CTE 칸
                k = min(grid, key=lambda k: grid[k][0].get(st, 9))
                best = (grid[k][1][st], k[0])
            spd_tbl[st], ld_tbl[st] = best[0], best[1]

        # 3) 조합한 표로 검증 + 어긋난 state 물리기
        for _ in range(6):
            ws, stt = measure(spd_tbl, ld_tbl)
            bad = [s for s in STATES if ws.get(s, 0) > limit]
            if not bad:
                break
            for s in bad:
                spd_tbl[s] = max(150, int(spd_tbl[s]*0.88))
        print()
        print('한계 %.2f m -> 주파 %.0fs  rms %.3f p95 %.3f max %.3f act %.1f  (미달 state %s)'
              % (limit, route_time(spd_tbl), stt['rms'], stt['p95'], stt['mx'], stt['act'], bad))
        print('  spd:', ' '.join('%d:%d' % (k, spd_tbl[k]) for k in STATES))
        print('  Ld :', ' '.join('%d:%.1f' % (k, ld_tbl[k]) for k in STATES), flush=True)
