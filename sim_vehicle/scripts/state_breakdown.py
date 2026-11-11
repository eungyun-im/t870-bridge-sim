#!/usr/bin/env python3
"""어느 state 가 CTE 를 지배하는지 본다. 전체 배율로 다 올리는 대신
병목 state 만 따로 낮추면 같은 CTE 로 더 빨리 갈 수 있는지 판단하기 위한 것."""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_sweep
from full_sweep import RouteSim, segments, PTS, STATE_SPD

# 원본 사본. full_sweep.STATE_SPD 는 제자리 수정되므로 배율의 기준은
# 반드시 이 사본이어야 한다(안 그러면 호출마다 배율이 누적된다).
BASE_SPD = dict(STATE_SPD)

def per_state(alpha, mode, base, gain, mul):
    for k, v in BASE_SPD.items():
        full_sweep.STATE_SPD[k] = min(1000, int(round(v * mul)))
    acc = {}
    for s0, s1, st0 in segments():
        sim = RouteSim(PTS, s0, st0, alpha, base, gain, mode=mode)
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
            acc.setdefault(PTS[ci][2], []).append(best)
    return acc

if __name__ == '__main__':
    mode, base, gain, mul = 'fixed', 3.5, 0.0, float(sys.argv[1]) if len(sys.argv)>1 else 1.4
    print('mode=%s Ld=%.1f mul=%.1f  (state별 최악값, tau 3종 중 최악)'%(mode,base,mul))
    agg = {}
    for a in (0.0511, 0.0357, 0.0328):
        for st, cs in per_state(a, mode, base, gain, mul).items():
            m = max(cs)
            if m > agg.get(st, 0): agg[st] = m
    for st, m in sorted(agg.items(), key=lambda z: -z[1])[:12]:
        raw = min(1000, int(round(BASE_SPD.get(st,450)*mul)))
        print('  state %-3d max %.3f m   spd raw %d (%.2f m/s)' % (st, m, raw, raw*0.0041667))
