#!/usr/bin/env python3
"""2026-09-09: "CTE 한계 안에서 가장 빠른" state 별 속도표를 찾는다.

전체에 같은 배율을 곱하면 여유 있는 직선까지 같이 눌리고, 빡빡한 코너는
같이 튄다(state 15/7/3/34 가 전체 max 를 지배). state 마다 따로 올린다.

방법: 좌표 상승. 매 회 전 구간을 달려 state 별 max CTE 를 재고,
  - 한계보다 여유 있으면 그 state 속도를 올리고
  - 한계를 넘으면 내린다.
tau 0.38/0.55/0.60 중 최악값을 쓴다(실차 tau 를 모르므로).
"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_sweep
from full_sweep import RouteSim, segments, PTS, STATE_SPD

BASE_SPD = dict(STATE_SPD)
TAUS = [0.0511, 0.0357, 0.0328]
RAW_MIN, RAW_MAX = 150, 1000


def run_once(alpha, mode, base, gain):
    """state 별 CTE 목록과 전체 목록"""
    per, allc, steers = {}, [], []
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
            per.setdefault(PTS[ci][2], []).append(best)
            allc.append(best)
            steers.append(sd)
    return per, allc, steers


def evaluate(tbl, mode, base, gain):
    full_sweep.STATE_SPD.clear(); full_sweep.STATE_SPD.update(tbl)
    worst_state, allc, steers = {}, [], []
    for a in TAUS:
        per, ac, st = run_once(a, mode, base, gain)
        for s, cs in per.items():
            m = max(cs)
            if m > worst_state.get(s, 0): worst_state[s] = m
        allc += ac; steers += st
    s = sorted(allc)
    return worst_state, dict(
        rms=math.sqrt(sum(c*c for c in allc)/len(allc)),
        p95=s[int(0.95*len(s))], mx=s[-1],
        act=50.0*sum(abs(b-a) for a, b in zip(steers, steers[1:]))/max(1, len(steers)-1))


def route_time(tbl):
    t = 0.0
    for s0, s1, _ in segments():
        for i in range(s0, s1):
            d = math.hypot(PTS[i+1][0]-PTS[i][0], PTS[i+1][1]-PTS[i][1])
            t += d / max(0.3, tbl.get(PTS[i][2], 450)*0.0041667)
    return t


def optimize(limit, mode, base, gain, rounds=14):
    tbl = dict(BASE_SPD)
    for r in range(rounds):
        ws, _ = evaluate(tbl, mode, base, gain)
        changed = False
        for s in tbl:
            m = ws.get(s)
            if m is None:
                continue
            if m < limit * 0.80:
                nv = min(RAW_MAX, int(tbl[s] * 1.15))
            elif m > limit:
                nv = max(RAW_MIN, int(tbl[s] * 0.88))
            else:
                continue
            if nv != tbl[s]:
                tbl[s] = nv; changed = True
        if not changed:
            break
    ws, st = evaluate(tbl, mode, base, gain)
    return tbl, ws, st


if __name__ == '__main__':
    mode, base, gain = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    print('Ld: %s base %.1f gain %.1f' % (mode, base, gain))
    print('%-7s %-8s %-7s %-7s %-7s %s' % ('한계', '주파시간', 'rms', 'p95', 'max', 'act'))
    base_t = route_time(BASE_SPD)
    _, st0 = evaluate(BASE_SPD, mode, base, gain)
    print('%-7s %-8.0f %-7.3f %-7.3f %-7.3f %.1f  <- 현행 속도표' %
          ('-', base_t, st0['rms'], st0['p95'], st0['mx'], st0['act']))
    for lim in (0.30, 0.35, 0.40, 0.50, 0.60):
        tbl, ws, st = optimize(lim, mode, base, gain)
        t = route_time(tbl)
        print('%-7.2f %-8.0f %-7.3f %-7.3f %-7.3f %.1f' %
              (lim, t, st['rms'], st['p95'], st['mx'], st['act']))
        print('   속도표:', ' '.join('%d:%d' % (k, v) for k, v in sorted(tbl.items())))
