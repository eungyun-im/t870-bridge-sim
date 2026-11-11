#!/usr/bin/env python3
"""offline_sim.py 가 ROS 실행 결과를 재현하는지 대조한다.

정답지는 ld_sweep.py 가 남긴 /tmp/<tag>_results.csv 다. 같은 조합을
오프라인으로 돌려 같은 측정창(state 3, idx 850~1400)의 CTE 를 비교한다.
"""
import csv, glob, math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offline_sim import Sim, load_csv

CSV = '/home/kuuve/catkin_ws/src/data_시흥/0.csv'
LO, HI = 850, 1400
START = 820
PTS = load_csv(CSV)


def cte_stats(log, path, lo, hi):
    ctes, steers = [], []
    for t, x, y, st, closest in log:
        if not (lo <= closest <= hi):
            continue
        best = 1e9
        for i in range(max(lo - 60, 1), min(hi + 60, len(path) - 1)):
            ax, ay = path[i][0], path[i][1]
            bx, by = path[i + 1][0], path[i + 1][1]
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey
            u = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
            d = math.hypot(x - (ax + ex * u), y - (ay + ey * u))
            if d < best:
                best = d
        ctes.append(best)
        steers.append(st)
    if not ctes:
        return None
    s = sorted(ctes)
    rms = math.sqrt(sum(c * c for c in ctes) / len(ctes))
    act = 50.0 * sum(abs(b - a) for a, b in zip(steers, steers[1:])) / max(1, len(steers) - 1)
    return dict(n=len(ctes), p50=s[len(s) // 2], p95=s[int(0.95 * len(s))],
                mx=s[-1], rms=rms, act=act)


def offline(alpha, base, gain, speed):
    sim = Sim(PTS, START, 2, alpha, base, gain, speed_raw=speed)
    log = sim.run(HI + 30)
    return cte_stats(log, PTS, LO, HI)


if __name__ == '__main__':
    rows = []
    for tag in sys.argv[1:]:
        for r in csv.DictReader(open('/tmp/%s_results.csv' % tag)):
            if not r['rms']:
                continue
            rows.append(r)
    print('%-5s %-5s %-6s %-6s | %-24s | %-24s' % ('tau', 'Ld', 'gain', 'v', 'ROS  rms/p95/max/act', 'OFF  rms/p95/max/act'))
    errs = []
    for r in rows:
        a, b, g, s = float(r['alpha']), float(r['base']), float(r['gain']), int(r['speed_raw'])
        o = offline(a, b, g, s)
        ros_rms = float(r['rms'])
        if o is None:
            print('%-5s %-5s %-6s %-6s | FAIL' % (r['tau'], b, g, r['v_mps']))
            continue
        errs.append((o['rms'] - ros_rms, ros_rms, o['rms']))
        print('%-5s %-5.1f %-6.1f %-6s | %5.3f %5.3f %5.3f %5.1f | %5.3f %5.3f %5.3f %5.1f'
              % (r['tau'], b, g, r['v_mps'], ros_rms, float(r['p95']), float(r['max']),
                 float(r.get('act') or 0.0), o['rms'], o['p95'], o['mx'], o['act']))
    if errs:
        stable = [e for e in errs if e[1] < 0.5]
        print()
        print('안정 조합 %d 개 rms 차이: 평균 %+.4f m, 최대 %.4f m'
              % (len(stable), sum(e[0] for e in stable) / len(stable),
                 max(abs(e[0]) for e in stable)))
