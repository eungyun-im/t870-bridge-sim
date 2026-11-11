#!/usr/bin/env python3
"""2026-09-09: 속도 x Ld 프론티어. "가장 작은 CTE 로 가장 빠르게" 를 찾는다.

속도 knob 은 state 테이블 전체에 곱하는 배율로 둔다. 전 구간을 한 값으로
덮으면(speed_override) state 24(raw 250, 급커브) 같은 곳까지 같이 올라가
설계 의도가 깨진다. 배율은 상대적인 완급을 유지한 채 전체를 올린다.
raw 1000(15km/h, T870 최고속)에서 자른다.

주차 구간은 오프라인 모델 밖이라 여기서 안 센다. 주차/회피 미션 성립은
ROS 통합 실행(sim_full_test.launch)으로 따로 확인해야 한다.
"""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_sweep
from full_sweep import run_route, STATE_SPD, segments, PTS

TAUS = [(0.0511, 0.38), (0.0357, 0.55), (0.0328, 0.60)]
BASE_SPD = dict(STATE_SPD)


def set_mul(m):
    for k, v in BASE_SPD.items():
        full_sweep.STATE_SPD[k] = min(1000, int(round(v * m)))


def route_time(mul):
    """토막 길이 / 지령속도 로 구한 이상적 주파 시간 [s]. 비교용 지표."""
    t = 0.0
    for s0, s1, _ in segments():
        for i in range(s0, s1):
            d = math.hypot(PTS[i+1][0]-PTS[i][0], PTS[i+1][1]-PTS[i][1])
            raw = min(1000, BASE_SPD.get(PTS[i][2], 450) * mul)
            v = max(0.3, raw * 0.0041667)
            t += d / v
    return t


CONFIGS = [('table', 0.0, 0.0),
           ('fixed', 3.0, 0.0), ('fixed', 3.5, 0.0), ('fixed', 4.0, 0.0),
           ('speed', 1.5, 0.7), ('speed', 1.5, 0.8), ('speed', 2.0, 0.4),
           ('speed', 2.0, 0.5), ('speed', 2.5, 0.3), ('speed', 1.0, 0.9)]
MULS = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2]

out = open('/tmp/speed_grid.csv', 'w')
out.write('mul,route_time,mode,base,gain,worst_rms,worst_p95,worst_max,worst_act\n')
rows = []
for mul in MULS:
    set_mul(mul)
    rt = route_time(mul)
    for mode, b, g in CONFIGS:
        rs = [run_route(a, mode, b, g) for a, _ in TAUS]
        wr = max(r['rms'] for r in rs)
        wp = max(r['p95'] for r in rs)
        wm = max(r['mx'] for r in rs)
        wa = max(r['act'] for r in rs)
        rows.append((mul, rt, mode, b, g, wr, wp, wm, wa))
        out.write('%.1f,%.1f,%s,%.1f,%.1f,%.4f,%.4f,%.4f,%.2f\n'
                  % (mul, rt, mode, b, g, wr, wp, wm, wa))
        out.flush()
        print('mul %.1f (%.0fs) %-6s b%.1f g%.1f -> rms %.3f p95 %.3f max %.3f act %.1f'
              % (mul, rt, mode, b, g, wr, wp, wm, wa), flush=True)
out.close()

print()
print('=== max CTE 한계별 최속 설정 ===')
for lim in (0.30, 0.40, 0.50, 0.60, 0.80, 1.00):
    ok = [r for r in rows if r[7] <= lim]
    if not ok:
        print('  max<=%.2fm : 없음' % lim)
        continue
    best = min(ok, key=lambda r: r[1])
    print('  max<=%.2fm : mul %.1f (%.0fs) %-6s base %.1f gain %.1f | rms %.3f max %.3f act %.1f'
          % (lim, best[0], best[1], best[2], best[3], best[4], best[5], best[7], best[8]))
