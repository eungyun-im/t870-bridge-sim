#!/usr/bin/env python3
"""전 구간 Ld 격자 스윕. tau 3종 전부에서 성립하는 설정을 찾는다."""
import sys, os, csv
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from full_sweep import run_route

TAUS = [(0.0511, 0.38), (0.0357, 0.55), (0.0328, 0.60)]
OUT = '/tmp/full_grid.csv'

combos = [('table', 0.0, 0.0)]
for b in [2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0]:
    combos.append(('fixed', b, 0.0))
for b in [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]:
    for g in [0.1*k for k in range(0, 13)]:
        combos.append(('speed', b, round(g, 1)))

with open(OUT, 'w') as f:
    f.write('mode,base,gain,tau,rms,p95,max,act\n')

rows = []
for mode, b, g in combos:
    rec = {}
    for a, tau in TAUS:
        r = run_route(a, mode, b, g)
        rec[tau] = r
        with open(OUT, 'a') as f:
            f.write('%s,%.1f,%.1f,%.2f,%.4f,%.4f,%.4f,%.2f\n'
                    % (mode, b, g, tau, r['rms'], r['p95'], r['mx'], r['act']))
    # tau 전체에서의 최악값으로 평가한다 (실차 tau 를 모르므로)
    wr = max(rec[t]['rms'] for _, t in TAUS)
    wm = max(rec[t]['mx'] for _, t in TAUS)
    wa = max(rec[t]['act'] for _, t in TAUS)
    rows.append((wr, wm, wa, mode, b, g))
    print('%-6s base %.1f gain %.1f -> worst rms %.3f max %.3f act %.1f'
          % (mode, b, g, wr, wm, wa), flush=True)

print()
print('=== worst-case rms 기준 상위 15 ===')
for wr, wm, wa, mode, b, g in sorted(rows)[:15]:
    print('  %-6s base %.1f gain %.1f | rms %.3f max %.3f act %.1f' % (mode, b, g, wr, wm, wa))
print()
print('=== worst-case max 기준 상위 15 ===')
for wr, wm, wa, mode, b, g in sorted(rows, key=lambda z: z[1])[:15]:
    print('  %-6s base %.1f gain %.1f | rms %.3f max %.3f act %.1f' % (mode, b, g, wr, wm, wa))
