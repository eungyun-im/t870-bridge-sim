#!/usr/bin/env python3
"""최종 격자. 토막 시작 정착 15m 를 뺀 뒤 Ld x 속도배율 프론티어."""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_sweep
from full_sweep import run_route, STATE_SPD, segments, PTS

BASE = dict(STATE_SPD)
TAUS = [(0.0511, 0.38), (0.0357, 0.55), (0.0328, 0.60)]

def set_mul(m):
    for k, v in BASE.items():
        full_sweep.STATE_SPD[k] = min(1000, int(round(v*m)))

def rtime(m):
    t = 0.0
    for s0, s1, _ in segments():
        for i in range(s0, s1):
            d = math.hypot(PTS[i+1][0]-PTS[i][0], PTS[i+1][1]-PTS[i][1])
            t += d / max(0.3, min(1000, BASE.get(PTS[i][2], 450)*m)*0.0041667)
    return t

print('%-6s %-5s %-7s %-7s %-7s %-7s %s' % ('mul', 'Ld', 'time', 'rms', 'p95', 'max', 'act'))
for mul in [1.0, 1.2, 1.4, 1.6, 1.8, 2.0]:
    set_mul(mul); t = rtime(mul)
    for mode, b, g in [('table',0,0),('fixed',2.5,0),('fixed',3.0,0),('fixed',3.5,0),
                       ('fixed',4.0,0),('speed',2.0,0.4),('speed',2.5,0.3)]:
        rs = [run_route(a, mode, b, g) for a, _ in TAUS]
        lbl = 'table' if mode=='table' else ('%.1f'%b if mode=='fixed' else '%.1f+%.1fv'%(b,g))
        print('%-6.1f %-5s %-7.0f %-7.3f %-7.3f %-7.3f %.1f'
              % (mul, lbl, t, max(r['rms'] for r in rs), max(r['p95'] for r in rs),
                 max(r['mx'] for r in rs), max(r['act'] for r in rs)), flush=True)
