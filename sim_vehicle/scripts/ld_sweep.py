#!/usr/bin/env python3
"""2026-09-08: Ld 피팅 스윕 드라이버.

ld_sweep.launch 를 조합별로 돌리고, 주행 로그를 0.csv 원경로에 대해
다시 투영해서 측정 구간(state 3, idx 850~1400)만의 CTE 를 낸다.
eval_metrics 가 쓰는 CTE 는 /final_waypoints(전방 윈도우) 최근접 '꼭짓점'
거리라서 구간을 못 자르고 값도 조금 크게 나온다. 여기서는 전체 경로에
대한 선분 투영 거리를 쓴다.
"""
import csv, math, os, signal, subprocess, sys, time

CSV = '/home/kuuve/catkin_ws/src/data_시흥/0.csv'
LO, HI = 850, 1400          # 측정 구간 (state 3)
SEARCH_LO, SEARCH_HI = 700, 1600   # 최근접 탐색 범위. 경로가 되돌아오는
                                   # 구간에 잘못 붙는 것을 막는다.
SPEED_SCALE = 0.0041667

def load_path():
    pts = []
    for r in csv.reader(open(CSV)):
        pts.append((float(r[1]), float(r[2])))
    return pts

PATH = load_path()

def project(x, y):
    """(최근접 인덱스, 선분까지의 수직거리)"""
    best_d, best_i = 1e9, -1
    for i in range(SEARCH_LO, SEARCH_HI):
        ax, ay = PATH[i]
        bx, by = PATH[i + 1]
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((x-ax)*ex + (y-ay)*ey) / L2))
        px, py = ax + ex * t, ay + ey * t
        d = math.hypot(x - px, y - py)
        if d < best_d:
            best_d, best_i = d, i
    return best_i, best_d

def analyze(path):
    ctes, steers = [], []
    entered = False
    with open(path) as f:
        rd = csv.reader(f)
        next(rd, None)
        for row in rd:
            if len(row) < 5:
                continue
            x, y, st = float(row[1]), float(row[2]), float(row[4])
            i, d = project(x, y)
            if LO <= i <= HI:
                entered = True
                ctes.append(d)
                steers.append(st)
            elif entered and i > HI:
                break
    if not ctes:
        return None
    ctes_s = sorted(ctes)
    q = lambda p: ctes_s[min(len(ctes_s) - 1, int(p * len(ctes_s)))]
    rev = sum(1 for a, b in zip(steers, steers[1:])
              if (a > 0.3 and b < -0.3) or (a < -0.3 and b > 0.3))
    rms = math.sqrt(sum(c * c for c in ctes) / len(ctes))
    # 조향 활동량. Ld 가 작으면 CTE 는 줄어도 조향이 떨리기 시작한다.
    # 부호 반전 횟수만으로는 그 떨림이 0 으로 나오는 경우가 많아
    # 평균 |조향 변화율| [deg/s] 를 같이 본다. 로그는 50Hz.
    act = 50.0 * sum(abs(b - a) for a, b in zip(steers, steers[1:])) / max(1, len(steers) - 1)
    return dict(n=len(ctes), p50=q(0.5), p95=q(0.95), mx=ctes_s[-1],
                rms=rms, rev=rev, act=act)

def run(alpha, base, gain, speed, out):
    v = speed * SPEED_SCALE
    dur = 58.0 / v + 12.0
    cmd = ['roslaunch', 'sim_vehicle', 'ld_sweep.launch',
           'steer_lag_alpha:=%.5f' % alpha, 'ld_base:=%.2f' % base,
           'ld_gain:=%.3f' % gain, 'speed_override:=%d' % speed,
           'out:=' + out]
    if os.path.exists(out):
        os.remove(out)
    log = open(out + '.log', 'w')
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                         preexec_fn=os.setsid)
    time.sleep(dur)
    os.killpg(os.getpgid(p.pid), signal.SIGINT)
    try:
        p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        p.wait()
    log.close()
    return analyze(out)

if __name__ == '__main__':
    # argv: alpha,base,gain,speed  를 세미콜론으로 이어 여러 조합
    combos = []
    for spec in sys.argv[1].split(';'):
        a, b, g, s = spec.split(',')
        combos.append((float(a), float(b), float(g), int(s)))
    tag = sys.argv[2] if len(sys.argv) > 2 else 'sweep'
    res_path = '/tmp/%s_results.csv' % tag
    with open(res_path, 'w') as rf:
        rf.write('alpha,tau,base,gain,speed_raw,v_mps,n,p50,p95,max,rms,rev,act\n')
    for a, b, g, s in combos:
        out = '/tmp/%s_a%.4f_b%.2f_g%.2f_s%d.csv' % (tag, a, b, g, s)
        r = run(a, b, g, s, out)
        tau = -0.02 / math.log(1 - a)
        if r is None:
            line = '%.4f,%.2f,%.2f,%.2f,%d,%.2f,0,,,,,,' % (a, tau, b, g, s, s*SPEED_SCALE)
            print('[FAIL] ' + line, flush=True)
        else:
            line = '%.4f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%d,%.1f' % (
                a, tau, b, g, s, s*SPEED_SCALE, r['n'], r['p50'], r['p95'],
                r['mx'], r['rms'], r['rev'], r['act'])
            print('tau=%.2f Ld=%.2f gain=%.2f v=%.2f -> p50 %.3f p95 %.3f max %.3f rms %.3f rev %d act %.1f (n=%d)'
                  % (tau, b, g, s*SPEED_SCALE, r['p50'], r['p95'], r['mx'],
                     r['rms'], r['rev'], r['act'], r['n']), flush=True)
        with open(res_path, 'a') as rf:
            rf.write(line + '\n')
    print('results -> ' + res_path, flush=True)
