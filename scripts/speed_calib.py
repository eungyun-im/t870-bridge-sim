#!/usr/bin/env python3
"""bag 에서 구동 raw -> 실제 속도[m/s] 캘리브레이션.

/vehicle/applied_speed (선에 나간 raw) 와 /vehicle/encoder (누적 카운트) 를 짝지어
raw 단계별 정상상태 속도를 뽑고 v = k*(raw - raw0) 를 회귀한다.

엔코더 스케일은 drift_0906_1257/1302 bag 에서 GPS 주행거리 대비로 구한 값이다
(두 bag 각각 0.01155 / 0.01162 m/count, 0.6% 이내 일치). 실내에서 GPS 없이
측정할 수 있는 이유가 이것이다. 다른 바퀴/타이어로 바꾸면 다시 재야 한다.

사용법:
    python3 speed_calib.py sweep.bag
    python3 speed_calib.py sweep.bag --mpc 0.01158 --min-hold 2.0
"""
import argparse
import sys

import numpy as np

MPC_DEFAULT = 0.011585   # m/count


def load(bagpath, topics):
    import rosbag
    out = {t: [] for t in topics}
    with rosbag.Bag(bagpath) as b:
        have = set(b.get_type_and_topic_info().topics)
        missing = [t for t in topics if t not in have]
        if missing:
            print("경고: bag 에 없는 토픽 %s" % missing, file=sys.stderr)
        for topic, msg, t in b.read_messages(topics=topics):
            out[topic].append((t.to_sec(), msg.data))
    return {k: np.array(v) for k, v in out.items() if v}


def enc_speed(t, c, mpc, win=10):
    """카운트 -> 속도. win 샘플(20Hz 기준 0.5s) 차분.

    엔코더는 후진에서 감소한다(방향 부호가 붙어 있다). 캘리브레이션에 필요한 건
    속력이므로 호출부에서 절댓값을 쓰되, 전진/후진을 나눠 보고한다 -- 실측에서
    같은 raw 900 에 전진 2.88 / 후진 2.73 m/s 로 5% 차이가 났다.
    """
    v = np.full(len(t), np.nan)
    for i in range(win, len(t)):
        dt = t[i] - t[i - win]
        if dt > 0:
            v[i] = (c[i] - c[i - win]) * mpc / dt
    return v


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag")
    p.add_argument("--mpc", type=float, default=MPC_DEFAULT, help="m/count")
    p.add_argument("--min-hold", type=float, default=2.0, help="정상상태로 인정할 최소 유지시간 [s]")
    p.add_argument("--settle", type=float, default=0.5, help="구간 앞쪽 이만큼은 과도로 보고 버림 [s]")
    p.add_argument("--speed-topic", default="/vehicle/applied_speed",
                   help="raw 명령 토픽. 구버전 bag 이면 /vehicle/cmd_speed")
    p.add_argument("--anchor", nargs=2, type=float, action="append", metavar=("RAW", "V"),
                   help="외부 실측점 추가 (예: --anchor 350 1.088). 다른 bag 에서 얻은 "
                        "점을 회귀에 같이 넣을 때 쓴다")
    a = p.parse_args()

    d = load(a.bag, [a.speed_topic, "/vehicle/encoder"])
    if a.speed_topic not in d:
        raise SystemExit(f"{a.speed_topic} 이 없다. --speed-topic /vehicle/cmd_speed 로 시도해볼 것 "
                         "(단 cmd_speed 는 램프 전 AUTO 지령이라 MANUAL 에서는 비어 있다)")
    if "/vehicle/encoder" not in d:
        raise SystemExit("/vehicle/encoder 가 없다")

    cmd = d[a.speed_topic]
    enc = d["/vehicle/encoder"]
    te, ce = enc[:, 0], enc[:, 1]
    ve = enc_speed(te, ce, a.mpc)

    # raw 값이 일정하게 유지된 구간으로 자른다
    raw = cmd[:, 1].astype(int)
    edges = [0] + [i for i in range(1, len(raw)) if raw[i] != raw[i - 1]] + [len(raw)]
    rows = []
    for i0, i1 in zip(edges[:-1], edges[1:]):
        r = int(raw[i0])
        if r <= 0:
            continue
        ta, tb = cmd[i0, 0], cmd[i1 - 1, 0]
        if tb - ta < a.min_hold:
            continue
        m = (te >= ta + a.settle) & (te <= tb)
        vv = ve[m]
        vv = vv[~np.isnan(vv)]
        if len(vv) < 3:
            continue
        mean_signed = float(np.mean(vv))
        direction = "후진" if mean_signed < 0 else "전진"
        rows.append((r, tb - ta, abs(mean_signed), float(np.std(vv)), len(vv), direction))

    if not rows:
        raise SystemExit(f"{a.min_hold}s 이상 유지된 raw 구간이 없다. --min-hold 를 낮추거나 "
                         "각 단계를 더 길게 유지할 것")

    print(f"엔코더 스케일 {a.mpc:.5f} m/count,  20Hz 분해능 {a.mpc*20:.3f} m/s")
    print(f"raw 토픽 {a.speed_topic},  정상상태 판정: 유지 {a.min_hold}s 이상, 앞 {a.settle}s 제외\n")
    print(f"{'raw':>6} {'유지s':>7} {'v [m/s]':>9} {'km/h':>7} {'std':>7} {'n':>5}  방향")
    for r, dur, v, sd, n, dr in sorted(rows):
        print(f"{r:6d} {dur:7.1f} {v:9.3f} {v*3.6:7.2f} {sd:7.3f} {n:5d}  {dr}")

    fwd = [v for r, dur, v, sd, n, dr in rows if dr == "전진"]
    rev = [v for r, dur, v, sd, n, dr in rows if dr == "후진"]
    if fwd and rev:
        print(f"\n전진 평균 {np.mean(fwd):.3f} m/s (n={len(fwd)}), "
              f"후진 평균 {np.mean(rev):.3f} m/s (n={len(rev)}), "
              f"차이 {100*(np.mean(fwd)-np.mean(rev))/np.mean(fwd):+.1f}%")

    # 같은 raw 가 여러 번 나오면 평균
    agg = {}
    for r, dur, v, sd, n, dr in rows:
        agg.setdefault(r, []).append(v)
    for ar, av in (a.anchor or []):
        agg.setdefault(int(ar), []).append(av)
        print(f"  외부 앵커 추가: raw {int(ar)} -> {av:.3f} m/s")
    R = np.array(sorted(agg))
    V = np.array([np.mean(agg[r]) for r in R])

    print(f"\n유효 raw 레벨 {len(R)}개: {list(R)}")
    if len(R) < 2:
        print("회귀하려면 서로 다른 raw 레벨이 2개 이상 필요하다.")
        return
    k, b = np.polyfit(R, V, 1)
    raw0 = -b / k if k else float("nan")
    resid = V - (k * R + b)
    print(f"\n회귀: v = {k:.6f} * raw {b:+.4f}")
    print(f"  기동 문턱 raw0 = {raw0:.0f}   (v = k*(raw - raw0) 형태)")
    print(f"  잔차 최대 {np.abs(resid).max():.3f} m/s, RMS {np.sqrt((resid**2).mean()):.3f} m/s")
    for q in (350, 650, 900, 1000):
        print(f"  raw {q:4d} -> {k*q+b:5.2f} m/s ({(k*q+b)*3.6:5.2f} km/h)"
              + ("  <- 측정 범위 밖, 외삽" if q > R.max() else ""))


if __name__ == "__main__":
    main()
