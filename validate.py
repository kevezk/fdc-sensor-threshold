"""
validate.py - 새 기록(정상 30분 + 이상 만들기)으로 설정 검증

  python validate.py trace_20261119_185552.csv [--old trace_20261119_182333.csv]

하는 일
  1) 정상 구간 오경보 비교 : 현재 / 개선(replay.py 그대로) / 개선 + dp 계단 검출기
  2) dp 계단 검출기 문턱(h) : 정상 구간 앞쪽 절반에서만 정하고, 뒤쪽 절반과 옛 trace로 검증
  3) 직접 만든 이상 4개      : 설정별 검출 여부와 지연
  4) 계단 주입              : 정상 구간 뒤쪽 절반에 +5/+10/+20 Pa를 30초 동안 더해 검출률·지연 측정

replay.py와 같은 폴더에 두고 실행한다.
"""

import argparse
import collections
import copy
import csv
import statistics

import replay as R

HZ = R.HZ
QUIET_END = "19:25:53"        # 시작 후 30분 (건드리지 않은 구간 끝)
EVAL_FROM = 1200              # 정상 구간 비교 시작 샘플: 2 Hz × 600초 = 10분 (모든 설정의 학습이 끝난 뒤)
TARGET_PA, TARGET_SEC = 10.0, 10.0


# ------------------------------------------------------- dp 계단 검출기
class DpStep:
    """짧은 평균(5초)과 '5초 전까지의 2분 중앙값'의 차이 D로 계단 변화를 잡는다.
    느린 영점 표류는 기준(2분 중앙값)이 따라가므로 지워지고, 갑작스러운 변화만 남는다.
    |D| > h 가 2점 연속이면 SET, |D| < h/2 가 10초 이어지면 CLEAR."""

    def __init__(self, h, short=10, ref=240, gap=10, set_n=2, clear_n=20):
        self.h, self.short, self.ref, self.gap = h, short, ref, gap
        self.set_n, self.clear_n = set_n, clear_n
        self.buf = collections.deque(maxlen=short + gap + ref)
        self.on = False
        self.n_in = self.n_ok = 0
        self.last_d = None

    def d_value(self):
        if len(self.buf) < self.buf.maxlen:
            return None
        b = list(self.buf)
        ref = statistics.median(b[: self.ref])
        return statistics.mean(b[-self.short:]) - ref

    def update(self, v):
        if v is not None:
            self.buf.append(v)
        d = self.d_value()
        self.last_d = d
        if d is None:
            return set()
        if abs(d) > self.h:
            self.n_in += 1
            self.n_ok = 0
        else:
            self.n_in = 0
            self.n_ok = self.n_ok + 1 if abs(d) < self.h / 2 else 0
        if not self.on and self.n_in >= self.set_n:
            self.on = True
        elif self.on and self.n_ok >= self.clear_n:
            self.on = False
        return {"STEP:dp"} if self.on else set()


# ------------------------------------------------------- 공통 실행기
def load_with_time(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    samples = R.load(path)
    times = [r["time"][11:19] for r in rows]
    return samples, times


def run(samples, cfg=None, h=None):
    """샘플마다 켜져 있는 알람 집합의 리스트를 돌려준다.
    cfg: replay.py 설정 딕셔너리(None이면 SPC 없음), h: dp 계단 검출기 문턱(None이면 없음)"""
    spc = {v: R.VarSPC(v, cfg) for v in cfg["vars"]} if cfg else {}
    det = DpStep(h) if h is not None else None
    states = []
    for i, s in enumerate(samples):
        now = set()
        for v, sp in spc.items():
            now |= {f"{r}:{v}" for r in sp.update(s[v], i)}
        if det:
            now |= det.update(s["dp"])
        states.append(now)
    return states


def count(states, lo, hi):
    sets, on = collections.Counter(), collections.Counter()
    prev = states[lo - 1] if lo > 0 else set()
    for i in range(lo, hi):
        for a in states[i] - prev:
            sets[a] += 1
        for a in states[i]:
            on[a] += 1
        prev = states[i]
    return sets, on, hi - lo


def first_set(states, start, window, prefixes):
    """start 이후 window 샘플 안에서 prefixes 중 하나가 새로 SET된 첫 시점(샘플 차이).
    시작 시점에 이미 켜져 있던 알람은 'already'로 따로 표시."""
    match = lambda a: any(a.split(":")[1] == p for p in prefixes)
    already = sorted(a for a in states[start - 1] if match(a))
    for k in range(window):
        i = start + k
        new = {a for a in states[i] - states[i - 1] if match(a)}
        if new:
            return k, sorted(new), already
    return None, [], already


# ------------------------------------------------------- 설정
IMP = R.IMPROVED
IMP10 = copy.deepcopy(R.IMPROVED)
IMP10["name"] = "개선(학습10분)"
IMP10["baseline_sec"] = 600
# dp는 계단 검출기가 맡고, 분 단위로 흔들리는 dp에는 RUN7/EWMA를 쓰지 않는 설정
IMP_NODP = copy.deepcopy(IMP10)
IMP_NODP["name"] = "개선v2"
IMP_NODP["vars"] = [v for v in IMP10["vars"] if v != "dp"]
# 최종 설정: 개선v2 + dt 최솟값 0.1 °C
FINAL = copy.deepcopy(IMP_NODP)
FINAL["name"] = "최종"
FINAL["min_std_raw"] = dict(FINAL["min_std_raw"], dt=0.1)
FINAL["min_std_blk"] = dict(FINAL["min_std_blk"], dt=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--old", default=None, help="독립 검증용 옛 정상 trace")
    a = ap.parse_args()
    samples, times = load_with_time(a.trace)
    q_end = next(i for i, t in enumerate(times) if t >= QUIET_END)
    quiet = samples[:q_end]
    half = EVAL_FROM + (q_end - EVAL_FROM) // 2
    print(f"정상 구간: 0~{q_end}샘플 ({q_end / HZ / 60:.1f}분), 비교 구간 {EVAL_FROM}~{q_end}\n")

    # ---------- 2) h 정하기: 정상 구간 앞쪽 절반만 사용
    det = DpStep(h=1e9)
    ds = []
    for i, s in enumerate(quiet[:half]):
        det.update(s["dp"])
        if i >= EVAL_FROM and det.last_d is not None:
            ds.append(abs(det.last_d))
    ds.sort()
    p999, dmax = ds[int(0.999 * (len(ds) - 1))], ds[-1]
    h = round(max(dmax * 1.25, 4.0) * 2) / 2      # 최대값의 1.25배, 0.5 Pa 단위
    print(f"[문턱 정하기] 앞쪽 절반 {len(ds)}샘플: |D| 99.9% {p999:.2f} Pa, 최대 {dmax:.2f} Pa → h = {h} Pa")
    print(f"   10 Pa 계단이면 D ≈ 10 Pa → 여유 {10 - h:.1f} Pa\n")

    configs = [("현재", R.CURRENT, None), ("개선", IMP, None),
               ("개선(학습10분)", IMP10, None), ("개선v2(dt0.05)", IMP_NODP, h), ("최종", FINAL, h)]

    # ---------- 1) 정상 구간 오경보
    print("[1] 정상 구간 오경보 (비교 구간 = 앞쪽 절반 / 뒤쪽 절반)")
    keys = ["OOC:dp", "RUN7:dp", "EWMA:dp", "STEP:dp", "OOC:t_in", "RUN7:t_in", "EWMA:t_in",
            "OOC:dt", "RUN7:dt", "EWMA:dt", "OOC:rh", "RUN7:rh", "EWMA:rh",
            "OOC:t_dht", "RUN7:t_dht", "EWMA:t_dht"]
    results = {}
    for name, cfg, hh in configs:
        st = run(quiet, cfg, hh)
        results[name] = (count(st, EVAL_FROM, half), count(st, half, q_end))
    head = " ".join(f"{n:>16}" for n, _, _ in configs)
    print(f"{'알람':10} {head}")
    for k in keys:
        cells = []
        for name, cfg, hh in configs:
            has = (k.split(":")[1] in (cfg["vars"] if cfg else [])) or (k == "STEP:dp" and hh)
            if not has:
                cells.append(f"{'-':>16}")
                continue
            (s1, o1, n1), (s2, o2, n2) = results[name]
            cells.append(f"{s1.get(k, 0):3d}/{s2.get(k, 0):<3d}{100 * (o1.get(k, 0) + o2.get(k, 0)) / (n1 + n2):6.1f}%")
        print(f"{k:10} " + " ".join(cells))
    for name, *_ in configs:
        (s1, _, n1), (s2, _, n2) = results[name]
        tot = sum(s1.values()) + sum(s2.values())
        print(f"   {name}: SET 합계 {tot}회 = 시간당 {tot / ((n1 + n2) / HZ / 3600):.0f}회")
    print("   (칸: 앞쪽 SET/뒤쪽 SET, 켜짐 비율은 두 구간 합)\n")

    if a.old:
        old = R.load(a.old)
        st = run(old, None, h)
        s, o, n = count(st, 260, len(old))
        print(f"[독립 검증] 옛 정상 trace({n}샘플)에서 계단 검출기 오경보: SET {s.get('STEP:dp', 0)}회, "
              f"켜짐 {100 * o.get('STEP:dp', 0) / n:.1f}%")
        # dt 최솟값 검증: 옛 trace는 9.5분이라 학습 3분으로 돌림
        for fl in (0.05, 0.1):
            c = copy.deepcopy(FINAL)
            c["baseline_sec"] = 180
            c["min_std_raw"] = dict(c["min_std_raw"], dt=fl)
            c["min_std_blk"] = dict(c["min_std_blk"], dt=fl)
            st = run(old, c, None)
            s, o, n = count(st, 360, len(old))
            dts = {k: v for k, v in s.items() if k.endswith(":dt")}
            on = {k: round(100 * v / n, 1) for k, v in o.items() if k.endswith(":dt")}
            print(f"[독립 검증] 옛 trace, dt 최솟값 {fl}: dt 알람 SET {dts or 0}, 켜짐% {on or 0}  ({n}샘플)")
        print()

    # ---------- 3) 직접 만든 이상
    def onset(after, var, th):
        i0 = next(i for i, t in enumerate(times) if t >= after)
        base = statistics.mean(samples[j][var] for j in range(i0 - 60, i0))
        return next(i for i in range(i0, len(samples)) if samples[i][var] - base > th)

    events = [("E1 내부센서 덮고 약하게 누름", onset("19:26:40", "t_in", 0.02), ["dp", "t_in", "dt"]),
              ("E2 내부센서 덮고 세게 누름", onset("19:28:50", "t_in", 0.02), ["dp", "t_in", "dt"]),
              ("E3 DHT11 입김", onset("19:31:00", "rh", 2), ["rh", "t_dht"]),
              ("E4 손가락 근접", onset("19:33:00", "t_in", 0.02), ["dp", "t_in", "dt"])]
    print("[3] 직접 만든 이상 (시작 시점은 t_in·rh가 오르기 시작한 샘플로 추정, 60초 안 새 SET 기준)")
    full = {name: run(samples, cfg, hh) for name, cfg, hh in configs}
    for ev, i, vars_ in events:
        dp_pre = statistics.mean(samples[j]["dp"] for j in range(i - 60, i))
        dp_dur = statistics.mean(samples[j]["dp"] for j in range(i, i + 60))
        print(f" {ev} @ {times[i]} (dp 변화 {dp_dur - dp_pre:+.1f} Pa)")
        for name, *_ in configs:
            k, new, already = first_set(full[name], i, 60 * HZ, vars_)
            res = f"{k / HZ:5.1f}초 {','.join(new)}" if k is not None else "  못 잡음"
            print(f"    {name:14} {res:40} 시작 때 이미 켜짐: {','.join(already) or '없음'}")
    print()

    # ---------- 4) 계단 주입
    print("[4] 계단 주입: 정상 구간 뒤쪽 절반, 30초 지속, 10초 안 새 SET = 검출")
    positions = list(range(half + 300, q_end - 80, 80))
    # 대조군: 주입 없이 같은 위치에서도 10초 안에 dp 알람이 새로 뜨는가 (우연 검출률)
    print(f" 대조(주입 없음, {len(positions)}곳)")
    for name, cfg, hh in configs:
        st = run(quiet, cfg, hh)
        chance = sum(first_set(st, p, int(TARGET_SEC * HZ), ["dp"])[0] is not None for p in positions)
        print(f"    {name:14} 우연히 새 SET {chance}/{len(positions)}")
    for size in (5.0, 10.0, 20.0):
        print(f" +{size:.0f} Pa ({len(positions)}곳)")
        for name, cfg, hh in configs:
            if cfg is not None and "dp" not in cfg["vars"] and hh is None:
                continue
            hit, already, lat = 0, 0, []
            for p in positions:
                inj = copy.deepcopy(quiet)
                for j in range(p, p + 30 * HZ):
                    inj[j]["dp"] += size
                st = run(inj, cfg, hh)
                k, _, alr = first_set(st, p, int(TARGET_SEC * HZ), ["dp"])
                if k is not None:
                    hit += 1
                    lat.append(k / HZ)
                elif alr:
                    already += 1
            med = f"{statistics.median(lat):.1f}초" if lat else "-"
            print(f"    {name:14} 검출 {hit}/{len(positions)}  (이미 켜져 있어 새 SET 없음 {already})  지연 중앙값 {med}")


if __name__ == "__main__":
    main()
