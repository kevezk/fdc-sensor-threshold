"""
replay.py - trace CSV를 다시 돌려 '현재 설정'과 '개선 설정'의 SPC 알람을 비교

사용:
  python replay.py trace_20261119_182333.csv
  python replay.py trace.csv --eval-from 180      (비교 구간 시작, 초 단위. 기본=개선 설정 학습 종료 시점)

'현재 설정'은 fdc_monitor.py v2.2의 SPC 로직을 그대로 옮긴 것이다
(같은 trace에서 alarms CSV와 알람 횟수가 일치하는지 먼저 확인할 것).
"""

import argparse
import collections
import csv
import math
import statistics

HZ = 2  # 펌웨어 보고 주기 2 Hz

# ------------------------------------------------------------------ 설정
CURRENT = {
    "name": "현재",
    # 실제 실행 명령(--vars dp t_in rh)과 맞춤. fdc_monitor.py 기본값은 t_dht도 포함
    "vars": ["dp", "t_in", "rh"],
    "baseline_sec": 30,          # 60샘플
    "block": 1,                  # RUN7/EWMA를 원데이터(0.5초)에 적용
    "ooc_set": 1, "ooc_clear": 1,      # 1점 이탈 즉시 SET, 1점 복귀 즉시 CLEAR
    "blk_clear": 1,
    "min_std_raw": {"dp": 3.0, "t_in": 0.05, "t_dht": 0.5, "rh": 0.3},
    "min_std_blk": {"dp": 3.0, "t_in": 0.05, "t_dht": 0.5, "rh": 0.3},
}

IMPROVED = {
    "name": "개선",
    # dt = t_in - t_out : 같은 실내 온도 변화는 지워지고 챔버만의 온도 변화가 남음
    "vars": ["dp", "t_in", "dt", "t_dht", "rh"],
    "baseline_sec": 180,         # 3분 (이 trace가 9.5분이라 3분. 실제 운영은 10분 권장)
    "block": 20,                 # RUN7/EWMA는 10초 평균에 적용 (7점 = 70초)
    "ooc_set": 2, "ooc_clear": 20,     # 2점 연속 이탈 시 SET, 10초 연속 정상이어야 CLEAR
    "blk_clear": 3,                    # 10초 평균 3개(30초) 연속 정상이어야 CLEAR
    # 원데이터 OOC용 최소 σ
    "min_std_raw": {"dp": 3.0, "t_in": 0.3, "dt": 0.05, "t_dht": 0.5, "rh": 1.0},
    # 10초 평균용 최소 σ
    "min_std_blk": {"dp": 1.0, "t_in": 0.3, "dt": 0.05, "t_dht": 0.5, "rh": 1.0},
}

LAM = 0.2
RUN_N = 7
DEADBAND = 0.5


# ------------------------------------------------------------------ 엔진
class VarSPC:
    def __init__(self, name, cfg):
        self.name, self.cfg = name, cfg
        self.base_n = cfg["baseline_sec"] * HZ
        self.block = cfg["block"]
        self.raw_base, self.blk_base = [], []
        self.buf = []
        self.ready = False
        self.side = collections.deque(maxlen=RUN_N)
        # 알람 상태와 지속/복귀 카운터
        self.state = {"OOC": False, "RUN7": False, "EWMA": False}
        self.ooc_in = self.ooc_out = 0
        self.blk_ok = {"RUN7": 0, "EWMA": 0}

    def _finish_baseline(self):
        c = self.cfg
        self.mean = statistics.mean(self.raw_base)
        self.std = max(statistics.pstdev(self.raw_base), c["min_std_raw"].get(self.name, 1e-6))
        if self.block == 1:
            self.bmean, self.bstd = self.mean, max(statistics.pstdev(self.raw_base),
                                                   c["min_std_blk"].get(self.name, 1e-6))
        else:
            bs = self.blk_base or [self.mean]
            self.bmean = statistics.mean(bs)
            sd = statistics.pstdev(bs) if len(bs) > 1 else 0.0
            self.bstd = max(sd, c["min_std_blk"].get(self.name, 1e-6))
        self.ewma = self.bmean
        self.ready = True

    def update(self, v, sample_idx):
        """샘플 하나 처리 → 현재 걸려 있는 알람 집합"""
        c = self.cfg
        # 블록 평균 누적 (값이 None이어도 샘플 수로 블록을 끊음)
        if v is not None:
            self.buf.append(v)
        block_done = (sample_idx + 1) % self.block == 0
        bval = None
        if block_done:
            bval = statistics.mean(self.buf) if self.buf else None
            self.buf = []

        if not self.ready:
            if v is not None:
                self.raw_base.append(v)
            if bval is not None:
                self.blk_base.append(bval)
            if sample_idx + 1 >= self.base_n and self.raw_base:
                self._finish_baseline()
            return set()

        # --- OOC: 원데이터, SET/CLEAR 지속 조건
        if v is not None:
            out = abs(v - self.mean) > 3 * self.std
            if out:
                self.ooc_in += 1
                self.ooc_out = 0
            else:
                self.ooc_out += 1
                self.ooc_in = 0
            if not self.state["OOC"] and self.ooc_in >= c["ooc_set"]:
                self.state["OOC"] = True
            elif self.state["OOC"] and self.ooc_out >= c["ooc_clear"]:
                self.state["OOC"] = False

        # --- RUN7 / EWMA: 블록 평균(현재 설정은 block=1 → 원데이터)
        if bval is not None:
            d = bval - self.bmean
            self.side.append(0 if abs(d) <= DEADBAND * self.bstd else (1 if d > 0 else -1))
            hit = {"RUN7": len(self.side) == RUN_N and abs(sum(self.side)) == RUN_N}
            self.ewma = LAM * bval + (1 - LAM) * self.ewma
            hit["EWMA"] = abs(self.ewma - self.bmean) > 3 * self.bstd * math.sqrt(LAM / (2 - LAM))
            for r in ("RUN7", "EWMA"):
                if hit[r]:
                    self.state[r] = True
                    self.blk_ok[r] = 0
                elif self.state[r]:
                    self.blk_ok[r] += 1
                    if self.blk_ok[r] >= c["blk_clear"]:
                        self.state[r] = False
        return {r for r, on in self.state.items() if on}


def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))

    def f(r, k):
        x = r.get(k, "")
        return None if x in ("", "null", "None", None) else float(x)

    out = []
    for r in rows:
        d = {k: f(r, k) for k in ("dp", "t_in", "t_out", "t_dht", "rh")}
        d["dt"] = None if d["t_in"] is None or d["t_out"] is None else d["t_in"] - d["t_out"]
        out.append(d)
    return out


def replay(samples, cfg, eval_from):
    spc = {v: VarSPC(v, cfg) for v in cfg["vars"]}
    prev = set()
    sets = collections.Counter()
    on = collections.Counter()
    n_eval = 0
    for i, s in enumerate(samples):
        now = set()
        for v, sp in spc.items():
            now |= {f"{r}:{v}" for r in sp.update(s[v], i)}
        if i >= eval_from:
            n_eval += 1
            for a in now - prev:
                sets[a] += 1
            for a in now:
                on[a] += 1
        prev = now
    any_on = None
    return sets, on, n_eval, spc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--eval-from", type=float, default=None,
                    help="비교 시작 시점(초). 기본: 두 설정 중 늦은 학습 종료 시점")
    a = ap.parse_args()
    samples = load(a.trace)
    start = int((a.eval_from if a.eval_from is not None
                 else max(CURRENT["baseline_sec"], IMPROVED["baseline_sec"])) * HZ)

    res = {c["name"]: replay(samples, c, start) for c in (CURRENT, IMPROVED)}
    n_eval = res["현재"][2]
    print(f"전체 {len(samples)}샘플, 비교 구간: {start}번째 샘플부터 {n_eval}샘플 ({n_eval / HZ / 60:.1f}분)\n")

    for name, (_, _, _, spc) in res.items():
        print(f"[{name}] 학습 결과")
        for v, sp in spc.items():
            if sp.ready:
                print(f"   {v:6} 평균 {sp.mean:9.3f}  원데이터σ {sp.std:.3f}  "
                      f"블록평균σ {sp.bstd:.3f}  (블록 {sp.block}샘플)")
        print()

    allvars = []
    for c in (CURRENT, IMPROVED):
        for v in c["vars"]:
            if v not in allvars:
                allvars.append(v)
    print(f"{'변수':6} {'규칙':5} | {'현재 SET':>8} {'켜짐%':>7} | {'개선 SET':>8} {'켜짐%':>7}")
    print("-" * 52)
    for v in allvars:
        for r in ("OOC", "RUN7", "EWMA"):
            key = f"{r}:{v}"
            cells = []
            for name, cfg in (("현재", CURRENT), ("개선", IMPROVED)):
                if v not in cfg["vars"]:
                    cells.append(f"{'-':>8} {'-':>7}")
                    continue
                sets, on, n, _ = res[name]
                cells.append(f"{sets.get(key, 0):8d} {100 * on.get(key, 0) / n:6.1f}%")
            print(f"{v:6} {r:5} | {cells[0]} | {cells[1]}")
    print("-" * 52)
    for name in ("현재", "개선"):
        sets, on, n, _ = res[name]
        print(f"{name}: 알람 SET 합계 {sum(sets.values())}회")


if __name__ == "__main__":
    main()
