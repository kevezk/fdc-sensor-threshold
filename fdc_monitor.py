"""
미니 공정 챔버 FDC 서버 v2.2
ESP32(fdc_node)가 USB로 보내는 데이터를 실시간으로 받아

  1) 통신 무결성 검사   : 체크섬, 시퀀스 누락, 무응답, 장비 재부팅
  2) SPC 이상 감지      : ±3σ 관리한계, 7연속 규칙, EWMA 드리프트
  3) 원본 CSV 저장      : trace_*.csv  (모든 샘플, 분석용)
  4) 알람 이력 CSV 저장 : alarms_*.csv (알람 SET / CLEAR / 단발 EVENT)
  4-1) 1분 요약 CSV     : summary_*.csv (AI 분석용: 변수별 통계 + 그 1분의 알람·이벤트)
  5) 구글 시트 업로드   : 10초 요약 + 알람 이력 + 현재 상태 (모니터링용, 선택)
  6) 실시간 그래프

설치:
  pip install pyserial matplotlib

실행 예:
  python fdc_monitor.py --port COM3                  (기본: dp t_in t_dht rh)
  python fdc_monitor.py --port COM3 --vars dp t_in rh
  python fdc_monitor.py --port COM3 --logdir "G:/내 드라이브/FDC_logs"
  python fdc_monitor.py --port COM3 --logdir "G:/내 드라이브/FDC_logs" \
                        --sheet-url https://script.google.com/macros/s/xxxx/exec --key 내비밀키
"""

import argparse
import collections
import csv
import json
import math
import os
import statistics
import threading
import time
import urllib.request
from datetime import datetime

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

BASELINE_N = 60          # SPC 기준 학습 샘플 수 (2Hz → 30초)
WINDOW = 240             # 그래프에 보일 샘플 수 (2분)
NO_DATA_SEC = 3.0        # 이 시간 동안 데이터가 없으면 장비 무응답
META_KEYS = {"seq", "ms", "state", "err", "run", "step_t"}   # 센서값이 아닌 필드

# 변수별 최소 표준편차: 센서 분해능·노이즈보다 좁은 관리한계를 막음 (과민 알람 방지)
MIN_STD = {"t_in": 0.05, "t_bmp": 0.05, "t_sht": 0.05, "t_dht": 0.5, "t_vote": 0.05,
           "dp": 3.0, "p_in": 0.03, "p_out": 0.03, "rh": 0.3, "vib": 0.002, "dist": 0.5}
# 그래프 세로축 최소 표시 폭 (작은 노이즈가 크게 확대돼 보이지 않게)
MIN_SPAN = {"t_in": 1.0, "t_bmp": 1.0, "t_sht": 1.0, "t_dht": 2.0, "t_vote": 1.0,
            "dp": 30.0, "p_in": 0.5, "p_out": 0.5, "rh": 5.0, "vib": 0.02, "dist": 10.0}
# RUN7: 평균에서 이 폭(σ 배수) 안에 있는 점은 '중심선 위'로 보고 연속 판정에서 제외
RUN7_DEADBAND = 0.5

# 1분 요약 (AI가 큰 trace 파일 대신 읽는 작은 파일)
SUMMARY_SEC = 60
SUMMARY_VARS = ["dp", "p_in", "p_out", "t_in", "t_dht", "rh", "dist"]

plt.rcParams["font.family"] = "Malgun Gothic"   # 윈도우 한글 글꼴
plt.rcParams["axes.unicode_minus"] = False


def now_str():
    return datetime.now().isoformat(sep=" ", timespec="milliseconds")


def is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# =================================================================== SPC
class SPC:
    """정상 상태를 학습한 뒤 관리한계·추세·드리프트를 감시"""

    def __init__(self, name, lam=0.2):
        self.name = name
        self.lam = lam
        self.base = []
        self.mean = self.std = self.ewma = None
        self.side = collections.deque(maxlen=7)

    @property
    def ready(self):
        return self.mean is not None

    def limits(self):
        return self.mean - 3 * self.std, self.mean + 3 * self.std

    def update(self, v):
        if v is None:
            return []
        if not self.ready:
            self.base.append(v)
            if len(self.base) >= BASELINE_N:
                self.mean = statistics.mean(self.base)
                self.std = max(statistics.pstdev(self.base), MIN_STD.get(self.name, 1e-6))
                self.ewma = self.mean
                lcl, ucl = self.limits()
                print(f"[학습완료] {self.name}: 평균 {self.mean:.3f}, 관리한계 {lcl:.3f} ~ {ucl:.3f}")
            return []

        alarms = []
        lcl, ucl = self.limits()
        if v > ucl or v < lcl:
            alarms.append(f"SPC_OOC:{self.name}")
        d = v - self.mean
        if abs(d) <= RUN7_DEADBAND * self.std:
            self.side.append(0)            # 중심선 근처 → 한쪽 연속 끊김
        else:
            self.side.append(1 if d > 0 else -1)
        if len(self.side) == 7 and abs(sum(self.side)) == 7:
            alarms.append(f"SPC_RUN7:{self.name}")
        self.ewma = self.lam * v + (1 - self.lam) * self.ewma
        if abs(self.ewma - self.mean) > 3 * self.std * math.sqrt(self.lam / (2 - self.lam)):
            alarms.append(f"SPC_EWMA:{self.name}")
        return alarms


# ======================================================= 구글 시트 업로더
class SheetUploader:
    """10초마다 요약·알람·상태를 구글 시트(Apps Script 웹앱)로 전송.
    네트워크가 끊기면 보내지 못한 데이터를 쌓아 두었다가 다음에 한꺼번에 보냄."""

    ALARM_HEADER = ["time", "type", "alarm_id", "state", "detail"]

    def __init__(self, url, key, interval=10):
        self.url, self.key, self.interval = url, key, interval
        self.lock = threading.Lock()
        self.window = []                       # 이번 10초 동안의 샘플
        self.window_alarms = set()
        self.new_alarm_rows = []
        self.pending_summary = collections.deque(maxlen=2000)   # 미전송 보관함
        self.pending_alarms = collections.deque(maxlen=2000)
        self.keys = None
        self.status = {}
        self.ok_count = self.fail_count = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def add_sample(self, pkt, active_alarms):
        with self.lock:
            self.window.append(pkt)
            self.window_alarms |= set(active_alarms)

    def add_alarm_row(self, row):
        with self.lock:
            self.new_alarm_rows.append(row)

    def set_status(self, **kw):
        with self.lock:
            self.status.update(kw)

    def _summarize(self, samples, alarms):
        if self.keys is None:
            self.keys = [k for k in samples[0] if k not in META_KEYS]
        row = [now_str(), samples[-1]["state"], len(samples)]
        for k in self.keys:
            vals = [s.get(k) for s in samples if is_num(s.get(k))]
            row.append(round(statistics.mean(vals), 4) if vals else "")
        row.append(" | ".join(sorted(alarms)))
        return row

    def _loop(self):
        while True:
            time.sleep(self.interval)
            with self.lock:
                samples, self.window = self.window, []
                alarms, self.window_alarms = self.window_alarms, set()
                new_alarms, self.new_alarm_rows = self.new_alarm_rows, []
                status = dict(self.status)
            if samples:
                self.pending_summary.append(self._summarize(samples, alarms))
            self.pending_alarms.extend(new_alarms)
            if not self.pending_summary and not self.pending_alarms:
                continue

            summary = list(self.pending_summary)
            alarm_rows = list(self.pending_alarms)
            payload = {
                "key": self.key,
                "summary_header": ["time", "state", "samples"]
                                  + [f"{k}_avg" for k in (self.keys or [])] + ["alarms"],
                "summary": summary,
                "alarm_header": self.ALARM_HEADER,
                "alarms": alarm_rows,
                "status": [[k, str(v)] for k, v in status.items()] + [["업데이트", now_str()]],
            }
            try:
                req = urllib.request.Request(self.url, data=json.dumps(payload).encode("utf-8"),
                                             headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=20) as r:
                    resp = json.loads(r.read().decode("utf-8"))
                if not resp.get("ok"):
                    raise RuntimeError(resp.get("error", "unknown"))
                for _ in summary:
                    self.pending_summary.popleft()
                for _ in alarm_rows:
                    self.pending_alarms.popleft()
                self.ok_count += 1
            except Exception as e:
                self.fail_count += 1
                print(f"{datetime.now():%H:%M:%S} [시트] 전송 실패 (보관 중 요약 {len(self.pending_summary)}건): {e}")


# =============================================================== 1분 요약
class SummaryWriter:
    """SUMMARY_SEC마다 한 줄: 변수별 평균·최소·최대·표준편차 + 그 구간의 알람·이벤트.
    샘플이 하나도 없던 구간(장비 무응답)도 samples=0으로 기록한다."""

    def __init__(self, path):
        self.f = open(path, "w", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.f)
        head = ["time_start", "time_end", "samples", "seq_first", "seq_last",
                "n_run", "n_warn", "n_interlock"]
        for v in SUMMARY_VARS:
            head += [f"{v}_mean", f"{v}_min", f"{v}_max", f"{v}_std"]
        head += ["wafer_last", "flame_max", "dev_err", "alarm_set", "alarm_clear",
                 "events", "active_at_end", "bad_checksum", "seq_gap"]
        self.w.writerow(head)
        self.f.flush()
        self._reset()

    def _reset(self):
        self.t0 = now_str()
        self.start = time.time()
        self.n = 0
        self.seq_first = self.seq_last = None
        self.states = collections.Counter()
        self.vals = {v: [] for v in SUMMARY_VARS}
        self.wafer = ""
        self.flame = 0
        self.err = collections.Counter()
        self.sets = collections.Counter()
        self.clears = collections.Counter()
        self.events = collections.Counter()
        self.bad = 0
        self.gap = 0

    def add_sample(self, pkt):
        self.n += 1
        seq = pkt.get("seq")
        if self.seq_first is None:
            self.seq_first = seq
        self.seq_last = seq
        self.states[pkt.get("state", "")] += 1
        for v in SUMMARY_VARS:
            x = pkt.get(v)
            if is_num(x):
                self.vals[v].append(float(x))
        if "wafer" in pkt:
            self.wafer = pkt["wafer"]
        if is_num(pkt.get("flame")):
            self.flame = max(self.flame, pkt["flame"])
        for e in pkt.get("err", []):
            self.err[e] += 1

    def add_alarm(self, typ, alarm_id, detail=""):
        if typ == "SET":
            self.sets[alarm_id] += 1
        elif typ == "CLEAR":
            self.clears[alarm_id] += 1
        else:
            if alarm_id == "COMM_BAD_CHECKSUM":
                self.bad += 1
            elif alarm_id == "COMM_SEQ_GAP":
                self.gap += 1
            # 재부팅 원인처럼 detail이 중요한 이벤트는 내용까지 남김
            key = f"{alarm_id}({detail})" if alarm_id in ("DEV_BOOT", "DEV_REINIT", "DEV_WAFER") and detail else alarm_id
            self.events[key] += 1

    @staticmethod
    def _join(counter):
        return "|".join(f"{k}x{c}" if c > 1 else k for k, c in sorted(counter.items()))

    def maybe_flush(self, active, force=False):
        if not force and time.time() - self.start < SUMMARY_SEC:
            return
        row = [self.t0, now_str(), self.n, self.seq_first, self.seq_last,
               self.states.get("RUN", 0), self.states.get("WARN", 0), self.states.get("INTERLOCK", 0)]
        for v in SUMMARY_VARS:
            xs = self.vals[v]
            if xs:
                row += [round(statistics.mean(xs), 3), round(min(xs), 3), round(max(xs), 3),
                        round(statistics.pstdev(xs), 3)]
            else:
                row += ["", "", "", ""]
        row += [self.wafer, self.flame, self._join(self.err), self._join(self.sets),
                self._join(self.clears), self._join(self.events), "|".join(sorted(active)),
                self.bad, self.gap]
        self.w.writerow(row)
        self.f.flush()
        self._reset()


# ================================================================ 수신부
class Monitor:
    def __init__(self, port, baud, logdir, plot_vars, uploader):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.ser.reset_input_buffer()   # 시작 전에 쌓인 오래된 데이터 버리기 (체크섬 오류 몰림 방지)
        self.lock = threading.Lock()
        self.plot_vars = plot_vars
        self.spc = {}
        self.hist = {v: collections.deque(maxlen=WINDOW) for v in plot_vars}
        self.uploader = uploader
        self.state = "WAIT"
        self.active = set()          # 현재 걸려 있는 알람 (SET 상태)
        self.events_recent = collections.deque(maxlen=4)
        self.last_rx = time.time()
        self.no_data = False
        self.last_seq = None
        self.stats = collections.Counter()

        os.makedirs(logdir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trace_path = os.path.join(logdir, f"trace_{stamp}.csv")
        self.alarm_path = os.path.join(logdir, f"alarms_{stamp}.csv")
        self.trace_f = open(self.trace_path, "w", newline="", encoding="utf-8-sig")
        self.trace_w = None                                  # 첫 패킷을 보고 열 구성
        self.alarm_f = open(self.alarm_path, "w", newline="", encoding="utf-8-sig")
        self.alarm_w = csv.writer(self.alarm_f)
        self.alarm_w.writerow(SheetUploader.ALARM_HEADER)
        self.summary_path = os.path.join(logdir, f"summary_{stamp}.csv")
        self.summary = SummaryWriter(self.summary_path)
        print(f"원본 로그 : {self.trace_path}\n알람 이력 : {self.alarm_path}\n1분 요약 : {self.summary_path}")

        threading.Thread(target=self.reader, daemon=True).start()

    # ---------- 알람 기록 ----------
    def log_alarm(self, typ, alarm_id, detail=""):
        row = [now_str(), typ, alarm_id, self.state, detail]
        self.alarm_w.writerow(row)
        self.alarm_f.flush()
        if self.uploader:
            self.uploader.add_alarm_row(row)
        self.summary.add_alarm(typ, alarm_id, detail)
        print(f"{datetime.now():%H:%M:%S} [{typ:5}] {alarm_id} {detail}")
        if typ != "CLEAR":
            self.events_recent.append(alarm_id)

    def update_alarms(self, now_set):
        for a in sorted(now_set - self.active):
            self.log_alarm("SET", a)
        for a in sorted(self.active - now_set):
            self.log_alarm("CLEAR", a)
        self.active = now_set

    # ---------- 패킷 검사 ----------
    @staticmethod
    def parse(line):
        """'{...}*XX' 형식. 정상이면 dict, 체크섬 오류면 'BAD', 데이터 줄이 아니면 None"""
        if "*" not in line:
            return None
        body, cs = line.rsplit("*", 1)
        calc = 0
        for b in body.encode():
            calc ^= b
        try:
            if int(cs, 16) != calc:
                return "BAD"
            return json.loads(body)
        except ValueError:
            return "BAD"

    # ---------- 수신 루프 ----------
    def reader(self):
        while True:
            raw = self.ser.readline()
            self.summary.maybe_flush(self.active | ({"HOST_NO_DATA"} if self.no_data else set()))

            # 무응답 감시 (readline은 1초마다 깨어남)
            silent = time.time() - self.last_rx > NO_DATA_SEC
            if silent and not self.no_data:
                self.no_data = True
                self.log_alarm("SET", "HOST_NO_DATA", "장비 응답 없음")
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()

            if line.startswith("#"):                         # 장비가 보낸 이벤트
                self.log_alarm("EVENT", "DEV_" + line[1:].split()[0], line[1:])
                if line.startswith("#BOOT"):
                    self.stats["reboot"] += 1
                    self.last_seq = None
                continue

            pkt = self.parse(line)
            if pkt is None:
                continue
            if pkt == "BAD":
                self.stats["bad_checksum"] += 1
                self.log_alarm("EVENT", "COMM_BAD_CHECKSUM", f"누적 {self.stats['bad_checksum']}회")
                continue

            if self.no_data:
                self.no_data = False
                self.log_alarm("CLEAR", "HOST_NO_DATA")

            seq = pkt["seq"]
            if self.last_seq is not None and seq != self.last_seq + 1:
                if seq < self.last_seq:
                    self.log_alarm("EVENT", "DEV_REBOOTED")
                else:
                    self.log_alarm("EVENT", "COMM_SEQ_GAP", f"{seq - self.last_seq - 1}개 누락")
            self.last_seq = seq

            spc_alarms = []
            with self.lock:
                self.last_rx = time.time()
                self.state = pkt["state"]
                for k, v in pkt.items():
                    if k in META_KEYS or not (is_num(v) or v is None):
                        continue
                    if k in self.plot_vars:
                        spc_alarms += self.spc.setdefault(k, SPC(k)).update(v)
                        self.hist[k].append(math.nan if v is None else v)

            now_set = set(spc_alarms) | {f"DEV_{e}" for e in pkt["err"]}
            self.update_alarms(now_set)

            # 원본 CSV
            if self.trace_w is None:
                fields = ["time"] + [k for k in pkt if k != "err"] + ["dev_err", "host_alarm"]
                self.trace_w = csv.DictWriter(self.trace_f, fieldnames=fields, extrasaction="ignore")
                self.trace_w.writeheader()
            rec = {k: v for k, v in pkt.items() if k != "err"}
            rec.update(time=now_str(), dev_err="|".join(pkt["err"]), host_alarm="|".join(spc_alarms))
            self.trace_w.writerow(rec)
            self.trace_f.flush()
            self.summary.add_sample(pkt)

            if self.uploader:
                self.uploader.add_sample(pkt, now_set)
                self.uploader.set_status(장비상태=pkt["state"],
                                         현재알람=" | ".join(sorted(now_set)) or "없음",
                                         체크섬오류=self.stats["bad_checksum"],
                                         재부팅=self.stats["reboot"])


# ================================================================ 그래프
def run(args):
    uploader = SheetUploader(args.sheet_url, args.key) if args.sheet_url else None
    mon = Monitor(args.port, args.baud, args.logdir, args.vars, uploader)

    fig, axes = plt.subplots(len(args.vars), 1, figsize=(10, 2.6 * len(args.vars)), sharex=True)
    if len(args.vars) == 1:
        axes = [axes]

    def draw(_):
        with mon.lock:
            for ax, v in zip(axes, args.vars):
                ax.clear()
                ax.plot(list(mon.hist[v]), color="tab:blue", lw=1.2)
                s = mon.spc.get(v)
                if s and s.ready:
                    lcl, ucl = s.limits()
                    ax.axhline(s.mean, color="green", ls="--", lw=1)
                    ax.axhline(ucl, color="red", ls="--", lw=1)
                    ax.axhline(lcl, color="red", ls="--", lw=1)
                else:
                    n = len(s.base) if s else 0
                    ax.text(0.01, 0.85, f"기준 학습 중 {n}/{BASELINE_N}",
                            transform=ax.transAxes, color="gray")
                ax.set_ylabel(v)
                ax.ticklabel_format(axis="y", useOffset=False)
                vals = [x for x in mon.hist[v] if not math.isnan(x)]
                if vals:
                    lo, hi = min(vals), max(vals)
                    if s and s.ready:
                        lo, hi = min(lo, s.limits()[0]), max(hi, s.limits()[1])
                    span = MIN_SPAN.get(v, 0)
                    if hi - lo < span:
                        mid = (hi + lo) / 2
                        lo, hi = mid - span / 2, mid + span / 2
                    pad = (hi - lo) * 0.1
                    ax.set_ylim(lo - pad, hi + pad)

            if mon.no_data:
                title, color = "HOST_NO_DATA: 장비 응답 없음", "red"
            else:
                act = sorted(mon.active)
                color = "red" if mon.state == "INTERLOCK" else ("orange" if act else "green")
                title = f"상태: {mon.state}" + (f"  |  {', '.join(act[:4])}" if act else "")
            fig.suptitle(title, color=color, fontsize=13)

            sheet = ""
            if uploader:
                sheet = f"   시트 전송 성공 {uploader.ok_count} / 실패 {uploader.fail_count}"
            axes[-1].set_xlabel(f"샘플 (2Hz)   체크섬 오류 {mon.stats['bad_checksum']}"
                                f"   재부팅 {mon.stats['reboot']}{sheet}")

    _ani = FuncAnimation(fig, draw, interval=500, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="미니 공정 챔버 FDC 서버")
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--logdir", default="logs", help="CSV 저장 폴더 (구글 드라이브 폴더 지정 가능)")
    ap.add_argument("--vars", nargs="+", default=["dp", "t_in", "t_dht", "rh"], help="그래프·SPC로 감시할 변수")
    ap.add_argument("--sheet-url", default=None, help="Apps Script 웹앱 URL (없으면 시트 업로드 생략)")
    ap.add_argument("--key", default="", help="Apps Script에 설정한 비밀키")
    run(ap.parse_args())
