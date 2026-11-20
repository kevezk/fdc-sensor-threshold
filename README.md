# 센서 잡음과 알람 문턱 정하기 — 미니 FDC 장비

## ▶ [3D 챔버 재생 화면 열기](https://kevezk.github.io/fdc-sensor-threshold/chamber.html)

실제 측정 기록을 3D 챔버 위에서 재생하며, 처음 설정과 최종 설정의 알람 차이와
+10 Pa 가상 주입 시 계단 검출기가 울리는 모습을 볼 수 있습니다.

ESP32 + BMP280 ×2 + DHT11로 만든 반도체 팹 FDC(이상 감지) 축소 모델에서,
정상 상태 오경보를 시간당 602회 → 3회로 줄이고 다른 기록으로 검증한 프로젝트입니다.

- 분석 보고서: https://kevezk.github.io/fdc-sensor-threshold/
- `replay.py` : 기록(trace CSV)을 다시 돌려 설정별 알람 비교
- `validate.py` : 독립 검증, 직접 만든 이상, 계단 주입 시험
- `trace_20261119_182333.csv` : 첫 번째 기록 (약 10분)
- `trace_20261119_185552.csv` : 두 번째 기록 (정상 30분 + 이상 만들기)

실행: `python validate.py trace_20261119_185552.csv --old trace_20261119_182333.csv`

정현 · 동아대학교 전자공학과
