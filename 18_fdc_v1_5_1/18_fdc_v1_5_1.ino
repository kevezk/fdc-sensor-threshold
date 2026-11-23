// =====================================================================
//  미니 공정 챔버 FDC 노드 v1.5.1 (ESP32-32D) - 통합본
//  - 차압(BMP280 x2) + 영점 보정, 온도, 습도(DHT11), 웨이퍼 감지(초음파),
//    화재 인터락, 센서 자가진단/오류코드, 체크섬 데이터 → PC(fdc_monitor.py)
//
//  라이브러리: Adafruit BMP280, Adafruit GC9A01A, Adafruit GFX, DHT sensor library
//
//  연결 (보라색 확장보드, 점퍼 3.3V):
//    BMP280 #1 (챔버 내부 0x76)  SDA→P21 S, SCL→P22 S, SDO→GND
//    BMP280 #2 (챔버 외부 0x77)  SDA→P26 S, SCL→P27 S, CSB·SDO→3.3V
//    불꽃 센서 DO→P35 S          부저 I/O→P32 S
//    DHT11 DATA→P13 S (+ V, - G)
//    초음파 Trig→P14 S, Echo→P34 S, VCC→왼쪽아래 3V, GND→왼쪽아래 GND
//      ※ P12 줄은 절대 사용 금지 (부팅 스트래핑 핀)
//    원형 디스플레이 SDA→23, SCL→18, CS→5, DC→16, RST→17
//
//  조작: BOOT 버튼 2초 / 시리얼 "ZERO" → 차압 영점 보정 (플래시에 저장)
//
//  오류코드 0xDDTT
//    DD: 01 BMP_IN  02 BMP_OUT  03 MPU  04 FLAME  05 DHT  06 SONAR
//    TT: 01 NOT_FOUND  02 READ_FAIL  03 RANGE  04 STUCK  05 MISMATCH  10 INTERLOCK
// =====================================================================

#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <DHT.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <esp_system.h>

// ---------- 사용할 센서 (연결 안 한 센서는 0) ----------
#define USE_BMP_OUT 1
#define USE_MPU     0      // 납땜 후 1
#define USE_DHT     1
#define USE_SONAR   1

// ---------- 핀 ----------
#define FLAME_PIN  35
#define BUZZER_PIN 32
#define BOOT_PIN   0
#define DHT_PIN    13
#define TRIG_PIN   14
#define ECHO_PIN   34
#define TFT_CS     5
#define TFT_DC     16
#define TFT_RST    17

#define ADDR_BMP_IN  0x76
#define ADDR_BMP_OUT 0x77
#define ADDR_MPU     0x68

// ---------- 주기 / 기준값 ----------
const uint32_t REPORT_MS   = 500;    // PC 보고 (2Hz)
const uint32_t RETRY_MS    = 3000;   // 빠진 센서 재연결
const uint32_t VIB_MS      = 2;      // 진동 샘플링 (500Hz)
const uint32_t DHT_MS      = 2000;   // DHT11은 1초 이상 간격 필요
const uint32_t SONAR_MS    = 100;    // 초음파 1회 측정 간격
const int      STUCK_LIMIT = 20;

const float P_MIN = 900.0, P_MAX = 1100.0;        // 정상 대기압 범위 (hPa)
const int   ZERO_SAMPLES = 20;                    // 영점 평균 개수 (10초)
const float GAUGE_RANGE = 50, DEV_WARN = 15, DEV_BAD = 30;   // 차압 게이지 (Pa)

const float WAFER_ON_CM  = 8.0;      // 이보다 가까우면 웨이퍼 있음
const float WAFER_OFF_CM = 10.0;     // 이보다 멀면 없음 (히스테리시스)
const float SONAR_MAX_CM = 400.0;    // 이 이상은 "물체 없음"
const float T_MISMATCH   = 5.0;      // DHT11 vs BMP 온도 차 허용 (°C)

// ---------- 오류 코드 ----------
enum Dev     { DEV_BMP_IN = 0x01, DEV_BMP_OUT = 0x02, DEV_MPU = 0x03,
               DEV_FLAME = 0x04, DEV_DHT = 0x05, DEV_SONAR = 0x06 };
enum ErrType { E_NOT_FOUND = 0x01, E_READ_FAIL = 0x02, E_RANGE = 0x03,
               E_STUCK = 0x04, E_MISMATCH = 0x05, E_INTERLOCK = 0x10 };

// ---------- 색상 ----------
#define RGB(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
const uint16_t C_BG    = 0x0000;
const uint16_t C_TRACK = RGB(15, 18, 24);
const uint16_t C_DIM   = RGB(130, 140, 155);
const uint16_t C_TEXT  = RGB(235, 240, 245);
const uint16_t C_OK    = RGB(40, 210, 160);
const uint16_t C_WARN  = RGB(255, 176, 32);
const uint16_t C_BAD   = RGB(255, 70, 70);
const uint16_t C_CAL   = RGB(90, 160, 255);

// ---------- 객체 ----------
Adafruit_BMP280 bmpIn;
Adafruit_BMP280 bmpOut(&Wire1);
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 cv(150, 140);
Preferences prefs;
#if USE_DHT
DHT dht(DHT_PIN, DHT11);
#endif

struct Health { bool present = false; int stuckCnt = 0; float last = NAN; };
Health hIn, hOut, hMpu;

uint16_t errs[10];
int nErr = 0;
void addErr(uint8_t dev, uint8_t type) { if (nErr < 10) errs[nErr++] = (dev << 8) | type; }

uint32_t seq = 0, lastReport = 0, lastRetry = 0, lastVib = 0, lastFlameMs = 0;
double vibSumSq = 0;
uint32_t vibN = 0, vibFail = 0;
bool fire = false;

// 영점 보정
float dpOffset = 0;
bool zeroing = false;
int zN = 0;
double zSum = 0;
uint32_t bootPressMs = 0;
bool bootHandled = false;
String serialBuf = "";

// DHT11
float tDht = NAN, rh = NAN;
int dhtFail = 0;
uint32_t lastDht = 0;

// 초음파
float sonarBuf[3] = {NAN, NAN, NAN};
int sonarIdx = 0;
int noEchoCnt = 0, echoHighCnt = 0;
uint32_t lastSonar = 0;
float distCm = NAN;     // 중간값 (물체 없으면 NAN)
bool wafer = false;

// 화면
uint16_t segColor[54];
bool heartbeat = false;

// =====================================================================
bool i2cPresentOn(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

void initSensors() {
  if (!hIn.present) hIn.present = bmpIn.begin(ADDR_BMP_IN);
#if USE_BMP_OUT
  if (!hOut.present) hOut.present = bmpOut.begin(ADDR_BMP_OUT);
#endif
#if USE_MPU
  if (!hMpu.present && i2cPresentOn(Wire, ADDR_MPU)) {
    Wire.beginTransmission(ADDR_MPU);
    Wire.write(0x6B); Wire.write(0x00);
    hMpu.present = (Wire.endTransmission() == 0);
  }
#endif
}

bool readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADDR_MPU, (uint8_t)6) != 6) return false;
  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  ax = x / 16384.0; ay = y / 16384.0; az = z / 16384.0;
  return true;
}

void checkStuck(Health &h, float v, uint8_t dev) {
  if (v == h.last) { if (++h.stuckCnt >= STUCK_LIMIT) addErr(dev, E_STUCK); }
  else h.stuckCnt = 0;
  h.last = v;
}

// BMP280 읽기 + 범위 검사 + 이상하면 즉시 재초기화
void readBmp(Adafruit_BMP280 &bmp, TwoWire &bus, uint8_t addr, Health &h,
             uint8_t dev, const char *name, float &p, float &t) {
  p = NAN; t = NAN;
  if (h.present && !i2cPresentOn(bus, addr)) h.present = false;
  if (!h.present) { addErr(dev, E_NOT_FOUND); return; }

  t = bmp.readTemperature();
  p = bmp.readPressure() / 100.0;
  if (isnan(p) || p < P_MIN || p > P_MAX) {
    Serial.printf("#REINIT %s p=%.1f\n", name, p);
    if (bmp.begin(addr)) {
      t = bmp.readTemperature();
      p = bmp.readPressure() / 100.0;
    }
    if (isnan(p) || p < P_MIN || p > P_MAX) {
      addErr(dev, E_RANGE);
      h.present = false;
      p = NAN; t = NAN;
      return;
    }
  }
  checkStuck(h, p, dev);
}

String num(float v, int d) { return isnan(v) ? String("null") : String(v, d); }

void buzzerOff() { noTone(BUZZER_PIN); digitalWrite(BUZZER_PIN, HIGH); }

// ---------- 영점 보정 ----------
void startZero(const char *by) {
  zeroing = true; zN = 0; zSum = 0;
  Serial.printf("#ZERO_START by=%s\n", by);
}

void feedZero(float dpRaw) {
  if (!zeroing || isnan(dpRaw)) return;
  zSum += dpRaw;
  if (++zN >= ZERO_SAMPLES) {
    dpOffset = zSum / zN;
    zeroing = false;
    prefs.putFloat("dpofs", dpOffset);
    Serial.printf("#ZERO_DONE ofs=%.1f\n", dpOffset);
  }
}

void checkZeroRequest(uint32_t ms) {
  if (digitalRead(BOOT_PIN) == LOW) {
    if (bootPressMs == 0) bootPressMs = ms;
    if (!bootHandled && ms - bootPressMs > 2000) { bootHandled = true; startZero("button"); }
  } else { bootPressMs = 0; bootHandled = false; }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf.trim();
      serialBuf.toUpperCase();
      if (serialBuf == "ZERO") startZero("serial");
      serialBuf = "";
    } else if (serialBuf.length() < 20) serialBuf += c;
  }
}

// ---------- DHT11 (2초마다) ----------
void pollDht(uint32_t ms) {
#if USE_DHT
  if (ms - lastDht < DHT_MS) return;
  lastDht = ms;
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    dhtFail++;
    if (dhtFail >= 3) { tDht = NAN; rh = NAN; }   // 3번 연속 실패 시 값 버림
  } else {
    dhtFail = 0; tDht = t; rh = h;
  }
#endif
}

// ---------- 초음파 (0.1초마다 1회, 최근 3회 중간값) ----------
float median3(float a, float b, float c) {
  if (isnan(a) || isnan(b) || isnan(c)) return NAN;
  if (a > b) { float x = a; a = b; b = x; }
  if (b > c) { float x = b; b = c; c = x; }
  if (a > b) { float x = a; a = b; b = x; }
  return b;
}

void pollSonar(uint32_t ms) {
#if USE_SONAR
  if (ms - lastSonar < SONAR_MS) return;
  lastSonar = ms;

  // Echo가 트리거 전부터 HIGH면 선/센서 이상
  if (digitalRead(ECHO_PIN) == HIGH) { echoHighCnt++; return; }
  echoHighCnt = 0;

  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 40000);   // 40ms 대기 (물체 없음 펄스 ~26ms 포함)

  float cm;
  if (dur == 0) { noEchoCnt++; cm = NAN; }      // 응답 자체가 없음
  else {
    noEchoCnt = 0;
    cm = dur * 0.0343 / 2;
    if (cm > SONAR_MAX_CM) cm = SONAR_MAX_CM;    // 물체 없음 / 튀는 값
  }
  if (!isnan(cm)) { sonarBuf[sonarIdx] = cm; sonarIdx = (sonarIdx + 1) % 3; }

  float m = median3(sonarBuf[0], sonarBuf[1], sonarBuf[2]);
  distCm = (isnan(m) || m >= SONAR_MAX_CM) ? NAN : m;

  // 웨이퍼 감지 (히스테리시스)
  bool prev = wafer;
  if (!isnan(distCm) && distCm < WAFER_ON_CM) wafer = true;
  else if (isnan(distCm) || distCm > WAFER_OFF_CM) wafer = false;
  if (wafer != prev) Serial.printf("#WAFER %s dist=%.1f\n", wafer ? "LOAD" : "UNLOAD", distCm);
#endif
}

// =====================================================================
//  화면
// =====================================================================
void centerText(const char *s, int cx, int baseline) {
  int16_t x1, y1; uint16_t w, h;
  cv.getTextBounds(s, 0, baseline, &x1, &y1, &w, &h);
  cv.setCursor(cx - (int)w / 2 - x1, baseline);
  cv.print(s);
}

void drawSegment(int i, uint16_t col) {
  const float CX = 120, CY = 120, R0 = 104, R1 = 116;
  float a0 = (135 + i * 5 + 0.6) * DEG_TO_RAD;
  float a1 = (135 + i * 5 + 4.4) * DEG_TO_RAD;
  int x0 = CX + R0 * cos(a0), y0 = CY + R0 * sin(a0);
  int x1 = CX + R1 * cos(a0), y1 = CY + R1 * sin(a0);
  int x2 = CX + R1 * cos(a1), y2 = CY + R1 * sin(a1);
  int x3 = CX + R0 * cos(a1), y3 = CY + R0 * sin(a1);
  tft.fillTriangle(x0, y0, x1, y1, x2, y2, col);
  tft.fillTriangle(x0, y0, x2, y2, x3, y3, col);
}

void drawRing(float dp, uint16_t col) {
  const int MID = 27;
  int target = MID;
  if (!isnan(dp)) {
    float f = constrain(dp / GAUGE_RANGE, -1.0, 1.0);
    target = MID + (int)round(f * 27);
    target = constrain(target, 0, 53);
  }
  int lo = min(MID, target), hi = max(MID, target);
  for (int i = 0; i < 54; i++) {
    uint16_t c = (i >= lo && i <= hi && !isnan(dp)) ? col : C_TRACK;
    if (i == MID && isnan(dp)) c = C_DIM;
    if (c != segColor[i]) { drawSegment(i, c); segColor[i] = c; }
  }
}

// 아래쪽 센서 상태: 이름 + 점(초록 정상 / 빨강 이상)
void statusItem(int &x, const char *label, bool ok) {
  tft.setCursor(x, 198);
  tft.print(label);
  x += strlen(label) * 6 + 5;
  tft.fillCircle(x, 201, 3, ok ? C_OK : C_BAD);
  x += 11;
}

void drawUI(const String &state, float dp, float tIn, float pIn) {
  uint16_t stCol = (state == "RUN") ? C_OK : (state == "WARN") ? C_WARN : C_BAD;
  const char *stText = (state == "RUN") ? "NORMAL" : (state == "WARN") ? "WARNING" : "INTERLOCK";
  float adp = fabs(dp);
  uint16_t dpCol = zeroing ? C_CAL : fire ? C_BAD
                 : (isnan(dp) ? C_DIM : adp < DEV_WARN ? C_OK : adp < DEV_BAD ? C_WARN : C_BAD);

  drawRing(zeroing ? NAN : dp, dpCol);

  char buf[32];
  cv.fillScreen(C_BG);
  cv.cp437(true);

  // 장비 상태
  cv.setFont(&FreeSansBold9pt7b);
  cv.setTextColor(stCol);
  {
    int16_t x1, y1; uint16_t w, h;
    cv.getTextBounds(stText, 0, 14, &x1, &y1, &w, &h);
    int sx = 75 - (int)(w + 12) / 2;
    cv.fillCircle(sx + 4, 9, 4, stCol);
    cv.setCursor(sx + 12 - x1, 14);
    cv.print(stText);
  }

  // 고장 코드
  cv.setFont(NULL);
  cv.setTextSize(1);
  if (nErr == 0) { cv.setTextColor(C_DIM); centerText("NO FAULT", 75, 20); }
  else {
    cv.setTextColor(stCol);
    if (nErr == 1) snprintf(buf, sizeof(buf), "FAULT %04X", errs[0]);
    else           snprintf(buf, sizeof(buf), "FAULT %04X +%d", errs[0], nErr - 1);
    centerText(buf, 75, 20);
  }

  // 차압
  cv.setTextColor(C_DIM);
  centerText("DIFF PRESSURE (Pa)", 75, 34);
  cv.setFont(&FreeSansBold18pt7b);
  cv.setTextColor(dpCol);
  if (zeroing)        snprintf(buf, sizeof(buf), "CAL");
  else if (isnan(dp)) snprintf(buf, sizeof(buf), "---");
  else                snprintf(buf, sizeof(buf), "%+.1f", dp);
  centerText(buf, 75, 72);

  cv.setFont(NULL);
  cv.setTextColor(zeroing ? C_CAL : C_DIM);
  if (zeroing) snprintf(buf, sizeof(buf), "CALIBRATING %d/%d", zN, ZERO_SAMPLES);
  else         snprintf(buf, sizeof(buf), "OFFSET %.1f Pa", dpOffset);
  centerText(buf, 75, 80);

  // 온도 / 습도 / 기압  (이름 - 값 - 단위)
  cv.setTextColor(C_DIM);
  centerText("TEMP", 25, 92);
  centerText("HUMID", 75, 92);
  centerText("PRESS", 125, 92);
  cv.setFont(&FreeSans9pt7b);
  cv.setTextColor(C_TEXT);
  if (isnan(tIn)) snprintf(buf, sizeof(buf), "--"); else snprintf(buf, sizeof(buf), "%.1f", tIn);
  centerText(buf, 25, 114);
  if (isnan(rh))  snprintf(buf, sizeof(buf), "--"); else snprintf(buf, sizeof(buf), "%.0f", rh);
  centerText(buf, 75, 114);
  if (isnan(pIn)) snprintf(buf, sizeof(buf), "--"); else snprintf(buf, sizeof(buf), "%.0f", pIn);
  centerText(buf, 125, 114);
  cv.setFont(NULL);
  cv.setTextColor(C_DIM);
  centerText("\xF8" "C", 25, 118);
  centerText("%RH", 75, 118);
  centerText("hPa", 125, 118);

  // 웨이퍼
  if (wafer) { cv.setTextColor(C_CAL); centerText("WAFER PRESENT", 75, 131); }
  else       { cv.setTextColor(C_DIM); centerText("NO WAFER", 75, 131); }

  tft.drawRGBBitmap(45, 50, cv.getBuffer(), 150, 140);

  // 위쪽: 장비 이름 + 하트비트
  heartbeat = !heartbeat;
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(C_DIM, C_BG);
  tft.setCursor(102, 30);
  tft.print("FDC-01");
  tft.fillCircle(144, 33, 2, heartbeat ? C_OK : C_BG);

  // 아래쪽: 센서 상태
  int x = 36;
  statusItem(x, "IN",   hIn.present);
  statusItem(x, "OUT",  hOut.present || !USE_BMP_OUT);
  statusItem(x, "HUM",  !isnan(rh) || !USE_DHT);
  statusItem(x, "DIST", (noEchoCnt < 30 && echoHighCnt < 10) || !USE_SONAR);
  statusItem(x, "FIRE", !fire);
}

// =====================================================================
//  2Hz 보고
// =====================================================================
void report() {
  nErr = 0;
  float pIn, tIn, pOut = NAN, tOut = NAN, vib = NAN;

  readBmp(bmpIn, Wire, ADDR_BMP_IN, hIn, DEV_BMP_IN, "BMP_IN", pIn, tIn);
#if USE_BMP_OUT
  readBmp(bmpOut, Wire1, ADDR_BMP_OUT, hOut, DEV_BMP_OUT, "BMP_OUT", pOut, tOut);
#endif

  float dpRaw = (!isnan(pIn) && !isnan(pOut)) ? (pIn - pOut) * 100.0 : NAN;
  feedZero(dpRaw);
  float dp = (zeroing || isnan(dpRaw)) ? NAN : dpRaw - dpOffset;

#if USE_MPU
  if (hMpu.present && !i2cPresentOn(Wire, ADDR_MPU)) hMpu.present = false;
  if (hMpu.present) {
    if (vibN > 0) { vib = sqrt(vibSumSq / vibN); checkStuck(hMpu, vib, DEV_MPU); }
    if (vibFail > vibN) addErr(DEV_MPU, E_READ_FAIL);
  } else addErr(DEV_MPU, E_NOT_FOUND);
#endif
  vibSumSq = 0; vibN = 0; vibFail = 0;

#if USE_DHT
  if (dhtFail >= 3) addErr(DEV_DHT, E_READ_FAIL);
  else if (!isnan(rh)) {
    if (tDht < 0 || tDht > 50 || rh < 5 || rh > 95) addErr(DEV_DHT, E_RANGE);
    if (!isnan(tIn) && fabs(tDht - tIn) > T_MISMATCH) addErr(DEV_DHT, E_MISMATCH);   // 센서 교차검증
  }
#endif

#if USE_SONAR
  if (echoHighCnt >= 10) addErr(DEV_SONAR, E_STUCK);          // Echo가 HIGH로 고정
  else if (noEchoCnt >= 30) addErr(DEV_SONAR, E_NOT_FOUND);   // 3초간 응답 없음
#endif

  if (fire) addErr(DEV_FLAME, E_INTERLOCK);

  String state = fire ? "INTERLOCK" : (nErr > 0 ? "WARN" : "RUN");

  String body = "{\"seq\":" + String(seq++) +
                ",\"ms\":" + String(millis()) +
                ",\"p_in\":" + num(pIn, 2) +
                ",\"p_out\":" + num(pOut, 2) +
                ",\"dp\":" + num(dp, 1) +
                ",\"dp_raw\":" + num(dpRaw, 1) +
                ",\"t_in\":" + num(tIn, 2) +
                ",\"t_out\":" + num(tOut, 2) +
                ",\"t_dht\":" + num(tDht, 1) +
                ",\"rh\":" + num(rh, 0) +
                ",\"dist\":" + num(distCm, 1) +
                ",\"wafer\":" + String(wafer ? 1 : 0) +
                ",\"vib\":" + num(vib, 4) +
                ",\"flame\":" + String(fire ? 1 : 0) +
                ",\"state\":\"" + state + "\",\"err\":[";
  for (int i = 0; i < nErr; i++) {
    char hb[8]; snprintf(hb, sizeof(hb), "%04X", errs[i]);
    body += String(i ? "," : "") + "\"" + hb + "\"";
  }
  body += "]}";

  uint8_t cs = 0;
  for (unsigned int i = 0; i < body.length(); i++) cs ^= (uint8_t)body[i];
  Serial.printf("%s*%02X\n", body.c_str(), cs);

  drawUI(state, dp, tIn, pIn);
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(FLAME_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  buzzerOff();

  Wire.begin(21, 22);
  Wire1.begin(26, 27);
  Wire.setTimeOut(20);
  Wire1.setTimeOut(20);
#if USE_DHT
  dht.begin();
#endif

  tft.begin();
  tft.fillScreen(C_BG);
  for (int i = 0; i < 54; i++) segColor[i] = 0xFFFF;   // 첫 그리기 강제

  int rr = (int)esp_reset_reason();
  const char *rname = (rr == ESP_RST_POWERON) ? "POWERON"
                    : (rr == ESP_RST_BROWNOUT) ? "BROWNOUT"
                    : (rr == ESP_RST_PANIC) ? "PANIC"
                    : (rr == ESP_RST_SW) ? "SOFTWARE"
                    : (rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT) ? "WATCHDOG"
                    : (rr == ESP_RST_EXT) ? "EXTERNAL" : "OTHER";
  Serial.printf("#BOOT reason=%s fw=1.5.1\n", rname);

  initSensors();
  Serial.printf("#SENSORS bmp_in=%d bmp_out=%d mpu=%d dht=%d sonar=%d\n",
                hIn.present, hOut.present, hMpu.present, USE_DHT, USE_SONAR);

  prefs.begin("fdc", false);
  if (prefs.isKey("dpofs")) {
    dpOffset = prefs.getFloat("dpofs", 0);
    Serial.printf("#ZERO_LOAD ofs=%.1f\n", dpOffset);
  } else startZero("auto");
}

void loop() {
  uint32_t ms = millis();

#if USE_MPU
  if (hMpu.present && ms - lastVib >= VIB_MS) {
    lastVib = ms;
    float ax, ay, az;
    if (readAccel(ax, ay, az)) {
      double d = sqrt(ax * ax + ay * ay + az * az) - 1.0;
      vibSumSq += d * d; vibN++;
    } else vibFail++;
  }
#endif

  // 화재: LOW = 불꽃, 마지막 감지 후 1초 유지
  if (digitalRead(FLAME_PIN) == LOW) lastFlameMs = ms;
  fire = (lastFlameMs != 0) && (ms - lastFlameMs < 1000);
  if (fire) tone(BUZZER_PIN, ((ms / 250) % 2) ? 2400 : 1500);
  else      buzzerOff();

  checkZeroRequest(ms);
  pollDht(ms);
  pollSonar(ms);

  if (ms - lastRetry > RETRY_MS) { lastRetry = ms; initSensors(); }
  if (ms - lastReport >= REPORT_MS) { lastReport = ms; report(); }
}