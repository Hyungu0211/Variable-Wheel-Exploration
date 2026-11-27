// 가변 휠 1개 캠 구동 시험 — Arduino Uno/Nano (ATmega328P, CH340) 2026-11-25
// 서보: 360° 연속회전 마이크로 서보(TS90D360 / DF9GMS 360) → 웜 → 캠판
//   연속회전 서보는 각도가 아니라 "속도" 제어: 1500 us 정지, 크면 한쪽, 작으면 반대쪽 회전
//
// 배선 (Uno/Nano 공통):
//   서보 신호(주황/노랑) → D9
//   서보 +(빨강)        → 5V   (시험은 보드 5V 로 가능. 힘이 모자라면 외부 5~6 V 로 분리, GND 공통)
//   서보 −(갈색/검정)   → GND
//
// 조작 (시리얼 115200. 아두이노 시리얼 모니터는 e + Enter 또는 Enter만(빈 줄), host/space_test.py 는 스페이스바 바로):
//   [스페이스]  1번째: 확장 시작  → 2번째: 확장 멈춤(걸린 시간 기억)
//               3번째: 기억한 시간만큼 역회전(수축) 후 자동 정지 → 다시 1번째로
//   x           즉시 정지, 상태 초기화(수축 상태로 간주)
//   + / -       회전 속도(펄스 폭 차) 50 us 씩 증감 (기본 200 us)
//   [ / ]       정지 펄스 미세조정 −10/+10 us (정지인데 조금씩 돌면 조정)
//   ?           현재 상태 출력

#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const int     EXPAND_DIR = +1;      // 스페이스 첫 번째에 바퀴가 "접히면" -1 로 바꾼다
int  stopUs  = 1500;                // 정지 펄스 [확인: 서보마다 1480~1520 사이에서 미세조정]
int  speedUs = 200;                 // 정지 대비 펄스 차 (클수록 빠름, 최대 500)
const uint32_t MAX_RUN_MS = 20000;  // 안전: 확장이 20 s 넘으면 자동 정지

Servo servo;
enum State { CONTRACTED, EXPANDING, EXPANDED, CONTRACTING };
State    st = CONTRACTED;
uint32_t tStart = 0;        // 현재 동작 시작 시각
uint32_t tExpand = 0;       // 확장에 걸린 시간(ms) → 수축에 그대로 사용

void drive(int dir) { servo.writeMicroseconds(stopUs + dir * EXPAND_DIR * speedUs); }   // dir +1 확장, -1 수축
void stopServo()    { servo.writeMicroseconds(stopUs); }

const char* name(State s) {
  switch (s) { case CONTRACTED: return "수축(대기)"; case EXPANDING: return "확장 중"; case EXPANDED: return "확장(멈춤)"; default: return "수축 중"; }
}
void report() {
  Serial.print(F("상태=")); Serial.print(name(st));
  Serial.print(F("  확장시간=")); Serial.print(tExpand); Serial.print(F(" ms"));
  Serial.print(F("  속도=")); Serial.print(speedUs); Serial.print(F(" us  정지=")); Serial.print(stopUs); Serial.println(F(" us"));
}

void onSpace() {
  switch (st) {
    case CONTRACTED:  st = EXPANDING;   tStart = millis(); drive(+1); Serial.println(F("▶ 확장 시작 (스페이스로 멈춤)")); break;
    case EXPANDING:   tExpand = millis() - tStart; stopServo(); st = EXPANDED;
                      Serial.print(F("■ 확장 멈춤 ")); Serial.print(tExpand); Serial.println(F(" ms (스페이스로 수축)")); break;
    case EXPANDED:    st = CONTRACTING; tStart = millis(); drive(-1);
                      Serial.print(F("◀ 수축 ")); Serial.print(tExpand); Serial.println(F(" ms 역회전")); break;
    case CONTRACTING: stopServo(); st = CONTRACTED; Serial.println(F("■ 수축 중단 → 대기 (위치는 어긋났을 수 있음, x 로 기준 재설정)")); break;
  }
}

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN, 1000, 2000);
  stopServo();
  Serial.println(F("가변 휠 캠 시험. 스페이스: 확장→멈춤→수축, x: 정지/초기화, +/-: 속도, [/]: 정지 보정, ?: 상태"));
  report();
}

void loop() {
  // 수축은 확장에 걸린 시간만큼만 역회전하고 자동 정지
  if (st == CONTRACTING && millis() - tStart >= tExpand) { stopServo(); st = CONTRACTED; Serial.println(F("■ 수축 완료 → 대기")); }
  // 안전: 확장이 너무 길면 정지
  if (st == EXPANDING && millis() - tStart >= MAX_RUN_MS) { tExpand = MAX_RUN_MS; stopServo(); st = EXPANDED; Serial.println(F("! 20 s 초과 자동 정지")); }

  // 시리얼 모니터가 공백만 있는 줄을 버리므로, 빈 줄(Enter만)도 스페이스로 취급
  static bool lineHadChar = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (!lineHadChar && c == '\n') onSpace(); if (c == '\n') lineHadChar = false; continue; }
    lineHadChar = true;
    switch (c) {
      case ' ': case 'e': case 'E': onSpace(); break;
      case 'x': case 'X': stopServo(); st = CONTRACTED; tExpand = 0; Serial.println(F("정지·초기화 (현재 위치를 수축 기준으로)")); break;
      case '+': case '=': speedUs = min(speedUs + 50, 500); if (st == EXPANDING) drive(+1); if (st == CONTRACTING) drive(-1); report(); break;
      case '-': case '_': speedUs = max(speedUs - 50, 50);  if (st == EXPANDING) drive(+1); if (st == CONTRACTING) drive(-1); report(); break;
      case '[': stopUs -= 10; if (st == CONTRACTED || st == EXPANDED) stopServo(); report(); break;
      case ']': stopUs += 10; if (st == CONTRACTED || st == EXPANDED) stopServo(); report(); break;
      case '?': report(); break;
      default: break;   // \r \n 등 무시
    }
  }
}
