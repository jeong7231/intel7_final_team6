#include <TurtleBot3_ROS2.h>
#include <Servo.h>

Servo s;

// 두 상태만 사용
static const uint16_t POS_CLOSE_US = 1500;   // 닫힘
static const uint16_t POS_OPEN_US  = 1000;   // 열림
static const uint32_t CMD_TIMEOUT_MS = 3000; // 신호 없으면 자동 닫힘
static uint32_t last_cmd_ms = 0;

inline void set_open(bool open){
  s.writeMicroseconds(open ? POS_OPEN_US : POS_CLOSE_US);
  last_cmd_ms = millis();
}

void setup() {
  TurtleBot3Core::begin("Burger");

  s.attach(9);          // OpenCR D9
  set_open(false);      // 부팅 시 닫힘
  // Serial은 TB3 코어가 이미 초기화. 별도 baud 설정 불필요.
}

void loop() {
  TurtleBot3Core::run();

  // ASCII 명령 수신: "BOX OPEN\n" 또는 "BOX CLOSE\n"
  static char buf[16]; 
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[idx] = 0; 
      idx = 0;
      if (strcmp(buf, "BOX OPEN")  == 0) set_open(true);
      if (strcmp(buf, "BOX CLOSE") == 0) set_open(false);
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  // 타임아웃 안전동작
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    set_open(false);
  }
}
