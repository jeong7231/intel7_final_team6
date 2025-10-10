#include <TurtleBot3_ROS2.h>
#include <Servo.h>

// Dumpbox 서보는 D9 핀에 연결되어 있으며, TurtleBot3 제어 테이블 주소 360을 통해 명령을 받습니다.
// 이 스케치는 공식 TurtleBot3 펌웨어에 이미 통합된 기능을 간단하게 분리한 버전으로,
// Nav2/ROS2 노드와의 통신은 Dynamixel 프로토콜(제어 테이블)을 통해 이루어집니다.
// ASCII 명령을 직접 파싱하던 기존 코드는 TurtleBot3Core::run()과 충돌하므로 제거했습니다.

namespace dumpbox
{
constexpr uint8_t kServoPin = 9;
constexpr uint16_t kAddrServoCmd = 360;
constexpr uint16_t kPulseOpen = 1000;   // OPEN
constexpr uint16_t kPulseClose = 1500;  // CLOSE
constexpr uint32_t kTimeoutMs = 3000;   // 명령 타임아웃

static Servo servo;
static uint32_t last_cmd_ms = 0;
static uint8_t last_cmd = 0;  // 0: close, 1: open

inline void apply(uint8_t cmd)
{
  last_cmd = cmd ? 1 : 0;
  servo.writeMicroseconds(last_cmd ? kPulseOpen : kPulseClose);
  last_cmd_ms = millis();
}

inline void ensure_timeout()
{
  if ((millis() - last_cmd_ms) > kTimeoutMs) {
    apply(0);
  }
}
}

void setup()
{
  TurtleBot3Core::begin("Burger");
  dumpbox::servo.attach(dumpbox::kServoPin);
  dumpbox::apply(0);
}

void loop()
{
  TurtleBot3Core::run();
  dumpbox::ensure_timeout();
}
