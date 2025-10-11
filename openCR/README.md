# OpenCR 제어

```bash
/home/ubuntu123/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.3/libraries/turtlebot3_ros2/src/turtlebot3
```

여기있는 turtlebot3.cpp 수정 필요

- [turtlebot3.cpp](turtlebot3.cpp)

수정 후 turtlebot3_burger.ino 업로드

추가·변경 사항 요약본

## 헤더

```cpp
#include <Servo.h>
```

## 컨트롤 테이블 주소

```cpp
enum ControlTableItemAddr {
  /* ... */
  ADDR_DUMPBOX_SERVO_CMD = 360,
};
```

## ControlItemVariables 변경

```cpp
typedef struct ControlItemVariables{
  /* ... 기존 필드 ... */
  float   sonar;                // sornar → sonar
  bool    joint_goal_current_rd;// 오타 수정
  uint8_t dumpbox_servo_cmd;    // 서보 명령
} ControlItemVariables;
```

## 서보 전역과 상수

```cpp
static Servo dumpbox_servo;
static uint32_t dumpbox_servo_last_cmd_ms = 0;
static const uint16_t DUMPBOX_SERVO_OPEN_US  = 1000;
static const uint16_t DUMPBOX_SERVO_CLOSE_US = 1500;
static const uint32_t DUMPBOX_SERVO_TIMEOUT_MS = 3000;
```

## 서보 헬퍼 함수

```cpp
static inline void dumpbox_servo_set(bool open){
  dumpbox_servo.writeMicroseconds(open ? DUMPBOX_SERVO_OPEN_US : DUMPBOX_SERVO_CLOSE_US);
  dumpbox_servo_last_cmd_ms = millis();
  control_items.dumpbox_servo_cmd = open ? 1 : 0;
}

static inline void dumpbox_servo_apply_command(){
  dumpbox_servo_set(control_items.dumpbox_servo_cmd != 0);
}
```

## begin() 추가 코드

```cpp
control_items.dumpbox_servo_cmd = 0;

dumpbox_servo.attach(9);
dumpbox_servo_set(false);
dxl_slave.addControlItem(ADDR_DUMPBOX_SERVO_CMD, control_items.dumpbox_servo_cmd);

dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);
```

## run() 추가 호출

```cpp
update_dumpbox_servo(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
```

## 주기적 타임아웃 처리

```cpp
static void update_dumpbox_servo(uint32_t interval_ms){
  static uint32_t pre_time = 0;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  if ((millis() - dumpbox_servo_last_cmd_ms) > DUMPBOX_SERVO_TIMEOUT_MS){
    dumpbox_servo_set(false);
  }
}
```

## DXL 슬레이브 쓰기 콜백 처리

```cpp
static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg){
  /* ... 기존 switch ... */
  case ADDR_DUMPBOX_SERVO_CMD:
    dumpbox_servo_apply_command();
    break;
}
```

## 센서 업데이트 키 정리

```cpp
control_items.sonar = (float)sensors.getSonarData();
```

## 누락 함수 정의(링크 오류 방지)

```cpp
void TurtleBot3Core::run(){
  /* 기존 로직 + update_dumpbox_servo 호출 포함 */
}
```
