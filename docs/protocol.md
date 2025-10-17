# 통신 프로토콜 규약

[STM32](#stm32)<br>
[ROS](#ros토픽-형태)<br>
[Qt](#qt)<br>
[Jetson](#jetson)


```less
[SRC]|[WHAT]|[VAL]|[EXTRA]
```

- `SRC`: 발신자, 수신자 (STM32, QT, JETSON, ROS 등)
- `CMD`: 수행 명령(또는 이벤트명) — 기존 WHAT보다 명령/이벤트 의미가 명확
- `DATA`: 실제 명령에 필요한 주 데이터(숫자나 문자열 등)
- `META`: 선택적 부가 정보 (key=value, 여러 개는 ,로 구분)

---

## STM32

### → Jetson
택배 검수하는 곳에서 브레이크빔으로 물건이 올라온 것을 인식하였을 때
- jetson|capture|t

### → Qt
E-Stop(물리) 버튼을 눌렀을 경우
- qt|es|t

택배 무게측정한 값 전송
- qt|weight|[value]

### ← Qt
E-Stop(GUI) 버튼을 눌렀을 경우
- stm|es|t

E-Stop(GUI) 재개
- stm|es|f

발송 지역(0 : 미분류(반송), 1 : 서울, 2: 대전, 3: 부산)
- stm|zon|[0~3]

터틀봇이 집하존으로 출발
- stm|tur|t

터틀봇이 집하존에서 제자리로 돌아옴
- stm|tur|f

---

## ROS(토픽 형태)

### → Qt
집하존으로 터틀봇이 갔다가 왔을 경우
```bash
/unload/returned std_msgs/msg/Empty
```

### ← Qt
E-Stop 눌렀을 경우
```bash
/emergency_stop std_msgs/msg/Bool "{data: true}"
```

E-Stop 재개
```bash
/emergency_resume std_msgs/msg/Bool "{data: true}" 
```

지역 번호 전송
```bash
# 서울
/unload/destination std_msgs/msg/Int32 '{data: 0}'
# 대전
/unload/destination std_msgs/msg/Int32 '{data: 1}'
# 부산
/unload/destination std_msgs/msg/Int32 '{data: 2}'
```

---

## Qt

### → STM32
E-Stop(GUI) 버튼을 눌렀을 경우
- stm|es|t

E-Stop(GUI) 재개
- stm|es|f

발송 지역(0 : 미분류(반송), 1 : 서울, 2: 대전, 3: 부산)
- stm|zon|[0~3]

터틀봇이 집하존으로 출발
- stm|tur|t

터틀봇이 집하존에서 제자리로 돌아옴
- stm|tur|f

### ← STM32
E-Stop(물리) 버튼을 눌렀을 경우
- qt|es|t

### → ROS
E-Stop 눌렀을 경우
```bash
/emergency_stop std_msgs/msg/Bool "{data: true}"
```

E-Stop 재개
```bash
/emergency_resume std_msgs/msg/Bool "{data: true}" 
```

지역 번호 전송
```bash
# 서울
/unload/destination std_msgs/msg/Int32 '{data: 0}'
# 대전
/unload/destination std_msgs/msg/Int32 '{data: 1}'
# 부산
/unload/destination std_msgs/msg/Int32 '{data: 2}'
```

### ← ROS
집하존으로 터틀봇이 갔다가 왔을 경우
```bash
/unload/returned std_msgs/msg/Empty
```

### ← Jetson
카메라로 택배의 수령지 정보와 주의 라벨 정보를 전송
- jetson|[ocr value]|[detection value]
    - ocr value : 0, 1, 2, 3(0 : 미분류, 1 : 서울, 2: 대전, 3: 부산)
    - detection value : a, b, c, d

---

## Jetson

### ← STM32
택배 검수하는 곳에서 브레이크빔으로 물건이 올라온 것을 인식하였을 때
- jetson|capture|t

### → Qt
카메라로 택배의 수령지 정보와 주의 라벨 정보를 전송
- jetson|[ocr value]|[detection value]
    - ocr value : 0, 1, 2(0 : 서울, 1 : 대전, 2: 부산)
    - detection value : a, b, c, d