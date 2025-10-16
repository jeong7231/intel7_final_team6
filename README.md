# turtlebot3_emergency

터틀봇3의 이동 관련 `cmd_vel` 토픽을 가로채어 비상 정지(Stop)와 재개(Resume)를 처리하는 간단한 브릿지 노드입니다.

## 기능
- `/cmd_vel_nav` (또는 원하는 입력 토픽)을 구독하여 현재 속도 명령을 감시합니다.
- `/emergency_stop` 토픽에서 `std_msgs/msg/Bool` 값이 `true`로 들어오면 비상 정지 상태로 전환하고 `/cmd_vel`에 0 속도를 주기적으로 발행합니다.
- `/emergency_resume` 토픽에서 `true`가 들어오면 정상 상태로 복귀하여 입력된 속도 명령을 다시 통과시킵니다.
- 현재 비상 상태는 `/emergency/state` 토픽(`Bool`)으로 브로드캐스트됩니다.

## 실행 예
```bash
ros2 launch turtlebot3_emergency emergency_bridge.launch.py   input_cmd_vel:=/cmd_vel_nav   output_cmd_vel:=/cmd_vel
```

### 비상 정지 / 해제 토픽 발행
```bash
# 비상 정지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"

# 비상 정지 해제
ros2 topic pub --once /emergency_resume std_msgs/msg/Bool "{data: true}"
```

비상 정지 노드를 사용할 때에는 NAV2, 미션 노드 등 이동 명령을 내리는 쪽에서 `/cmd_vel` 대신 `/cmd_vel_nav`(또는 지정한 입력 토픽 이름)으로 발행하고, 실제 `/cmd_vel`은 비상 정지 노드가 담당하도록 Launch 파일에서 remap해 주세요.
