# turtlebot3_emergency

`turtlebot3_emergency`는 TurtleBot3의 모터 전원을 비상 정지/재개 토픽으로 제어하는 간단한 노드입니다.

## 동작 요약
- `/emergency_stop` (`std_msgs/Bool`)에 `true`가 들어오면 `/motor_power` 서비스에 `false`를 호출하여 모터 전원을 차단합니다.
- `/emergency_resume`에 `true`가 들어오면 `/motor_power` 서비스에 `true`를 호출하여 모터 전원을 다시 켭니다.
- 현재 비상 상태는 `/emergency/state` 토픽(`Bool`)으로 게시됩니다.

## 실행 예시
```bash
ros2 launch turtlebot3_emergency emergency_bridge.launch.py

# 비상 정지
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"

# 재개
ros2 topic pub --once /emergency_resume std_msgs/msg/Bool "{data: true}"

# 상태 모니터링
ros2 topic echo /emergency/state
```

> 주의: `/motor_power` 서비스는 TurtleBot3 기본 bringup(`ros2 launch turtlebot3_bringup robot.launch.py`)이 실행 중이어야 사용 가능합니다.
