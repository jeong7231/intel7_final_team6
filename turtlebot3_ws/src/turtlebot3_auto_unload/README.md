# turtlebot3_auto_unload

이 패키지는 목적지 ID(`std_msgs/msg/Int32`)를 입력받아 다음 순서의 임무를 자동으로 수행하는 ROS 2 Python 노드를 제공합니다.

1. NAV2 `NavigateToPose` 액션으로 중간 집결지(스테이징 포인트)까지 이동
2. ArUco 마커가 감지될 때까지 회전 탐색 후 `turtlebot3_automatic_parking_vision` 노드를 트리거
3. 자동 주차 비전 노드가 `state` 업데이트를 통해 완주를 알리면 덤프박스 서보를 호출하여 하차
4. 지정된 복귀 지점 또는 기본 홈 위치로 다시 이동해 임무 종료

## 주요 구성 요소
- `mission_manager`: 상태 머신 기반 임무 조정 노드. NAV2, TF2, 덤프박스 서비스, 자동 주차 상태 토픽과 연동합니다.
- `auto_unload.launch.py`: 미션 노드 단독 실행용 런치 파일.
- `auto_unload_system.launch.py`: ArUco 추적, 자동 주차 비전, 덤프박스 로봇 브링업과 미션 노드를 함께 기동하는 통합 런치 파일.

## 설정 파일
- `config/destinations.yaml`: 기본 좌표 세트(예시 값). 실제 맵에 맞춰 각 목적지의 `marker_id`, `staging_pose`, `drop_pose`, `return_pose`를 수정하세요.
- `config/destinations.example.yaml`: 포맷 참고용 예제.

## 런치 예시
```bash
# 미션 노드만 실행 (기존 인프라가 별도로 올라와 있는 경우)
ros2 launch turtlebot3_auto_unload auto_unload.launch.py config_file:=config/destinations.yaml

# ArUco 추적 + 자동 주차 + 덤프박스 + 미션을 한 번에 실행
ros2 launch turtlebot3_auto_unload auto_unload_system.launch.py \
  config_file:=config/destinations.yaml \
  parking_status_topic:=/automatic_parking/status \
  parking_reset_service:=/automatic_parking/reset
```

> **주의**: 통합 런치 파일은 NAV2 스택이 별도로 가동 중이라고 가정합니다. 필요 시 `ros2 launch turtlebot3_navigation2 navigation2.launch.py` 등과 조합해 사용하세요.

## 자동 주차 상태 연동
`turtlebot3_automatic_parking_vision` 노드는 `status_topic` 파라미터(기본 `/automatic_parking/status`)를 통해 `state=<STATE>, marker=<ID>, detail=...` 형식의 문자열을 발행하고, `reset_service` 파라미터(기본 `/automatic_parking/reset`)로 상태를 초기화할 수 있습니다. 미션 노드는 해당 토픽을 모니터링하여 주차 완료(`detail`에 `complete` 포함)를 감지한 뒤 덤프박스를 작동합니다.
