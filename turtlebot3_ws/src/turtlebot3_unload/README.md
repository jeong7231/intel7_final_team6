# turtlebot3_unload

`turtlebot3_unload` 패키지는 목적지 ID를 입력받아 **중간 지점 이동 → ArUco 마커 탐색 → 비전 기반 자동 주차 → 덤프박스 하차 → 복귀** 순으로 임무를 수행하는 상위 미션 노드를 제공합니다.

## 동작 개요
1. `/unload/destination` 토픽에서 `0~2` 사이의 목적지 ID를 수신합니다.
2. YAML로 정의된 `staging_pose`(맵 좌표)로 NAV2 `NavigateToPose` 액션을 요청해 이동합니다.
3. 카메라로 아루코 마커를 찾을 때까지 회전하며 `/target_marker_id`로 마커 ID를 통지합니다.
4. `turtlebot3_automatic_parking_vision` 노드가 주차를 완료하면 덤프박스 서보 PWM 토픽을 발행해 적재물을 하차합니다.
5. `return_pose` 또는 기본 홈 위치로 복귀한 뒤 상태를 초기화합니다.

## 패키지 구성
```
turtlebot3_unload/
├── launch/
│   ├── unload_mission.launch.py      # 미션 노드 단독 실행
│   └── unload_system.launch.py       # 아루코/주차/덤프박스 통합 실행
├── config/
│   └── destinations.yaml             # 목적지 ID별 좌표 및 마커 설정
└── turtlebot3_unload/
    ├── __init__.py
    └── mission_manager.py            # 상태 머신 구현
```

## 주요 의존 노드
- `turtlebot3_navigation2`: NAV2 `navigate_to_pose` 액션 서버
- `turtlebot3_aruco`: ArUco 마커 인식 및 TF 브로드캐스트
- `turtlebot3_automatic_parking_vision`: 마커 기반 자동 주차
- `turtlebot3_dumpbox`: 덤프박스 서보 제어

## 실행 예시
```bash
# 전체 파이프라인 실행 (NAV2는 별도 실행 가정)
ros2 launch turtlebot3_unload unload_system.launch.py \
  config_file:=config/destinations.yaml \
  parking_status_topic:=/automatic_parking/status \
  parking_reset_service:=/automatic_parking/reset

# 목적지 ID 전송 (예: 1번 목적지)
ros2 topic pub /unload/destination std_msgs/msg/Int32 "{data: 1}"
```

## 좌표 설정
- `config/destinations.yaml`의 `staging_pose`, `return_pose`는 **맵 프레임 기준 절대 좌표**입니다.
- 현재 예시는 측정값을 반영해 `staging_pose ≈ (1.678, 0.106, -0.014rad)`, `return_pose ≈ (-0.052, 0.032, 0.053rad)`로 설정되어 있습니다.
- 환경이 변하면 RViz/`/amcl_pose` 등을 활용해 좌표를 다시 측정한 뒤 갱신하세요.
- `marker_id`는 `turtlebot3_automatic_parking_vision`과 아루코 TF(`ar_marker_<id>`)가 일치해야 합니다.

## 개발 메모
- `/unload/status`와 `/automatic_parking/status` 토픽으로 상태를 모니터링하십시오.
- `marker_search` 항목(rotation_speed, max_search_time)을 YAML에서 조정해 탐색 동작을 튜닝할 수 있습니다.
- 덤프박스 하차는 PWM 값(예: 1000 → Open, 2000 → Close)을 주는 방식과 Trigger 서비스를 병행할 수 있습니다.

## 향후 확장 아이디어
- 실패/타임아웃 상태를 분리해 경고 메시지를 명확히 제공
- launch_testing 기반 자동화 테스트 추가
- RViz 패널 및 위젯을 활용한 임무 모니터링 UI 제공
