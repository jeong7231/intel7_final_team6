# turtlebot3_unload

`turtlebot3_unload` 패키지는 목적지 ID를 입력받아 **좌표 기반 이동 → 덤프박스 하차 → 복귀** 순으로 임무를 수행하는 상위 미션 노드를 제공합니다.  
카메라나 ArUco 마커 없이, NAV2에 미리 정의된 포즈를 전달해 주차 위치까지 이동합니다.

## 동작 개요
1. `/unload/destination` 토픽으로 `0~2` 사이의 목적지 ID를 수신합니다.
2. `config/destinations.yaml`에 정의된 `drop_pose`로 NAV2 `NavigateToPose` 액션을 전송합니다. (선택적으로 `staging_pose`를 선행 사용할 수 있습니다.)
3. 주차 위치에 도착하면 덤프박스 서보 PWM을 발행해 화물을 하차합니다.
4. `return_pose`(또는 `default_home`)로 복귀한 뒤 상태를 초기화합니다.

## 패키지 구성
```
turtlebot3_unload/
├── launch/
│   ├── unload_mission.launch.py      # 미션 노드 단독 실행
│   └── unload_system.launch.py       # 덤프박스 + 미션 통합 실행
├── config/
│   └── destinations.yaml             # 목적지 ID별 좌표 및 마커 설정
└── turtlebot3_unload/
    ├── __init__.py
    └── mission_manager.py            # 상태 머신 구현
```

## 주요 의존 노드
- `turtlebot3_navigation2`: NAV2 `navigate_to_pose` 액션 서버
- `turtlebot3_dumpbox`: 덤프박스 서보 제어

## 실행 예시
```bash
# 전체 파이프라인 실행 (NAV2는 별도 실행 가정)
ros2 launch turtlebot3_unload unload_system.launch.py \
  config_file:=config/destinations.yaml

# 목적지 ID 전송 (예: 1번 목적지)
ros2 topic pub /unload/destination std_msgs/msg/Int32 "{data: 1}"
```

## 좌표 설정
- `config/destinations.yaml`의 `drop_pose`, `return_pose`는 **맵 프레임 기준 절대 좌표**입니다.
- 현재 예시는 `drop_pose ≈ (2.163, 0.074, -0.343rad)`, `return_pose ≈ (-0.027, 0.006, 0.011rad)`로 설정했습니다.
- 환경이 변하면 RViz/`/amcl_pose` 등을 활용해 좌표를 재측정한 뒤 갱신하세요.
- 필요하면 `staging_pose`를 추가해 중간 지점을 거친 뒤 drop 포즈로 이동하도록 설정할 수 있습니다.

## 개발 메모
- `/unload/status` 토픽으로 미션 상태를 모니터링할 수 있습니다.
- 덤프박스 하차는 PWM 값(기본 1000 → Open, 1500 → Close)을 이용합니다. `unload` 섹션의 파라미터로 값을 조정할 수 있습니다.

## 향후 확장 아이디어
- 실패/타임아웃 상태를 분리해 경고 메시지를 명확히 제공
- launch_testing 기반 자동화 테스트 추가
- RViz 패널 및 위젯을 활용한 임무 모니터링 UI 제공
