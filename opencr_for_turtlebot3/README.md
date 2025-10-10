# TurtleBot3 OpenCR 펌웨어 패치 묶음

이 디렉터리는 Dumpbox 서보 제어 기능이 반영된 `turtlebot3_ros2` 라이브러리의 핵심 파일을 담고 있습니다. 다른 Ubuntu PC에서 Arduino IDE를 사용해 OpenCR 보드에 업로드할 때, 아래 순서대로 적용하면 됩니다.

## 구성 파일
```
opencr_for_turtlebot3/
└── arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp
```

## 적용 방법
1. **대상 PC에 OpenCR 소스를 준비**
   - GitHub에서 ROBOTIS `OpenCR` 저장소를 내려받거나, 기존에 설치된 OpenCR Arduino 패키지를 사용합니다.
   - 기본 경로 예시: `~/OpenCR/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/...`

2. **파일 덮어쓰기**
   - 이 디렉터리를 대상 PC로 복사한 뒤, 기존 `turtlebot3.cpp`를 백업하고 아래처럼 덮어씁니다.
     ```bash
     # 대상 PC에서 실행
     cp -a opencr_for_turtlebot3/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp \
       ~/OpenCR/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/
     ```

3. **Arduino IDE(또는 arduino-cli)로 업로드**
   - Arduino IDE를 실행하고 `File → Examples → turtlebot3_ros2 → turtlebot3_burger` (사용 모델에 맞는 스케치) 를 엽니다.
   - 보드를 **OpenCR**로 선택하고, 포트(예: `/dev/ttyACM0`)를 지정한 뒤 업로드합니다.
   - 업로드가 완료되면 보드를 한 번 재부팅합니다(USB 케이블 재연결).

4. **동작 확인**
   - SBC에서 `ros2 launch turtlebot3_dumpbox robot_bringup.launch.py` 실행 후, 다음과 같이 명령을 내려 서보가 움직이는지 확인합니다.
     ```bash
     ros2 topic pub --once /turtlebot3_dumpbox/servo/pwm std_msgs/msg/Float32 '{data: 1000.0}'  # OPEN
     ros2 topic pub --once /turtlebot3_dumpbox/servo/pwm std_msgs/msg/Float32 '{data: 1600.0}'  # CLOSE
     ```
   - 3초 동안 명령이 없으면 자동으로 닫히도록 구성되어 있습니다.

## 참고
- ROS 2 측에서는 `turtlebot3_dumpbox` 패키지의 `servo_controller_node.cpp`가 제어 테이블 주소 `360`에 0/1 값을 쓰도록 변경되어 있습니다. OpenCR 펌웨어와 호스트 노드를 모두 업데이트해야 정상 동작합니다.
- 추가로 수정한 파일이 있다면 동일한 방식으로 이 디렉터리에 복사해 함께 배포하세요.
