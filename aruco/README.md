# aruco_pose_estimation

> 캘리브레이션 정확하게 안된거같음

## 의존성
```bash
sudo apt update
sudo apt install -y \
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-vision-msgs \
    ros-humble-v4l2-camera

pip install "numpy<2" opencv-contrib-python pyyaml scipy
```
>

## 1. ROS2 워크스페이스 준비

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```


## 2. 카메라 노드 설치 및 실행

- 패키지: `usb_cam` 사용
- 실행 예시:

```bash
ros2 run usb_cam usb_cam_node_exe \
  --ros-args \
  -p video_device:=/dev/video0 \
  -p framerate:=30.0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:=yuyv \
  -p camera_frame_id:=default_cam \
  -r /image_raw:=/camera/image_raw \
  -r /camera_info:=/camera/camera_info
```

- 확인:

```bash
ros2 topic echo /camera/image_raw --once
```


## 3. ArUco Pose Estimation 패키지 준비

1. `aruco_pose_estimation` (직접 작성/포팅) 패키지를 `~/ros2_ws/src/` 안에 배치
2. 주요 노드:
    - `aruco_detector` (마커 검출)
    - `aruco_pose_tf` (마커 Pose → TF 브로드캐스트)

## 4. OpenCV 버전 문제 해결

- OpenCV 최신 버전(4.12)에서는 API가 바뀌어 `DetectorParameters_create` / `estimatePoseSingleMarkers` 등이 제거됨.
- 해결: `cv2.aruco.ArucoDetector` 와 `cv2.solvePnP` 조합으로 코드 수정.


## 5. 카메라 캘리브레이션 파일 준비

1. `ros-humble-camera-calibration` 설치

    ```bash
    sudo apt install ros-humble-camera-calibration
    ```

2. 실행

    ```bash
    ros2 run camera_calibration cameracalibrator \
      --size 8x6 \
      --square 0.03 \
      image:=/camera/image_raw camera:=/camera
    ```

    (체커보드 코너 수/칸 크기 맞춰 실행)

3. `/tmp/calibrationdata.tar.gz` 안의 `calibration.yaml` 확보

```yaml
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [985.01101,   0.     , 355.46371,
           0.     , 965.79071, 350.22292,
           0.     ,   0.     ,   1.     ]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.068987, -0.272002, 0.006081, 0.015717, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [986.83321,   0.     , 360.50356,   0.     ,
           0.     , 973.11465, 351.67669,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]

```

## 7. 실행

```bash
ros2 run aruco_pose_estimation aruco_pose_tf \
  --ros-args \
  -p aruco_dictionary_name:=DICT_4X4_100 \
  -p aruco_marker_side_length:=0.10 \
  -p image_topic:=/camera/image_raw \
  -p camera_calibration_parameters_filename:=/home/ubuntu/ros2_ws/src/aruco_pose_estimation/calibration.yaml
```

출력 예시:
```bash
ros2 topic echo /tf
```

```yaml
---
transforms:
- header:
    stamp:
      sec: 1757141701
      nanosec: 885853798
    frame_id: default_cam
  child_frame_id: aruco_marker_0
  transform:
    translation:
      x: 0.028894963771946655
      y: -0.14430710201924984
      z: 0.6577681214660308
    rotation:
      x: 0.9852083151185003
      y: 0.08680064931340861
      z: -0.13662938946224673
      w: -0.05623729221175397
---
transforms:
- header:
    stamp:
      sec: 1757141701
      nanosec: 953378413
    frame_id: default_cam
  child_frame_id: aruco_marker_1
  transform:
    translation:
      x: 0.031426556301215734
      y: -0.1294346931298688
      z: 0.6354449059916838
    rotation:
      x: 0.9875916038707079
      y: 0.09084085460680849
      z: -0.1111433547720262
      w: -0.06370178795298495
---
```


## 8. RViz2에서 시각화

1. 실행:

    ```bash
    rviz2
    ```

2. **Global Options → Fixed Frame** → `default_cam` (또는 `aruco_marker_<id>`).
3. Displays → Add → `TF` 추가.
4. 원한다면 Displays → Add → `Image`, Topic → `/camera/image_raw` 설정.
5. 결과: 카메라 원점과 ArUco 마커 좌표축이 3D 뷰에 표시됨.

## 9. 결과

- 카메라 영상에서 마커 인식 → TF 트리(`/tf`)에 `aruco_marker_<id>` 좌표계가 생성됨.
- RViz2에서 `default_cam` 기준으로 마커 좌표축 확인 가능.
- 실제 거리와 오차는 캘리브레이션 품질에 따라 달라짐.


![aruco](image/Screenshot%20from%202025-09-06%2016-27-47.png)
![aruco2](image/Screenshot%20from%202025-09-06%2016-36-17.png)