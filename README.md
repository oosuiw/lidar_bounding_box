# lidar_bounding_box

`lidar_bounding_box`는 ROS 2 패키지로, 2D YOLO 객체 탐지 결과, LiDAR 포인트 클라우드, 그리고 카메라 정보를 동기화하여 3D 바운딩 박스를 생성하고 시각화합니다. 이 패키지는 카메라와 LiDAR 센서 간의 공간적 관계(TF)를 활용하여 2D 탐지 결과를 3D 공간으로 확장합니다.

## 주요 기능

*   **센서 데이터 동기화:** YOLO 2D 탐지 결과 (`ultralytics_ros/YoloResult`), 카메라 정보 (`sensor_msgs/CameraInfo`), LiDAR 포인트 클라우드 (`sensor_msgs/PointCloud2`)를 `message_filters::ApproximateTimeSynchronizer`를 사용하여 시간적으로 동기화합니다.
*   **LiDAR-카메라 융합:** TF(Transform)를 사용하여 LiDAR 포인트들을 카메라 좌표계로 변환하고, 카메라의 내부 파라미터를 이용하여 2D 이미지 평면에 투영합니다.
*   **3D 바운딩 박스 생성:** 2D YOLO 바운딩 박스 내에 투영된 LiDAR 포인트들을 기반으로 3D 바운딩 박스(AABB: Axis-Aligned Bounding Box)를 계산합니다.
*   **클래스 이름 시각화:** 생성된 3D 바운딩 박스 위에 탐지된 객체의 클래스 이름을 텍스트로 함께 시각화합니다.
*   **RViz 시각화:** `visualization_msgs/MarkerArray` 메시지를 통해 RViz에서 3D 바운딩 박스와 클래스 텍스트를 시각화할 수 있습니다.

## 의존성

이 패키지는 다음 ROS 2 패키지 및 외부 라이브러리에 의존합니다:

*   `rclcpp`
*   `rclcpp_components`
*   `ultralytics_ros` (YOLO 탐지 결과를 위한 메시지 타입)
*   `visualization_msgs`
*   `sensor_msgs`
*   `tf2_ros`
*   `tf2_eigen`
*   `pcl_conversions`
*   `PCL` (Point Cloud Library)

## 설치

1.  **워크스페이스 생성 (선택 사항):**
    ```bash
    mkdir -p ~/your_ws/src
    cd ~/your_ws/src
    ```

2.  **패키지 클론:**
    ```bash
    git clone <이곳에 이 저장소의 URL을 입력하세요>
    ```

3.  **의존성 설치:**
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```
    (ROS 2 Humble을 사용한다고 가정합니다. 다른 ROS 2 버전을 사용한다면 `humble`을 해당 버전으로 변경하세요.)

4.  **패키지 빌드:**
    ```bash
    cd ~/your_ws
    colcon build --packages-select lidar_bounding_box
    ```

5.  **환경 설정:**
    ```bash
    source install/setup.bash # 또는 setup.zsh, setup.ps1 등 사용 중인 쉘에 맞게
    ```

## 사용법

`lidar_bounding_box` 노드를 실행하기 전에, LiDAR 프레임과 카메라 프레임 간의 TF(Transform) 정보가 반드시 ROS TF 트리에 제공되어야 합니다.

### TF 설정 예시

LiDAR 프레임(`lidar_link`)과 카메라 프레임(`camera_link`) 간의 정적 변환을 브로드캐스트하는 예시입니다. 실제 값은 센서 설정에 따라 달라집니다.

```bash
rclcpp run tf2_ros static_transform_publisher X Y Z R P Y lidar_link camera_link
```
*   `X, Y, Z`: LiDAR 프레임 원점에서 카메라 프레임 원점까지의 병진(translation) 값 (미터).
*   `R, P, Y`: LiDAR 프레임에 대한 카메라 프레임의 회전(roll, pitch, yaw) 값 (라디안).
*   `lidar_link`: 부모 프레임 ID (LiDAR).
*   `camera_link`: 자식 프레임 ID (카메라).

예시:
```bash
ros2 run tf2_ros static_transform_publisher 0.5 0.0 0.2 0.0 0.0 0.0 lidar_link camera_link
```
(LiDAR에서 카메라까지 x축으로 0.5m, z축으로 0.2m 떨어져 있고 회전은 없다는 가정)

### 노드 실행

TF가 설정된 후, 다음 명령으로 노드를 실행할 수 있습니다:

```bash
ros2 launch lidar_bounding_box lidar_bounding_box.launch.py
```

### RViz에서 시각화

RViz를 실행하고 다음 토픽을 추가하여 3D 바운딩 박스와 클래스 텍스트를 시각화할 수 있습니다:

*   `/lidar_bounding_box` (Type: `MarkerArray`)

## ROS 파라미터

`lidar_bounding_box` 노드는 다음 파라미터를 선언하고 사용합니다:

| 파라미터 이름             | 타입     | 기본값             | 설명                                     |
| :------------------------ | :------- | :----------------- | :--------------------------------------- |
| `input_yolo_topic`        | `string` | `/yolo_result`     | YOLO 2D 탐지 결과를 수신할 토픽 이름.    |
| `input_camera_info_topic` | `string` | `/camera_info`     | 카메라 정보를 수신할 토픽 이름.          |
| `input_point_cloud_topic` | `string` | `/point_cloud`     | LiDAR 포인트 클라우드를 수신할 토픽 이름. |
| `output_bounding_box_topic` | `string` | `/lidar_bounding_box` | 3D 바운딩 박스 마커를 발행할 토픽 이름. |
| `camera_frame_id`         | `string` | `camera_link`      | 카메라 센서의 프레임 ID.                 |
| `lidar_frame_id`          | `string` | `lidar_link`       | LiDAR 센서의 프레임 ID.                  |

이 파라미터들은 런치 파일(`launch` 디렉토리 참조)을 통해 설정하거나, 명령줄에서 직접 재정의할 수 있습니다.

## ROS 토픽

### 구독하는 토픽

*   `/yolo_result` (`ultralytics_ros/msg/YoloResult`)
*   `/camera_info` (`sensor_msgs/msg/CameraInfo`)
*   `/point_cloud` (`sensor_msgs/msg/PointCloud2`)

### 발행하는 토픽

*   `/lidar_bounding_box` (`visualization_msgs/msg/MarkerArray`)

## 기여

기여는 언제나 환영합니다! 버그 리포트, 기능 요청, 풀 리퀘스트 등 어떤 형태의 기여라도 좋습니다.