# gas_map_navigation

**실내를 자율주행하면서 가스 농도를 위치와 함께 기록·시각화하는 ROS 모바일 로봇.**

사람이 돌면서 하는 가스 점검은 "어디서 몇 ppm이었는지"가 기록으로 남지 않고 재현도 어렵습니다.
이 프로젝트는 로봇이 스스로 실내를 돌면서 **측정값과 좌표를 함께 남기도록** 만든 것입니다.

---

## 무엇을 하는가

1. 라이다 기반 SLAM으로 실내 지도를 작성한다
2. 작성된 지도 위에서 위치를 추정하며 지정 지점까지 자율주행한다
3. 주행하는 내내 가스 센서 값을 읽고, **좌표 변환(TF)으로 얻은 로봇의 현재 위치와 결합**한다
4. 결합된 값을 RViz에 3D 마커로 실시간 표시하고, 동시에 파일로 기록한다

농도 구간에 따라 마커의 **색(초록→빨강)과 형태(화살표 / 큐브 / 구)** 가 함께 바뀝니다.
색만으로는 구간 경계가 눈에 안 들어와서, 형태를 두 번째 채널로 썼습니다.

---

## 패키지 구성

### `cartographer_position` — 이 프로젝트의 핵심

| 노드 | 역할 |
|---|---|
| `color_position.cpp` | 가스 센서 값 구독 → TF로 현재 위치 조회 → 농도별 색/형태를 정한 `MarkerArray` 발행 + 파일 기록 |
| `get_position.cpp` | 주행 중 로봇 위치를 추적해 측정 지점 좌표를 확보 |

측정값과 위치는 서로 다른 주기로 들어옵니다. 그래서 마커를 발행하는 시점에 **그때의 TF를 조회해
두 스트림을 맞춥니다.** 센서 콜백에서 위치를 함께 저장하는 방식은, 위치가 갱신되기 전 값에
측정치가 붙어 실제보다 뒤처진 좌표에 기록되는 문제가 있었습니다.

### `waypoint` — 경로 기록 후 재현 (teach & repeat)

수동으로 로봇을 몰면서 2초 간격으로 지도 좌표계 기준 위치와 방향을 기록해 저장하고,
이후 그 경로를 그대로 따라가며 자율주행합니다.
주행 중 라이다로 전방 장애물을 감지하면 정지하고, 키보드로 언제든 개입할 수 있습니다.
기록된 경로는 RViz에 마커로 표시됩니다.

점검 경로가 매번 같은 현장에서는, 목표 지점을 매번 찍는 것보다 이 방식이 실용적이었습니다.

### `slam` — 지도 작성

Cartographer와 Hector SLAM 런치 및 튜닝 설정(`lua`)을 함께 두고 비교했습니다.
오도메트리를 쓰는 구성과 라이다만 쓰는 구성을 나눠 두었습니다.

### `hongik_*` — 로봇 플랫폼

| 패키지 | 내용 |
|---|---|
| `hongik_localization` | 엔코더 오도메트리 발행(`odom_pub.cpp`), EKF 위치 추정, RViz 클릭 → 2D 목표 변환, 목표 이동·회전 스크립트 |
| `hongik_navigation` | AMCL · `move_base` 설정, 로컬 플래너 파라미터(DWA / TEB / trajectory), 작성된 맵 |
| `hongik_description` | URDF/xacro 로봇 모델링 |
| `hongik_imuconverter` | MPU6050 원시값 → ROS `sensor_msgs/Imu` 변환 |
| `hongik_bringup` | 하드웨어 기동 런치와 모터 파라미터 |
| `hongik_teleop` | 키보드 수동 조종 |
| `arduino/` | 모터 드라이버 · 엔코더 · LED/부저 펌웨어 (Arduino Nano / ESP32), udev 규칙 |

엔코더만으로는 회전 중 오차가 빠르게 누적돼서, **IMU를 EKF로 융합**해 위치 추정을 보강했습니다.

### 외부 패키지

`rplidar_ros`(라이다 드라이버), `rf2o_laser_odometry`(라이다 기반 오도메트리)는 외부 공개 패키지입니다.

---

## 구성

```
센서        RPLidar · MPU6050 (IMU) · 휠 엔코더 · 가스 센서
제어        Arduino Nano / ESP32 (모터·엔코더 펌웨어)
SLAM        Cartographer · Hector (비교 후 맵 확보)
내비게이션  AMCL + move_base (DWA / TEB 로컬 플래너)
위치추정    EKF (엔코더 오도메트리 + IMU)
언어        C++ · Python
환경        ROS Melodic / Ubuntu 18.04 · catkin
```

---

## 실행

```bash
catkin_make && source devel/setup.bash

# 1. 하드웨어 기동 (모터 · 엔코더 · IMU · 라이다)
roslaunch hongik_bringup esp32.launch

# 2. 오도메트리 + EKF 위치추정
roslaunch hongik_localization hongik_ekfPose.launch

# 3. 저장된 맵 위에서 자율주행
roslaunch hongik_navigation hongik_Navi.launch

# 4. 가스 농도 매핑 노드
rosrun cartographer_position color_position
```

RViz에서 `2D Nav Goal`로 목표 지점을 찍으면 주행이 시작되고, 경로를 따라 농도 마커가 쌓입니다.

지도를 새로 만들려면 `roslaunch slam hongik_cartographer.launch`,
경로를 기록해 재현하려면 `roslaunch waypoint waypoint_recording.launch` →
`roslaunch waypoint waypoint_following.launch` 순으로 실행합니다.

---

> `build/`, `devel/` 은 `catkin_make` 로 생성되므로 저장소에 포함하지 않습니다.
