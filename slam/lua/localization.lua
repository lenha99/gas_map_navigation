include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",  -- 로봇의 중심 프레임
  published_frame = "base_footprint",  -- 위치 추정 결과를 게시할 프레임
  odom_frame = "odom",
  provide_odom_frame = true,  -- Cartographer가 odom 프레임을 생성하도록 설정
  publish_frame_projected_to_2d = true,
  use_odometry = false,  -- 오도메트리 사용 안 함
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Pure localization 모드를 활성화
TRAJECTORY_BUILDER.pure_localization = true

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1  -- LiDAR의 최소 감지 거리
TRAJECTORY_BUILDER_2D.max_range = 5  -- LiDAR의 최대 감지 거리
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMU 데이터 사용 안 함
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 실시간 스캔 매칭 사용
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 움직임 필터 설정
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 1번의 스캔마다 매칭

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 서브맵에 포함될 데이터 수
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 80  -- 스캔 매칭의 번역 가중치
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100  -- 스캔 매칭의 회전 가중치

POSE_GRAPH.constraint_builder.min_score = 0.65  -- 제약 조건 생성 시 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 글로벌 위치 추정 시 최소 점수


-- Latency 줄이기 위한 설정
POSE_GRAPH.optimize_every_n_nodes = 1  -- 결과를 자주 얻기 위해 최적화 빈도를 높임
POSE_GRAPH.global_sampling_ratio = 0.001  -- Global SLAM에서 샘플링 비율을 낮춤
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 제약 조건 생성에서 샘플링 비율을 낮춤

return options

