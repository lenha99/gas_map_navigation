include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  num_accumulated_range_data = 1,
  min_range = 0.3,
  missing_data_ray_length = 2.,
  use_imu_data = false,
  ceres_scan_matcher = {
    translation_weight = 10,
    rotation_weight = 15,
  },
  submaps = {
    num_range_data = 100,
  },
  max_range = 10.,
}

POSE_GRAPH = {
  constraint_builder = {
    fast_correlative_scan_matcher = {
      angular_search_window = math.rad(15.),
      linear_search_window = 3.,
    },
    sampling_ratio = 0.4,
    min_score = 0.85,
  },
  optimization_problem = {
    huber_scale = 1e2,
  },
  optimize_every_n_nodes = 100,
  global_sampling_ratio = 0.003,
  global_constraint_search_after_n_seconds = 30,
}

return options

