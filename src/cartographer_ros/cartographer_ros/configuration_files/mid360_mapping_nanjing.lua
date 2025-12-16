include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  publish_tracked_pose=true,

}

MAP_BUILDER.num_background_threads = 8
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- 0.3
TRAJECTORY_BUILDER_2D.max_range = 25

TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 0.5



-- ┌─────────────────┬──────────┬──────────┬──────────┐
-- │   参数          │  含义     │  增加时  │  减少时  │
-- ├─────────────────┼──────────┼──────────┼──────────┤
-- │ occupied_space  │ 激光匹配 │ 更信任   │ 更信任   │
-- │ _weight         │ 重要性   │ 激光数据 │ IMU/Odom │
-- ├─────────────────┼──────────┼──────────┼──────────┤
-- │ translation     │ 位置偏离 │ 约束更强 │ 允许更大 │
-- │ _weight         │ 惩罚     │ 不易跳跃 │ 移动范围 │
-- ├─────────────────┼──────────┼──────────┼──────────┤
-- │ rotation        │ 角度偏离 │ 约束更强 │ 允许更大 │
-- │ _weight         │ 惩罚     │ 角度稳定 │ 角度调整 │
-- └─────────────────┴──────────┴──────────┴──────────┘


TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1--default 0.05

POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1-- GPS位置权重
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2


return options
