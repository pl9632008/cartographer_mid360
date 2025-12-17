include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_link",             -- 将所有传感器数据转换到这个坐标系下 有imu_link就用imu_link
  published_frame = "base_link",            -- tf: map -> base_link
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = true,                -- 是否提供odom的tf, 如果为true, 则tf树为map->odom->base_link
                                            -- 如果为false tf树为map->base_link
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上
  use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿
  use_odometry = false,                     -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = true,                       -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 0,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 1,                     -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  
  publish_tracked_pose=true,

}

MAP_BUILDER.num_background_threads = 8
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1  
TRAJECTORY_BUILDER_2D.max_range = 25

-- 以tracking_frame为坐标系
TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 0.5


-- ┌──────────────────────┬────────────┬──────────────┬──────────────┐
-- │ 参数                  │ 含义       │ 增加时       │ 减少时       │
-- ├──────────────────────┼────────────┼──────────────┼──────────────┤
-- │ occupied_space_weight│ 激光匹配   │ 更信任       │ 更信任       │
-- │                      │ 重要性     │ 激光数据     │ IMU/Odom     │
-- ├──────────────────────┼────────────┼──────────────┼──────────────┤
-- │ translation_weight   │ 位置偏离   │ 约束更强     │ 允许更大     │
-- │                      │ 惩罚       │ 不易跳跃     │ 移动范围     │
-- ├──────────────────────┼────────────┼──────────────┼──────────────┤
-- │ rotation_weight      │ 角度偏离   │ 约束更强     │ 允许更大     │
-- │                      │ 惩罚       │ 角度稳定     │ 角度调整     │
-- └──────────────────────┴────────────┴──────────────┴──────────────┘

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1 --default 0.05

POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2


return options
