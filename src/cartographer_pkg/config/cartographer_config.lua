-- cartographer_go2_2d.lua
-- 仅用于二维建图；建议在 launch 里把 imu remap 到 /imu_cart，scan 到 /scan，odom 到 /odom
-- 重要帧关系：
--   tracking_frame    = "base_link"   -- IMU/机体刚体坐标
--   published_frame   = "base_link"   -- 发布给 TF 的机器人姿态所在帧
--   odom_frame        = "odom"        -- 局部里程计系，Cartographer 提供 map->odom
--
-- 请确保 TF 链： map -> odom (Cartographer) -> base_link (里程计/融合) -> livox_frame (静态外参)
-- 激光 /scan 的 frame_id 通常是 livox_frame，有静态变换 base_link->livox_frame 即可

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = true,                -- 由 Cartographer 发布 map->odom
  publish_frame_projected_to_2d = true,     -- 仅发布 2D 位姿

  use_odometry = true,                      -- 使用 /odom（x,y,yaw 即可）
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,                      -- 使用 1 路 LaserScan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,      -- 不分割
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.5,       -- 放宽 TF 查找时间
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 启用 2D
MAP_BUILDER.use_trajectory_builder_2d = true

-- 激光参数（与你原配置一致）
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- 使用 IMU（要求 /imu 或 remap 后的 /imu_cart 提供三轴 gyro/accel，含重力）
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 机器狗抖动较大，增大重力时间常数更稳（可在 15~25 之间微调）
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 20.0

-- 前端在线相关性匹配（退化环境更稳）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 运动滤波：放宽角度阈值，减少过度滤波导致的丢匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- 约束构建阈值（稍微放松，利于回环/约束生成）
POSE_GRAPH.constraint_builder.min_score = 0.58
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

-- 如果你的 /odom 很稳，可逐步上调里程计权重（先小步调，观察效果）
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight    = 1e3

-- 如需关闭后端优化（调试用），可设置为 0（生产不建议）
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options