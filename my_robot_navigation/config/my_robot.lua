-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- === ФРЕЙМЫ ===
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",           -- КЛЮЧЕВОЙ ПАРАМЕТР
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,

  -- === ДАННЫЕ ===
  use_pose_extrapolator = true,
  use_odometry = true,                -- ВКЛЮЧЕНО
  use_nav_sat = false,
  use_landmarks = false,

  -- === СЕНСОРЫ ===
  num_laser_scans = 1,                -- 1 LaserScan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- === ОТКЛЮЧАЕМ IMU ===
  imu_sampling_ratio = 0,             -- ОТКЛЮЧАЕМ ОЖИДАНИЕ IMU

  -- === ТАЙМИНГИ ===
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- === СЭМПЛИНГ ===
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- === 2D SLAM ===
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

return options
