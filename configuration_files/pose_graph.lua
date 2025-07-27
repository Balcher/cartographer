-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- --  全局优化器
-- 主要负责：
-- 1. 后端约束构建（如回环检测）
-- 2. 全局图优化（使用ceres求解器）
-- 3. 激光回环匹配参数
-- 4. 约束的置信权重
-- 5. 3D/2D 匹配器配置
POSE_GRAPH = {
  -- 每收到多少帧节点数据，执行一次全局图优化
  optimize_every_n_nodes = 90,       -- 30~90	越小优化越频繁
  -- 约束构建参数
  constraint_builder = {
    sampling_ratio = 0.3,          -- 表示在构建回环或邻接约束时，仅使用30%的数据尝试构建，以减轻计算负担 0.3~1.0	越大越精准
    max_constraint_distance = 15., -- 最大约束生成距离，超过这个距离就不尝试构建约束（即不会做回环检测）.10~20	控制回环触发距离
    min_score = 0.55,    --Fast csm的最低分数，高于此分数才进行优化。对回环匹配结果进行评分（Fast CSM 输出），如果低于 0.55，就不会添加为约束。，0.5~0.6	提高可提升约束质量
    global_localization_min_score = 0.6,  --全局定位最小分数，低于此分数则认为目前全局定位不准确。全局定位（比如重定位）时的匹配分数门槛，低于此认为失败。
    loop_closure_translation_weight = 1.1e4, -- 回环约束的平移，数值越大，越信任回环结果
    loop_closure_rotation_weight = 1e5,      -- 回环约束的旋转权重，数值越大，越信任回环结果
    log_matches = true,                      -- 是否打印匹配信息（debug用）到控制台

    -- Fast Correlative Scan Matcher 2D 设置
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,                     -- 线性/角度搜索窗口：用于构建约束时，最大允许搜索的平移与旋转范围。
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,                    -- 控制搜索精度，数值越高精度越高，计算越慢
    },
    -- ceres_scan_matcher 2D 设置
    ceres_scan_matcher = {                           -- 用于优化每个匹配点的代价函数权重
      occupied_space_weight = 20.,                   -- 与地图重合区域的匹配程度
      translation_weight = 10.,                      -- 平移偏差惩罚。适度调大	提升约束强度
      rotation_weight = 1.,                          -- 角度偏差惩罚
      ceres_solver_options = {
        use_nonmonotonic_steps = true,               -- 允许代价函数短时间内变差（可能帮助跳出局部最优）
        max_num_iterations = 10,                     -- 最大优化步数
        num_threads = 1,                             -- 使用1线程
      },
    },
    -- Fast CSM 和 Ceres 用于 3D（可忽略，如果你只用2D）
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  --   匹配器约束优化参数
  matcher_translation_weight = 5e2,    -- 局部匹配中，平移的优化权重
  matcher_rotation_weight = 1.6e3,     -- 旋转误差的优化权重
--   优化器设置 
  optimization_problem = {
    huber_scale = 1e1,                 -- 鲁棒核函数的尺度（用于减少异常值影响）
    acceleration_weight = 1.1e2,       -- 对IMU加速度和旋转先验的置信度（如果你用IMU）
    rotation_weight = 1.6e4,
    local_slam_pose_translation_weight = 1e5,  -- 对于前端（local SLAM）估计位姿的信任程度
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,         -- 对里程计的置信权重（如果你开启了 use_odometry）
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,  -- 如果使用 GPS/FIX pose 等“固定参考坐标”，这些就是它们的权重
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false, -- 是否在GPS误差优化中使用容忍核函数
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,                 -- 是否打印每次优化的 Ceres 总结信息
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    -- 后端优化器的线程与迭代配置
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,    -- 闭环之后的最大优化迭代次数
  global_sampling_ratio = 0.003,     -- 全局回环检测采样率（越小越省资源）
  log_residual_histograms = true,    -- 打印每次优化残差的直方图（debug）
  global_constraint_search_after_n_seconds = 10.,    -- 每10秒尝试进行一次全局闭环检测
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}
