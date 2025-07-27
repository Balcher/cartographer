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

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,  -- 是否使用IMU数据
  min_range = 0.,       -- 深度数据最小范围,限制激光数据的有效距离和高度范围
  max_range = 30.,      -- 深度数据最大范围
  min_z = -0.8,         -- 传感器数据超出有效范围最大值时，按此值来处理，通常用于滤掉杂波（如近距离反射、天花板等），
  max_z = 2.,
  missing_data_ray_length = 5.,    -- 对于激光射线未命中物体的情况，用这个默认长度来补偿（用于自由空间估计）
  num_accumulated_range_data = 1,  -- 每几帧激光合并一次建图，1表示每帧都处理，响应速度快但计算量大
  voxel_filter_size = 0.025,       -- 对点云进行体素滤波，单位为米，降低点数，提高速度

  -- 自适应体素滤波，用于对激光点进行稀疏化
  adaptive_voxel_filter = {
    max_length = 0.5,              -- 体素边长上限
    min_num_points = 200,          -- 滤波后至少保留的点数
    max_range = 50.,               -- 滤波有效范围
  },

  -- 专门用于回环检测阶段
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,         -- 设定体素的最大边长为0.9米。这意味着在创建体素网格时，每个体素的边长不会超过0.9米
    min_num_points = 100,     -- 最小点数设置为100。这表示在一个体素内，至少需要有100个点才能被考虑为有效的体素。
    max_range = 50.,          -- 最大范围设置为50米。这意味着在进行滤波时，考虑的最大距离为50米
  },
---- 扫描匹配相关 ----
  use_online_correlative_scan_matching = false,    -- 是否使用实时回环检测来进行前端的扫描匹配
  real_time_correlative_scan_matcher = {           -- 实时相关扫描匹配器的配置
    linear_search_window = 0.1,                    -- 线性搜索窗口的大小，单位为米，表示搜索范围为0.1米
    angular_search_window = math.rad(20.),         -- 角度搜索窗口的大小，
    translation_delta_cost_weight = 1e-1,          -- 平移增量在匹配过程中的影响
    rotation_delta_cost_weight = 1e-1,             -- 旋转增量成本的权重，用于调整旋转增量在匹配过程中的影响
  },

-- 使用ceres solver优化帧间位姿
  ceres_scan_matcher = {
    occupied_space_weight = 1.,                    -- 权重因子，用于衡量点云与已建地图的匹配程度，值越大，匹配点云到占用栅格的效果越重要，小值意味着允许更多“偏离地图”的匹配
    translation_weight = 10.,                      -- 在优化过程中，平移（x，y，z）偏移项的惩罚权重，值越大表示我们更保守，不愿意让匹配器改变当前的平移估计太多
    rotation_weight = 40.,                         -- 优化中，旋转角度偏移的惩罚权重
    ceres_solver_options = {
      use_nonmonotonic_steps = false,              -- 设置为 false 表示每一步迭代都必须让目标函数下降（收敛更稳定但速度可能慢）
      max_num_iterations = 20,                     -- 设置最多迭代 20 次，用于防止长时间卡死。
      num_threads = 1,                             -- Ceres 求解器使用的线程数
    },
  },

  ---- 运动过滤器 ----
  -- 在5s内移动小于0.2m或转动小于1度，则跳过该帧
  motion_filter = {  
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),-- 运动过滤，检测运动变化，避免机器人静止时插入数据
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,
  ---- 设置如何从已有数据外推当前位姿 ----
  pose_extrapolator = {
    use_imu_based = false,               -- 使用IMU来外推，使用速度等信息，如果使用IMU，则可以设为true；若为false，则使用 constant_velocity 模块，即假设机器人以恒定速度运动的简单模型。
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {
      pose_queue_duration = 5.,          -- 在过去 5 秒内的位姿都会被考虑用于优化
      gravity_constant = 9.806,          -- 标准重力加速度，用于加速度方向的对齐
      pose_translation_weight = 1.,      -- 对位姿平移项的优化权重
      pose_rotation_weight = 1.,         -- 对位姿旋转项的优化权重
      imu_acceleration_weight = 1.,      -- IMU加速度数据对优化的影响权重
      imu_rotation_weight = 1.,          -- IMU角速度对优化的影响权重
      odometry_translation_weight = 1.,  -- 里程计平移数据的权重（如果启用里程计输入）
      odometry_rotation_weight = 1.,     -- 里程计旋转数据的权重
      solver_options = {                 -- 求解器选项（用于非线性优化）
        use_nonmonotonic_steps = false;  -- 不允许使用非单调下降的步长，有助于优化过程更稳定
        max_num_iterations = 10;         -- 每次优化的最大迭代次数，较小值提高实时性但可能牺牲精度
        num_threads = 1;                 -- 优化时使用的线程数，设置为1可避免多线程带来的调度问题
      },
    },
  },
  ---- 子图配置 ----
  submaps = {
    num_range_data = 90,                 -- 设置每个子图包含多少帧数据，即累积多少帧后新建子图（越小建图更频繁，越大图会更平滑，但回环更难匹配）
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",    -- 概率栅格地图，适合导航和回环
      resolution = 0.05,                 -- 分辨率越小地图越精细，计算和内存开销越大
    },
    range_data_inserter = {              -- 激光数据插入器配置
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",    -- 指定使用哪种方式将激光数据插入到地图中，此处使用的是针对概率地图的插入器。
      probability_grid_range_data_inserter = {
        insert_free_space = true,        -- 是否将激光束路径上的空闲空间也更新为“未占据”，能更清楚地划分障碍物与自由空间。
        hit_probability = 0.55,          -- 当一个激光点命中某个栅格时，对该格子占据概率的提升值（大于0.5表示更倾向于“被占据”）
        miss_probability = 0.49,         -- 当激光束穿过某格子而未命中时，对该格子占据概率的下降值（小于0.5表示更倾向于“空闲”）
      },
      tsdf_range_data_inserter = {       -- 虽然此配置中 range_data_inserter_type 为 "PROBABILITY_GRID_INSERTER_2D"，但这里也提供了 TSDF 插入器的配置（实际不会被使用，除非切换类型）
        truncation_distance = 0.3,       -- 用于控制距离场构建的细节
        maximum_weight = 10.,            -- 用于控制距离场构建的细节
        update_free_space = false,       -- 通常用于不处理空闲区域
        normal_estimation_options = {    -- 提供法线估计方法，用于精细建模
          num_normal_samples = 4,        -- 对每个插入点，采样周围 4 个邻近点来估计法线方向
          sample_radius = 0.5,           -- 在多大范围（米）内寻找邻点来估计法线。
        },
        project_sdf_distance_to_scan_normal = true,                    -- 启用后，TSDF值投影到扫描法线方向，而不是激光束方向
        update_weight_range_exponent = 0,                              -- 控制权重随深度距离变化的指数因子
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5, -- 控制插入点的法线方向与激光束方向之间夹角对权重的影响
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,     -- 控制插入点距离命中点的远近对权重的影响
      },
    },
  },
}
