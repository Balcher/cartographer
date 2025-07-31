/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

// 该函数用于根据给定的Lua参数字典创建轨迹构建器选项
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class TrajectoryBuilderInterface {
 public:
  //  插入的结果信息
  struct InsertionResult {
    NodeId node_id;  // 节点ID
    std::shared_ptr<const TrajectoryNode::Data>
        constant_data;  // 常量数据的共享指针
    std::vector<std::shared_ptr<const Submap>>
        insertion_submaps;  // 插入的子图列表
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  // 定义一个回调函数类型，用于处理局部SLAM处理后的结果，主要用于报告传感器数据的插入情况。
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  //  定义了传感器的类型和ID
  struct SensorId {
    // 传感器类型的枚举类
    enum class SensorType {
      RANGE = 0,         // 激光雷达传感器
      IMU,               // 惯性测量单元传感器
      ODOMETRY,          // 里程计传感器
      FIXED_FRAME_POSE,  // 固定框架姿态传感器
      LANDMARK,          // 地标传感器
      LOCAL_SLAM_RESULT  // 局部SLAM结果传感器
    };

    SensorType type;  // 传感器类型
    std::string id;   // 传感器ID

    // 运算符重载，用于比较两个传感器ID是否相等
    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    // 运算符重载，用于比较两个传感器ID的大小
    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) =
      delete;  // 禁止拷贝构造
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;  // 禁止赋值操作

  // 允许添加不同传感器的数据，包括：时间标记的点云数据、IMU数据、里程计数据、固定框架姿态数据和地标数据。
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  // 该函数允许直接将局部slam结果添加到姿态图中，需要注意的是，如果轨迹已经有
  // LocalTrajectoryBuilder2D/3D，则不能添加局部slam结果。
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
