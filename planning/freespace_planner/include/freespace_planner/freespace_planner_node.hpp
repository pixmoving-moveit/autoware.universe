// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_

#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <autoware_auto_planning_msgs/srv/get_trajectory.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include "coverage_planning/coverage_planning_core.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/mission.hpp>
#include <autoware_auto_planning_msgs/srv/get_trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <route_handler/route_handler.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace freespace_planner
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Mission;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::srv::GetTrajectory;
using autoware_planning_msgs::msg::LaneletRoute;
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::RRTStar;
using freespace_planning_algorithms::RRTStarParam;
using freespace_planning_algorithms::VehicleShape;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Int16MultiArray;
using tier4_planning_msgs::msg::Scenario;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double coverage_sweeping_velocity;
  double reloading_velocity;
  double boundary_sweeping_velocity;
  double searching_sweeping_velocity;
  double dumping_trash_velocity;
  double parking_mpc_velocity;
  double navigation_mpc_velocity;
  double update_rate;  // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // ros
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_result_pub_;

  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;
  bool is_planning_success_ = false;


  coverage_planning::CoveragePlanningCore coverage_planning_core_;
  lanelet::LaneletMapPtr global_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr map_bin_;
  bool is_map_loaded_ = false;

  Mission::ConstSharedPtr current_mission_ = std::make_shared<Mission>(Mission());

  LaneletRoute::ConstSharedPtr route_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Scenario::ConstSharedPtr scenario_;
  Odometry::ConstSharedPtr odom_;
  std::shared_ptr<route_handler::RouteHandler> route_handler_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onScenario(const Scenario::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onMission(const Mission::ConstSharedPtr msg);
  void onLaneletMap(const HADMapBin::ConstSharedPtr msg);

  void onTimer();

  void reset();
  bool isPlanRequired();
  void planTrajectory();
  void planTrajectoryCoverage();
  void updateTargetIndex();
  void initializePlanningAlgorithm();

  TransformStamped getTransform(const std::string & from, const std::string & to);
};
}  // namespace freespace_planner

#endif  // FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
