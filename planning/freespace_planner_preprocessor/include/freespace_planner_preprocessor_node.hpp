#ifndef FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_
#define FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <motion_utils/vehicle/vehicle_state_checker.hpp>

#include <autoware_auto_planning_msgs/msg/mission.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <cstring>
#include <memory>

namespace freespace_planner_preprocessor
{
/* *************************************************************************************************
 */
class State;
class FreeSpacePlannerPreprocessorNode;

using autoware_auto_planning_msgs::msg::Mission;
using autoware_auto_vehicle_msgs::msg::Engage;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;
using motion_utils::VehicleStopChecker;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Bool;
using std_msgs::msg::Int16MultiArray;
using tier4_planning_msgs::msg::VelocityLimit;
/* *************************************************************************************************
 */
enum GoalType { Preset, Ultimate };

struct ContextData
{
  bool has_new_mission{false};

  Mission::ConstSharedPtr mission_ptr;
  LaneletRoute::ConstSharedPtr route_ptr;
  Int16MultiArray::ConstSharedPtr plan_result_ptr;
  Bool::ConstSharedPtr parking_state_ptr;

  std::shared_ptr<State> state;
  std::shared_ptr<State> prev_state;
  std::unordered_map<std::string, std::shared_ptr<State>> state_map;

  Pose current_pose;
  Pose goal_pose;
  GoalType goal_type;

  Pose station;
  Pose garbage_dump;
  Pose supply_station;
  std::vector<Pose> preset_points;
  std::vector<Pose> preset_points_of_garbage_dump;
  std::vector<Pose> preset_points_of_supply_station;
  size_t index_of_preset_point;

  int plan_count;
  double current_max_velocity;
};
/* *************************************************************************************************
 */
geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion quaternion_msg;
  tf2::convert(quaternion, quaternion_msg);
  return quaternion_msg;
};
using freespace_planner_preprocessor::createQuaternionFromYaw;

double GetYawFromPose(const Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}
using freespace_planner_preprocessor::GetYawFromPose;

double AngDiff(const double angle_a, const double angle_b)
{
  double diff = angle_a - angle_b;
  while (diff > M_PI) {
    diff -= 2.0 * M_PI;
  }
  while (diff < -M_PI) {
    diff += 2.0 * M_PI;
  }
  return diff;
}
using freespace_planner_preprocessor::AngDiff;

double Rad2Deg(const double rad)
{
  double deg = rad * 180.0 / M_PI;
  return deg;
}
using freespace_planner_preprocessor::Rad2Deg;

double Calculate2dDistance(const Pose & pose1, const Pose & pose2)
{
  return std::hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y);
}
using freespace_planner_preprocessor::Calculate2dDistance;

Pose CalculateGlobalPose(const Pose & base_pose, const Pose & relative_pose)
{
  Pose global_pose;
  const double yaw = tf2::getYaw(base_pose.orientation);
  global_pose.position.x = base_pose.position.x + relative_pose.position.x * std::cos(yaw) -
                           relative_pose.position.y * std::sin(yaw);
  global_pose.position.y = base_pose.position.y + relative_pose.position.x * std::sin(yaw) +
                           relative_pose.position.y * std::cos(yaw);
  global_pose.position.z = base_pose.position.z + relative_pose.position.z;
  global_pose.orientation = createQuaternionFromYaw(yaw + tf2::getYaw(relative_pose.orientation));
  return global_pose;
}
using freespace_planner_preprocessor::CalculateGlobalPose;
/* *************************************************************************************************
 */
class State
{
protected:
  FreeSpacePlannerPreprocessorNode * context_;
  std::string name_;

public:
  explicit State(std::string name) : name_(name) {}

  virtual ~State() { context_ = nullptr; }

  void SetContext(FreeSpacePlannerPreprocessorNode * context) { context_ = context; }

  std::string name() const { return name_; }

  virtual void Handle() = 0;
};

class MessageForward : public State
{
private:
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<Bool>::SharedPtr parking_state_sub_;
  void LoadSubscribers(FreeSpacePlannerPreprocessorNode * context);

public:
  explicit MessageForward(FreeSpacePlannerPreprocessorNode * context) : State("MessageForward")
  {
    SetContext(context);
    LoadSubscribers(context);
  }

  void Handle() override;
};

class Idle : public State
{
public:
  explicit Idle(FreeSpacePlannerPreprocessorNode * context) : State("Idle") { SetContext(context); }
  void Handle() override;
};

class ModifyGoalPose : public State
{
public:
  explicit ModifyGoalPose(FreeSpacePlannerPreprocessorNode * context) : State("ModifyGoalPose")
  {
    SetContext(context);
  }
  void Handle() override;
};

class WaitPlanResult : public State
{
private:
  rclcpp::Subscription<Int16MultiArray>::SharedPtr plan_result_sub_;
  void LoadSubscribers(FreeSpacePlannerPreprocessorNode * context);

public:
  explicit WaitPlanResult(FreeSpacePlannerPreprocessorNode * context) : State("WaitPlanResult")
  {
    SetContext(context);
    LoadSubscribers(context);
  }
  void Handle() override;
};

class StartUp : public State
{
private:
  void LoadSubscribers(FreeSpacePlannerPreprocessorNode * context);

public:
  explicit StartUp(FreeSpacePlannerPreprocessorNode * context) : State("StartUp")
  {
    SetContext(context);
  }
  void Handle() override;
};

class WaitForArrived : public State
{
private:
  bool IsCloseTo(const Pose & destination);
  bool HasStopped(const double stop_duration = 0.0);
  bool HasMetError();

public:
  explicit WaitForArrived(FreeSpacePlannerPreprocessorNode * context) : State("WaitForArrived")
  {
    SetContext(context);
  }
  void Handle() override;
};

/* *************************************************************************************************
 */
class FreeSpacePlannerPreprocessorNode : public rclcpp::Node
{
public:
  explicit FreeSpacePlannerPreprocessorNode(const rclcpp::NodeOptions & node_options);
  std::shared_ptr<State> CreateState(const std::string name);
  void SwitchTo(const std::string name);
  void SwitchState();
  bool engage(bool button);
  bool SetVelocityLimit(double velocity);
  bool LoadStationInfo();

  ContextData data_base;

  rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<Bool>::SharedPtr parking_state_pub_;

private:
  void Reset();
  void OnTimer();
  void OnMission(const Mission::ConstSharedPtr msg);
  void OnOdometry(const Odometry::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
};
/* *************************************************************************************************
 */
}  // namespace freespace_planner_preprocessor
#endif  // FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_