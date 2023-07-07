#ifndef FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_
#define FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <autoware_auto_vehicle_msgs/msg/detail/engage__struct.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <std_msgs/msg/bool.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
namespace freespace_planner_preprocessor
{

using autoware_planning_msgs::msg::LaneletRoute;
using autoware_adapi_v1_msgs::msg::RouteState;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;    
using tier4_planning_msgs::msg::Scenario;
using std_msgs::msg::Bool;
using autoware_auto_vehicle_msgs::msg::Engage;
using motion_utils::VehicleStopChecker;
using tier4_planning_msgs::msg::VelocityLimit;

struct Coordinate {
  double x;
  double y;
  double yaw;
};

enum GoalType {
  Preview,
  Ultimate
};

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion quaternion_msg;
  tf2::convert(quaternion, quaternion_msg);
  return quaternion_msg;
};
using freespace_planner_preprocessor::createQuaternionFromYaw;

double GetYawFromPose(const Pose &pose){
  return tf2::getYaw(pose.orientation);
}
using freespace_planner_preprocessor::GetYawFromPose;

double AngDiff(const double angle_a,const double angle_b){
  double diff = angle_a - angle_b;
  while( diff > M_PI) {
    diff -= 2.0 * M_PI;
  }
  while (diff < -M_PI) {
    diff += 2.0 * M_PI;
  }
  return diff;
}
using freespace_planner_preprocessor::AngDiff;


double Rad2Deg(const double rad){
  double deg = rad * 180.0 / M_PI;
  return deg;
}
using  freespace_planner_preprocessor::Rad2Deg;

double Calculate2dDistance(const Pose &pose1, const Pose &pose2){
  return std::hypot(pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y);
}
using freespace_planner_preprocessor::Calculate2dDistance;

class Preprocessor;
class FreeSpacePlannerPreprocessorNode;

class State {
 protected:
  std::shared_ptr<Preprocessor> preprocessor_;
  std::string name_;

 public:
  explicit State(std::string name) : name_(name) {
  }
  virtual ~State() {
  }

  void set_context(std::shared_ptr<Preprocessor> preprocessor) {
    this->preprocessor_ = preprocessor;
  }

  virtual void Handle() = 0;

  std::string name() const {return name_;}
};

class Idle : public State {
  private:
    bool hasInitial();
  public:
    Idle() : State("Idle") {
    }
    void Handle() override;
};

class ModifyGoalPose : public State {
  private:
    std::vector<Coordinate> relative_position_table_{
      {5.0, 0.0, 0}, //TODO：目前只有一个，后面根据场景选择不同的位置
    };
  public:
    ModifyGoalPose() : State("ModifyGoalPose") {
    }
    void Handle() override;
};

class StartUp : public State {
  public:
    StartUp() : State("StartUp") {
    }
    void Handle() override;
};

class WaitForArrival : public State {
  private:
    bool isCloseToDestination();
    bool hasMetErrorRequirements();
  public:
    WaitForArrival() : State("WaitForArrival") {
    }
    void Handle() override;
};

class Preprocessor : public std::enable_shared_from_this<Preprocessor> {
 private:
  std::shared_ptr<State> prev_state_;
  std::shared_ptr<State> state_;

 public:
  explicit Preprocessor(FreeSpacePlannerPreprocessorNode * node);
  void SwitchToState();
  void SwitchTo(std::shared_ptr<State> state);
  void onTimer();

  FreeSpacePlannerPreprocessorNode *node_;
  bool has_new_odom_;
  bool has_new_route_;
  bool has_new_route_state_;
  GoalType goal_type_;
  LaneletRoute route_;
  VehicleStopChecker vehicle_stop_checker_;
};

class FreeSpacePlannerPreprocessorNode : public rclcpp::Node
{
private:
    std::shared_ptr<Preprocessor> preprocessor_; 
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
    rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
    rclcpp::Subscription<Bool>::SharedPtr parking_state_sub_;
    rclcpp::Subscription<Scenario>::SharedPtr scenario_sub_;
    rclcpp::Subscription<Engage>::SharedPtr engage_sub_;

    void onTimer();
    void onOdometry(const Odometry::ConstSharedPtr msg);
    void onRoute(const LaneletRoute::ConstSharedPtr msg);
    void onRouteState(const RouteState::ConstSharedPtr msg);
    void onParkingState(const Bool::ConstSharedPtr msg);
    void onScenario(const Scenario::ConstSharedPtr msg);
    void onEngage(const Engage::ConstSharedPtr msg);

public:
    explicit FreeSpacePlannerPreprocessorNode(const rclcpp::NodeOptions & node_options);

    rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;
    rclcpp::Publisher<RouteState>::SharedPtr route_state_pub_;
    rclcpp::Publisher<Bool>::SharedPtr parking_state_pub_;
    rclcpp::Publisher<Scenario>::SharedPtr scenario_pub_;
    rclcpp::Publisher<Engage>::SharedPtr engage_pub_;
    rclcpp::Publisher<VelocityLimit>::SharedPtr velocity_limit_pub_;

    Odometry::ConstSharedPtr odom_ptr_;
    LaneletRoute::ConstSharedPtr route_ptr_;
    RouteState::ConstSharedPtr route_state_ptr_;
    Scenario::ConstSharedPtr scenario_ptr_;
    Engage::ConstSharedPtr engage_ptr_;
    bool is_parking_completed_;
};
} // namespace freespace_planner_preprocessor
#endif // FREESPACE_PLANNER_PREPROCESSOR_NODE_HPP_