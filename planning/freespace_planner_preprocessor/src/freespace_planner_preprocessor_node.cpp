#include <freespace_planner_preprocessor_node.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace freespace_planner_preprocessor {

bool Idle::hasInitial(){
  if(preprocessor_->node_->odom_ptr_ && preprocessor_->node_->route_ptr_){
    return true;
  }
  return false;
}

void Idle::Handle()
{
  if (!hasInitial()) return;
  if (preprocessor_->has_new_route_){
    preprocessor_->goal_type_ = GoalType::Preview;
    preprocessor_->has_new_route_ = false;
    preprocessor_->SwitchTo(std::make_shared<ModifyGoalPose>());
  }
}

void ModifyGoalPose::Handle(){
  Pose raw_goal_pose = preprocessor_->node_->route_ptr_->goal_pose;
  Pose modified_goal_pose = raw_goal_pose;
  if (preprocessor_->goal_type_ == GoalType::Preview) {
    const double yaw = tf2::getYaw(raw_goal_pose.orientation);
    const Coordinate relative_position = relative_position_table_[0];
    modified_goal_pose.position.x = raw_goal_pose.position.x + (relative_position.x * cos(yaw) + relative_position.y * -sin(yaw));
    modified_goal_pose.position.y = raw_goal_pose.position.y + (relative_position.x * sin(yaw) + relative_position.y * cos(yaw));
    modified_goal_pose.orientation = createQuaternionFromYaw(yaw + relative_position.yaw);
  } 
  LaneletRoute route = *(preprocessor_->node_->route_ptr_);
  route.goal_pose = modified_goal_pose;
  preprocessor_->node_->route_pub_->publish(route);
  preprocessor_->route_ = route;
  preprocessor_->SwitchTo(std::make_shared<StartUp>());
}

void StartUp::Handle(){
  // 当第一次倒车的时候，会ARRIVED，然后卡住动不了
  if (preprocessor_->node_->route_state_ptr_ && preprocessor_->node_->route_state_ptr_->state == RouteState::ARRIVED){
    RouteState msg;
    msg.state = RouteState::SET;
    preprocessor_->node_->route_state_pub_->publish(msg);
  }
  Engage msg;
  msg.engage = true;
  preprocessor_->node_->engage_pub_->publish(msg);
  preprocessor_->SwitchTo(std::make_shared<WaitForArrival>());
  std_msgs::msg::Bool is_parking_complete;
  is_parking_complete.data = false;
  preprocessor_->node_->parking_state_pub_->publish(is_parking_complete);
}

bool WaitForArrival::isCloseToDestination(){
  bool has_stopped = preprocessor_->vehicle_stop_checker_.isVehicleStopped(3);
  double distance = Calculate2dDistance(preprocessor_->route_.goal_pose, preprocessor_->node_->odom_ptr_->pose.pose);
  bool is_nearby = distance <= 1.0 ? true : false;
  if (has_stopped && is_nearby){
    return true;
  }
  return false;
}

bool WaitForArrival::hasMetErrorRequirements(){
  Pose goal_pose = preprocessor_->route_.goal_pose;
  Pose vehicle_pose = preprocessor_->node_->odom_ptr_->pose.pose;

  // calculate yaw error
  const double yaw4vehicle = GetYawFromPose(vehicle_pose);
  const double yaw4goal = GetYawFromPose(goal_pose);
  const double yaw_error = Rad2Deg(AngDiff(yaw4vehicle, yaw4goal)); 

  // calculate longitudinal error
  const double dx = goal_pose.position.x - vehicle_pose.position.x;
  const double dy = goal_pose.position.y - vehicle_pose.position.y;
  const double theta = atan2(dy, dx);
  const double longitudinal_error = sqrt(pow(dx, 2) + pow(dy, 2)) * cos(AngDiff(theta, yaw4goal));
  // calculate lateral error
  const double lateral_error = sqrt(pow(dx, 2) + pow(dy, 2)) * sin(AngDiff(theta, yaw4goal));

  RCLCPP_INFO(preprocessor_->node_->get_logger(), "lateral_error is %f", lateral_error);
  // 先放宽限制，跑通程序
  if (longitudinal_error <= 2.0 && yaw_error <= 3.14 && fabs(lateral_error)<= 0.05){
    RCLCPP_INFO(preprocessor_->node_->get_logger(), "has met the error requirement return true");
    return true;
  }
  RCLCPP_INFO(preprocessor_->node_->get_logger(), "hasn't met the error requirement return false");
  return false;
}

void WaitForArrival::Handle(){
  if(isCloseToDestination()){
    switch (preprocessor_->goal_type_){
      case GoalType::Ultimate: {
        if (hasMetErrorRequirements()){
          RouteState route_state;
          route_state.state = RouteState::ARRIVED;
          std_msgs::msg::Bool is_parking_complete;
          is_parking_complete.data = true;
          preprocessor_->node_->route_state_pub_->publish(route_state);
          preprocessor_->node_->parking_state_pub_->publish(is_parking_complete);
          preprocessor_->SwitchTo(std::make_shared<Idle>());
        } else {
          preprocessor_->SwitchTo(std::make_shared<ModifyGoalPose>());
          preprocessor_->goal_type_ = GoalType::Preview;
        }
        break;
      }
      case GoalType::Preview: {
        preprocessor_->SwitchTo(std::make_shared<ModifyGoalPose>());
        preprocessor_->goal_type_ = GoalType::Ultimate;
        break;
      }
      default: {
        break;
      }
    }
  }
}

Preprocessor::Preprocessor(FreeSpacePlannerPreprocessorNode * node) :  node_(node), vehicle_stop_checker_(node){
  has_new_odom_ = false; 
  has_new_route_ = false;
  has_new_route_state_ = false; 
  state_ = nullptr;
}

void Preprocessor::SwitchToState(){
  if (node_->odom_ptr_ == nullptr){
    return;
  }
  if (state_->name() == "Idle"){
    SwitchTo(std::make_shared<Idle>());
  }
  if (state_->name() == "ModifyGoalPose"){
    SwitchTo(std::make_shared<Idle>());
  }
  if (state_->name() == "StartUp"){
    SwitchTo(std::make_shared<Idle>());
  }
  if (state_->name() == "WaitForArrival"){
    SwitchTo(std::make_shared<Idle>());
  }
}

void Preprocessor::SwitchTo(std::shared_ptr<State> state) {
  prev_state_ = state_;
  state_ = state;
  std::shared_ptr<Preprocessor> this_ptr = shared_from_this();
  state_->set_context(this_ptr);
  RCLCPP_INFO(node_->get_logger(), "switch state: %s -> %s", prev_state_ ? prev_state_->name().c_str() : "nullptr", state_ ? state->name().c_str() : "nullptr");
}

void Preprocessor::onTimer(){
  // 主动切换状态
  if(has_new_route_)
  {
    SwitchToState();
  }
  state_->Handle();
}

FreeSpacePlannerPreprocessorNode::FreeSpacePlannerPreprocessorNode(const rclcpp::NodeOptions & node_options): Node("freespace_planner_preprocessor", node_options){

  // 初始化
  preprocessor_ = std::make_shared<Preprocessor>(this);
  preprocessor_->SwitchTo(std::make_shared<Idle>());

  // 订阅
  route_sub_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&FreeSpacePlannerPreprocessorNode::onRoute, this, std::placeholders::_1)
  );
  route_state_sub_ = create_subscription<RouteState>(
    "~/input/route_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&FreeSpacePlannerPreprocessorNode::onRouteState, this, std::placeholders::_1)
  );
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 100,
    std::bind(&FreeSpacePlannerPreprocessorNode::onOdometry, this, std::placeholders::_1)
  );
  scenario_sub_ = create_subscription<Scenario>(
    "~/input/scenario", 1, std::bind(&FreeSpacePlannerPreprocessorNode::onScenario, this, std::placeholders::_1));
  parking_state_sub_ = create_subscription<Bool>(
    "~/input/is_completed", rclcpp::QoS{1}.transient_local(),
    std::bind(&FreeSpacePlannerPreprocessorNode::onParkingState, this, std::placeholders::_1));

  // 发布
  route_pub_ = create_publisher<LaneletRoute>(
    "~/output/route", rclcpp::QoS{1}.transient_local()
  );
  route_state_pub_ = create_publisher<RouteState>(
    "~/output/route_state", rclcpp::QoS{1}.transient_local()
  );
  scenario_pub_ = create_publisher<Scenario>(
    "~/output/scenario", rclcpp::QoS{1}
  );
  parking_state_pub_ = create_publisher<Bool>(
    "~/output/is_completed", rclcpp::QoS{1}.transient_local()
  );
  engage_pub_ = create_publisher<Engage>(
    "~/output/engage", rclcpp::QoS{1}.transient_local()
  );
  // 定时器
  const double loop_rate = 10;
  const auto period_ns = rclcpp::Rate(loop_rate).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&FreeSpacePlannerPreprocessorNode::onTimer, this));
}

void FreeSpacePlannerPreprocessorNode::onOdometry(const Odometry::ConstSharedPtr msg){
  odom_ptr_ = msg;
  preprocessor_->has_new_odom_ = true;
}

void FreeSpacePlannerPreprocessorNode::onRoute(const LaneletRoute::ConstSharedPtr msg){
  route_ptr_ = msg;
  preprocessor_->has_new_route_ = true;
}

void FreeSpacePlannerPreprocessorNode::onRouteState(const RouteState::ConstSharedPtr msg){
  route_state_ptr_ = msg;
  preprocessor_->has_new_route_state_ = true;
}

void FreeSpacePlannerPreprocessorNode::onScenario(const Scenario::ConstSharedPtr msg){
  // 这里只是做中转，从scenario_selector->freespace_planner_preprocessor->freespace_planner
  scenario_ptr_ = msg;
  scenario_pub_->publish(*scenario_ptr_); 
}

void FreeSpacePlannerPreprocessorNode::onParkingState(const Bool::ConstSharedPtr msg){
  // 需要发到scenario_selector那边，不让不能正常切换lane_driving模式
  is_parking_completed_ = msg->data;
}

void FreeSpacePlannerPreprocessorNode::onEngage(const Engage::ConstSharedPtr msg){
  engage_ptr_ = msg;
}

void FreeSpacePlannerPreprocessorNode::onTimer(){
  preprocessor_->onTimer();
}
} // namespace freespace_planner_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(freespace_planner_preprocessor::FreeSpacePlannerPreprocessorNode)

// // 如果使用ros2 run/launch来单独启动，需要注释掉下面的main函数
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   rclcpp::spin(std::make_shared<freespace_planner_preprocessor::FreeSpacePlannerPreprocessorNode>(rclcpp::NodeOptions()));
//   rclcpp::shutdown();
//   return 0;
// }
