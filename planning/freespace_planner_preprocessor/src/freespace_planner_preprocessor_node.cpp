#include <freespace_planner_preprocessor_node.hpp>

#include <cstdint>

namespace freespace_planner_preprocessor
{
void MessageForward::LoadSubscribers(FreeSpacePlannerPreprocessorNode* context)
{
  auto OnRoute = [context](const LaneletRoute::ConstSharedPtr msg) {
    if (context->data_base.mission_ptr &&
        (context->data_base.mission_ptr->free_space_sweeping_mode == Mission::RELOADING ||
        context->data_base.mission_ptr->free_space_sweeping_mode == Mission::DUMPING_TRASH))
    {
      // TODO: 被激活scenario_selector会通过该回调传入数据，一旦切换为中转模式时，会直接被转发出去
      return;
    }
    context->data_base.route_ptr = msg;
  };
  auto OnParkingState = [context](const Bool::ConstSharedPtr msg) {
    if (context->data_base.mission_ptr &&
        (context->data_base.mission_ptr->free_space_sweeping_mode == Mission::RELOADING ||
        context->data_base.mission_ptr->free_space_sweeping_mode == Mission::DUMPING_TRASH))
    {
      // 当mission是RELOADING或者DUMPING_TRASH时，其信息由其他状态来控制发布,不作中转
      return;
    }
    context->data_base.parking_state_ptr = msg;
  };
  route_sub_ = context->create_subscription<LaneletRoute>("~/input/route", rclcpp::QoS{ 1 }.transient_local(), OnRoute);
  parking_state_sub_ =
      context->create_subscription<Bool>("~/input/parking_state", rclcpp::QoS{ 1 }.transient_local(), OnParkingState);
}

void MessageForward::Handle()
{
  if (context_->data_base.route_ptr)
  {
    context_->route_pub_->publish(*context_->data_base.route_ptr);
    context_->data_base.route_ptr = nullptr;
  }
  if (context_->data_base.parking_state_ptr)
  {
    context_->parking_state_pub_->publish(*context_->data_base.parking_state_ptr);
    context_->data_base.parking_state_ptr = nullptr;
  }
}

void Idle::Handle()
{
  // TODO: 检查状态复位没有，如果没有，就复位
}

void ModifyGoalPose::Handle()
{
  if (context_->data_base.goal_type == GoalType::Preset)
  {
    int idx = context_->data_base.index_of_preset_point;
    context_->data_base.goal_pose = context_->data_base.preset_points[idx];
  }
  else
  {
    context_->data_base.goal_pose = context_->data_base.station;
  }
  LaneletRoute route;
  route.goal_pose = context_->data_base.goal_pose;
  context_->route_pub_->publish(route);
  context_->SwitchTo("WaitPlanResult");
}

void WaitPlanResult::LoadSubscribers(FreeSpacePlannerPreprocessorNode* context)
{
  auto callback = [context](const Int16MultiArray::ConstSharedPtr msg) { context->data_base.plan_result_ptr = msg; };
  plan_result_sub_ = context->create_subscription<Int16MultiArray>("~/input/planning_result",
                                                                   rclcpp::QoS{ 1 }.transient_local(), callback);
}

void WaitPlanResult::Handle()
{
  if (context_->data_base.plan_result_ptr)
  {
    if (context_->data_base.plan_result_ptr->data[0] == 1)
    {
      context_->data_base.plan_result_ptr = nullptr;
      context_->data_base.plan_count = 0;
      context_->SwitchTo("StartUp");
      return;
    }
    if (context_->data_base.plan_result_ptr->data[1] == 1)
    {
      // 规划超时,每个点试2次，不行换下一个
      context_->data_base.plan_result_ptr = nullptr;
      if (++context_->data_base.plan_count >= 2)
      {
        // 失败超过2次
        context_->data_base.plan_count = 0;
        context_->data_base.index_of_preset_point++;
        if (context_->data_base.index_of_preset_point >= context_->data_base.preset_points.size())
        {
          // 所有点试过了，都不行，静止
          context_->data_base.plan_count = 0;
          context_->data_base.index_of_preset_point = 0;
          context_->SwitchTo("Idle");
          return;
        }
        if (context_->data_base.goal_type == GoalType::Ultimate)
        {
          // 在当前预设点到不了站点，就先去别的预设点试一下
          context_->data_base.goal_type = GoalType::Preset;
        }
        context_->SwitchTo("ModifyGoalPose");
        return;
      }
      context_->SwitchTo("ModifyGoalPose");
      return;
    }

    if (context_->data_base.plan_result_ptr->data[1] == 2)
    {
      // 完全堵死，直接换另一个点试
      context_->data_base.plan_result_ptr = nullptr;
      context_->data_base.plan_count = 0;
      context_->data_base.index_of_preset_point++;
      if (context_->data_base.index_of_preset_point >= (context_->data_base.preset_points.size()))
      {
        // 所有点试过了，都不行，静止
        context_->data_base.plan_count = 0;
        context_->data_base.index_of_preset_point = 0;
        context_->SwitchTo("Idle");
        return;
      }
      if (context_->data_base.goal_type == GoalType::Ultimate)
      {
        // 在当前预设点到不了站点，就先去别的预设点试一下
        context_->data_base.goal_type = GoalType::Preset;
      }
      context_->SwitchTo("ModifyGoalPose");
      return;
    }
  }
}

void StartUp::Handle()
{
  // TODO：等待收到轨迹后，再启动
  bool has_received_trajectory = true;
  // 设置车速，当车速设置生效后，再启动
  bool has_worked_for_SetVelocityLimit = context_->data_base.goal_type == GoalType::Preset ?
                                             context_->SetVelocityLimit(1.0) :
                                             context_->SetVelocityLimit(0.3);
  if (has_received_trajectory && has_worked_for_SetVelocityLimit)
  {
    context_->engage(true);
    context_->SwitchTo("WaitForArrived");
  }
}

bool WaitForArrived::IsCloseTo(const Pose& destination)
{
  return Calculate2dDistance(context_->data_base.current_pose, destination) < 1.0;
}

bool WaitForArrived::HasStopped(const double stop_duration)
{
  static VehicleStopChecker vehicle_stop_checker(context_);
  return vehicle_stop_checker.isVehicleStopped(stop_duration);
}

bool WaitForArrived::HasMetError()
{
  const Pose& goal_pose = context_->data_base.goal_pose;
  const Pose& vehicle_pose = context_->data_base.current_pose;

  // calculate yaw error
  const double yaw4vehicle = GetYawFromPose(vehicle_pose);
  const double yaw4goal = GetYawFromPose(goal_pose);
  const double yaw_error = Rad2Deg(AngDiff(yaw4vehicle, yaw4goal));

  // calculate longitudinal error
  const double dx = vehicle_pose.position.x - goal_pose.position.x;
  const double dy = vehicle_pose.position.y - goal_pose.position.y;
  const double theta = atan2(dy, dx);
  const double longitudinal_error = sqrt(pow(dx, 2) + pow(dy, 2)) * cos(AngDiff(theta, yaw4goal));
  // calculate lateral error
  const double lateral_error = sqrt(pow(dx, 2) + pow(dy, 2)) * sin(AngDiff(theta, yaw4goal));

  RCLCPP_INFO(context_->get_logger(), "longitudinal_error is %f, lateral_error is %f", longitudinal_error,
              lateral_error);
  if (fabs(longitudinal_error) <= 1.0 && fabs(yaw_error) <= 10 && fabs(lateral_error) <= 0.05)
  {
    RCLCPP_INFO(context_->get_logger(), "has met the error requirement return true");
    return true;
  }
  else
  {
    RCLCPP_INFO(context_->get_logger(), "hasn't met the error requirement return false");
    return false;
  }
}

void WaitForArrived::Handle()
{
  if (IsCloseTo(context_->data_base.goal_pose) && HasStopped(3.0))
  {
    if (context_->data_base.goal_type == GoalType::Ultimate)
    {
      if (HasMetError())
      {
        std_msgs::msg::Bool msg;
        msg.data = true;
        context_->parking_state_pub_->publish(msg);
        context_->SwitchTo("Idle");
      }
      else
      {
        context_->data_base.goal_type = GoalType::Preset;
        context_->data_base.index_of_preset_point = 0;
        context_->SwitchTo("ModifyGoalPose");
      }
      return;
    }
    if (context_->data_base.goal_type == GoalType::Preset)
    {
      context_->data_base.goal_type = GoalType::Ultimate;
      context_->SwitchTo("ModifyGoalPose");
      return;
    }
  }
}

FreeSpacePlannerPreprocessorNode::FreeSpacePlannerPreprocessorNode(const rclcpp::NodeOptions& node_options)
  : Node("freespace_planner_preprocessor", node_options)
{
  // 加载预设点和站点数据
  {
    if (!LoadStationInfo())
    {
      RCLCPP_ERROR(get_logger(), "Load pose info failed, please check the config file");
      return;
    }
  }
  // 设置初始状态
  {
    SwitchTo("MessageForward");
  }
  // 订阅
  {
    mission_sub_ = create_subscription<Mission>(
        "~/input/mission", rclcpp::QoS{ 1 }.transient_local(),
        std::bind(&FreeSpacePlannerPreprocessorNode::OnMission, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<Odometry>(
        "~/input/odometry", rclcpp::QoS{ 1 }.durability_volatile(),
        std::bind(&FreeSpacePlannerPreprocessorNode::OnOdometry, this, std::placeholders::_1));
  }
  // 发布
  {
    route_pub_ = create_publisher<LaneletRoute>("~/output/route", rclcpp::QoS{ 1 }.transient_local());
    parking_state_pub_ = create_publisher<Bool>("~/output/parking_state", rclcpp::QoS{ 1 }.transient_local());
  }
  // 定时器
  {
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(10).period(),
                                  std::bind(&FreeSpacePlannerPreprocessorNode::OnTimer, this));
  }
}

std::shared_ptr<State> FreeSpacePlannerPreprocessorNode::CreateState(const std::string name)
{
  std::shared_ptr<State> new_state;
  if (name == "MessageForward")
  {
    new_state = std::make_shared<MessageForward>(this);
  }
  else if (name == "Idle")
  {
    new_state = std::make_shared<Idle>(this);
  }
  else if (name == "ModifyGoalPose")
  {
    new_state = std::make_shared<ModifyGoalPose>(this);
  }
  else if (name == "WaitPlanResult")
  {
    new_state = std::make_shared<WaitPlanResult>(this);
  }
  else if (name == "StartUp")
  {
    new_state = std::make_shared<StartUp>(this);
  }
  else if (name == "WaitForArrived")
  {
    new_state = std::make_shared<WaitForArrived>(this);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "state name is not correct");
    new_state = std::make_shared<Idle>(this);
  }
  data_base.state_map.emplace(name, new_state);
  return new_state;
}

void FreeSpacePlannerPreprocessorNode::SwitchTo(const std::string name)
{
  if (data_base.state && data_base.state->name() == name)
  {
    return;
  }
  data_base.prev_state = data_base.state;
  auto it = data_base.state_map.find(name);
  data_base.state = (it != data_base.state_map.end()) ? it->second : CreateState(name);
  RCLCPP_INFO(get_logger(), "switch state: %s -> %s",
              data_base.prev_state ? data_base.prev_state->name().c_str() : "None", data_base.state->name().c_str());
}

void FreeSpacePlannerPreprocessorNode::SwitchState()
{
  if (data_base.mission_ptr->free_space_sweeping_mode != Mission::RELOADING &&
      data_base.mission_ptr->free_space_sweeping_mode != Mission::DUMPING_TRASH)
  {
    SwitchTo("MessageForward");
    return;
  }

  // TODO：创建临时发布者,发布话题给激活scenario_selector
  auto tmp_pub = create_publisher<LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{ 1 }.transient_local());

  if (data_base.mission_ptr->free_space_sweeping_mode == Mission::RELOADING)
  {
    // TODO: 激活scenario_selector的route_，让它进入freespace的场景
    {
      LaneletRoute route;
      route.goal_pose = data_base.supply_station;
      tmp_pub->publish(route);
    }
    Reset();
    data_base.station = data_base.supply_station;
    data_base.preset_points = data_base.preset_points_of_supply_station;
    data_base.goal_type = GoalType::Preset;
    data_base.index_of_preset_point = 0;
    SwitchTo("ModifyGoalPose");
    return;
  }

  if (data_base.mission_ptr->free_space_sweeping_mode == Mission::DUMPING_TRASH)
  {
    {
      // TODO: 激活scenario_selector的route_，让它进入freespace的场景
      LaneletRoute route;
      route.goal_pose = data_base.garbage_dump;
      tmp_pub->publish(route);
    }
    Reset();
    data_base.station = data_base.garbage_dump;
    data_base.preset_points = data_base.preset_points_of_garbage_dump;
    data_base.goal_type = GoalType::Preset;
    data_base.index_of_preset_point = 0;
    SwitchTo("ModifyGoalPose");
    return;
  }
}

bool FreeSpacePlannerPreprocessorNode::engage(bool button)
{
  static auto engage_pub = create_publisher<Engage>("/autoware/engage", rclcpp::QoS{ 1 }.transient_local());
  Engage msg;
  msg.engage = button;
  if (button == false)
  {
    engage_pub->publish(msg);
    return true;
  }
  else
  {
    engage_pub->publish(msg);
    return true;
  }
}

bool FreeSpacePlannerPreprocessorNode::SetVelocityLimit(double max_velocity)
{
  auto OnVelocityLimit = [this](const VelocityLimit::ConstSharedPtr msg) {
    this->data_base.current_max_velocity = msg->max_velocity;
  };
  static auto velocity_limit_sub = create_subscription<VelocityLimit>(
      "/planning/scenario_planning/current_max_velocity", rclcpp::QoS{ 1 }.transient_local(), OnVelocityLimit);
  static auto velocity_limit_pub = create_publisher<VelocityLimit>("/planning/scenario_planning/max_velocity",
                                                                   rclcpp::QoS{ 1 }.durability_volatile());
  if (fabs(data_base.current_max_velocity - max_velocity) < 1e-3)
  {
    return true;
  }
  static auto last_time = now();
  if ((now() - last_time).seconds() > 0.1)
  {
    VelocityLimit vel_msg;
    vel_msg.max_velocity = max_velocity;
    velocity_limit_pub->publish(vel_msg);
    last_time = now();
  }
  return false;
}

bool FreeSpacePlannerPreprocessorNode::LoadStationInfo()
{
  auto ss = declare_parameter<std::vector<double>>("supply_station");
  if (ss.size() % 7 != 0)
  {
    RCLCPP_ERROR(get_logger(), "the size of element for variable supply_station size is not correct");
    return false;
  }
  data_base.supply_station.position.x = ss[0];
  data_base.supply_station.position.y = ss[1];
  data_base.supply_station.position.z = ss[2];
  data_base.supply_station.orientation.x = ss[3];
  data_base.supply_station.orientation.y = ss[4];
  data_base.supply_station.orientation.z = ss[5];
  data_base.supply_station.orientation.w = ss[6];

  auto gd = declare_parameter<std::vector<double>>("garbage_dump");
  if (gd.size() % 7 != 0)
  {
    RCLCPP_ERROR(get_logger(), "the size of element for variable garbage_dump  is not correct");
    return false;
  }
  data_base.garbage_dump.position.x = gd[0];
  data_base.garbage_dump.position.y = gd[1];
  data_base.garbage_dump.position.z = gd[2];
  data_base.garbage_dump.orientation.x = gd[3];
  data_base.garbage_dump.orientation.y = gd[4];
  data_base.garbage_dump.orientation.z = gd[5];
  data_base.garbage_dump.orientation.w = gd[6];

  data_base.preset_points_of_garbage_dump.clear();
  data_base.preset_points_of_supply_station.clear();
  auto pps = declare_parameter<std::vector<double>>("preset_points");
  if (pps.size() % 3 != 0)
  {
    RCLCPP_ERROR(get_logger(), "the size of element for variable preset_points is not correct");
    return false;
  }
  for (size_t i = 0; i < pps.size(); i += 3)
  {
    Pose pp;
    pp.position.x = pps[i];
    pp.position.y = pps[i + 1];
    pp.position.z = 0.0;
    pp.orientation = createQuaternionFromYaw(pps[i + 2]);
    data_base.preset_points_of_garbage_dump.push_back(CalculateGlobalPose(data_base.garbage_dump, pp));
    data_base.preset_points_of_supply_station.push_back(CalculateGlobalPose(data_base.supply_station, pp));
  }
  return true;
}

void FreeSpacePlannerPreprocessorNode::Reset()
{
  std_msgs::msg::Bool msg;
  msg.data = false;
  parking_state_pub_->publish(msg);
  engage(false);
}

void FreeSpacePlannerPreprocessorNode::OnTimer()
{
  if (data_base.has_new_mission)
  {
    SwitchState();
    data_base.has_new_mission = false;
  }
  data_base.state->Handle();
}

void FreeSpacePlannerPreprocessorNode::OnMission(const Mission::ConstSharedPtr msg)
{
  if (data_base.mission_ptr && data_base.mission_ptr->free_space_sweeping_mode == msg->free_space_sweeping_mode)
  {
    return;
  }
  data_base.mission_ptr = msg;
  data_base.has_new_mission = true;
}

void FreeSpacePlannerPreprocessorNode::OnOdometry(const Odometry::ConstSharedPtr msg)
{
  data_base.current_pose = msg->pose.pose;
}
}  // namespace freespace_planner_preprocessor
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(freespace_planner_preprocessor::FreeSpacePlannerPreprocessorNode)
