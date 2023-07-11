#include "coverage_planning/coverage_planning.hpp"
#include <memory>

namespace coverage_planning  // namespace coverage_planning
{
// CoveragePlanning::CoveragePlanning():nh_("~"), tf_listener_(tf_buffer_)
CoveragePlanning::CoveragePlanning(const rclcpp::NodeOptions & node_options) : Node("coverage_planning", node_options)
{
  // NodeParam
  {
    node_param_.sub_map_topic = declare_parameter<std::string>("sub_map_topic", "~/input/lanelet2_map");
    node_param_.sub_current_mission_topic = declare_parameter<std::string>("sub_current_mission_topic", "~/input/current_mission");
    node_param_.sub_goal_pose_topic = declare_parameter<std::string>("sub_goal_pose_topic", "~/input/goal");
    node_param_.sub_current_twist_topic = declare_parameter<std::string>("sub_current_twist_topic", "~/input/current_twist");
    node_param_.pub_traj_topic = declare_parameter<std::string>("pub_traj_topic", "~/output/coverage_trajectory");
    node_param_.pub_partial_traj_index_topic = declare_parameter<std::string>("pub_partial_traj_index_topic", "~/output/partial_traj_index");
    node_param_.get_coverage_trajectory_service = declare_parameter<std::string>("get_coverage_trajectory", "~/srv/get_coverage_trajectory");
    node_param_.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps", 0.01);
    node_param_.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m", 1.0);
  }

  // Subscribers
  {
    auto qos_transient_local = rclcpp::QoS{1}.transient_local();
    sub_map_ = create_subscription<HADMapBin>(
      "~/input/lanelet2_map", qos_transient_local,
      std::bind(&CoveragePlanning::onLaneletMap, this, std::placeholders::_1));
    sub_current_mission_ = create_subscription<Mission>(
      "~/input/current_mission", 1,
      std::bind(&CoveragePlanning::onCurrentMission, this, std::placeholders::_1));
    // sub_goal_pose_ = nh_.subscribe(node_param_.sub_goal_pose_topic, 1, &CoveragePlanning::onGoal,
    // this);
    sub_current_twist_ = create_subscription<TwistStamped>(
      "~/input/current_twist", 1,
      std::bind(&CoveragePlanning::onTwist, this, std::placeholders::_1));
  }

  // Publishers
  {
    pub_traj_ = create_publisher<Trajectory>(node_param_.pub_traj_topic, 1);
    pub_partial_index_ = create_publisher<Int16MultiArray>(
      node_param_.pub_partial_traj_index_topic, 1);
  }

  // Serivces
  {
    serv_get_coverage_traj_ = this->create_service<GetTrajectory>(
      // node_param_.get_coverage_trajectory_service, &CoveragePlanning::planTraj);
      "~/input/src/get_coverage_path",
      std::bind(&CoveragePlanning::planTraj, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default);
  }
}

bool CoveragePlanning::planTraj(
  GetTrajectory::Request::SharedPtr req, GetTrajectory::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Got Coverage Planning Request");
  coverage_planning_core_.setMap(map_bin_);
  std::tuple<Trajectory, Int16MultiArray> result =
    coverage_planning_core_.planTraj(req->goal_pose);

  res->trajectory = std::get<0>(result);
  res->partial_index = std::get<1>(result);
  pub_traj_->publish(coverage_trajectory_);
  pub_partial_index_->publish(partial_traj_index_);
  return true;
}

void CoveragePlanning::onCurrentMission(const Mission::ConstSharedPtr & msg)
{
  current_mission_ = *msg;
}

void CoveragePlanning::onTwist(const TwistStamped::ConstSharedPtr & msg)
{
  current_twist_ = *msg;
}

void CoveragePlanning::onLaneletMap(const HADMapBin::ConstSharedPtr msg)
{
  map_bin_ = msg;
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr);
  global_lanelet_map_ptr_ = lanelet_map_ptr;
  is_map_loaded_ = true;
  RCLCPP_INFO(get_logger(), "Map Loaded.");
}
}  // namespace coverage_planning
