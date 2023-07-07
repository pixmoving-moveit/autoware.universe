#include "coverage_planning/coverage_planning.hpp"
#include <memory>

namespace coverage_planning  // namespace coverage_planning
{

bool CoveragePlanning::ptInPolygon(
  const Point<double> & p, const std::vector<Point<double>> & ptPolygon)
{
  int nCount = ptPolygon.size();
  //  number of cross point
  int nCross = 0;
  for (int i = 0; i < nCount; i++) {
    Point<double> p1 = ptPolygon[i];
    Point<double> p2 = ptPolygon[(i + 1) % nCount];

    if (p1._y == p2._y) continue;
    if (p._y < std::min(p1._y, p2._y)) continue;
    if (p._y >= std::max(p1._y, p2._y)) continue;
    double x = (p._y - p1._y) * (p2._x - p1._x) / (p2._y - p1._y) + p1._x;
    if (x > p._x) {
      nCross++;
    }
  }
  if ((nCross % 2) == 1)
    return true;
  else
    return false;
}

void CoveragePlanning::polygon2Vector(
  std::vector<Point<double>> & polygon_vector, lanelet::ConstPolygon3d polygon)
{
  polygon_vector.clear();
  for (const auto & point : polygon) {
    polygon_vector.push_back(Point<double>(point.x(), point.y(), 0));
  }
}

void CoveragePlanning::polygons2Vectors(
  std::vector<std::vector<Point<double>>> & polygon_vectors, lanelet::ConstPolygons3d polygons)
{
  polygon_vectors.clear();
  int i = 0;
  for (const auto & polygon : polygons) {
    std::vector<Point<double>> polygon_vector;
    polygon2Vector(polygon_vector, polygon);
    polygon_vectors.push_back(polygon_vector);
    i++;
  }
}

void CoveragePlanning::linestring2Vector(
  std::vector<Point<double>> & linestring_vector, lanelet::ConstLineString3d linestring)
{
  linestring_vector.clear();
  for (const auto & point : linestring) {
    linestring_vector.push_back(Point<double>(point.x(), point.y(), 0));
  }
}

void CoveragePlanning::linestrings2Vectors(
  std::vector<std::vector<Point<double>>> & linestring_vectors,
  lanelet::ConstLineStrings3d linestrings)
{
  linestring_vectors.clear();
  int i = 0;
  for (const auto & linestring : linestrings) {
    std::vector<Point<double>> linestring_vector;
    linestring2Vector(linestring_vector, linestring);
    linestring_vectors.push_back(linestring_vector);
    i++;
  }
}

std::vector<int> CoveragePlanning::sortIndex(std::vector<int> index_vector)
{
  // algorithm
  std::sort(index_vector.begin(), index_vector.end());
  return index_vector;
}

Trajectory CoveragePlanning::linestringToTrajectoryMsg(lanelet::ConstLineString3d & linestring)
{
  Trajectory traj;
  for (const auto & point : linestring) {
    TrajectoryPoint traj_point;
    traj_point.pose.position.x = point.x();
    traj_point.pose.position.y = point.y();
    traj.points.push_back(traj_point);
  }
  return traj;
}

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
    sub_map_ = create_subscription<HADMapBin>(
      "~/input/lanelet2_map", 1,
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
      node_param_.get_coverage_trajectory_service, std::bind(&CoveragePlanning::planTraj, this, std::placeholders::_1, std::placeholders::_2));
  }
}

bool CoveragePlanning::planTraj(
  const GetTrajectory::Request::SharedPtr req, GetTrajectory::Response::SharedPtr res)
{
  // bool poseInPolygon;
  lanelet::ConstPolygons3d parking_lots =
    lanelet::utils::query::getAllParkingLots(global_lanelet_map_ptr_);
  lanelet::ConstLineStrings3d cover_refs =
    lanelet::utils::query::getAllCoverRefs(global_lanelet_map_ptr_);
  // RCLCPP_WARN(
  //   get_logger(), "There are %d parking lots.\n There are %d coverage reference paths.",
  //   parking_lots.size(), cover_refs.size());
  Point<double> position(
    req->goal_pose.pose.position.x, req->goal_pose.pose.position.y, req->goal_pose.pose.position.z);
  std::vector<int> cover_refs_in_parking_lot_index;
  int cover_refs_index = 0;
  std::vector<std::vector<Point<double>>> parking_lots_polygon_vectors;
  std::vector<Point<double>> parking_lot_polygon_vector;
  coverage_trajectory_.points.clear();
  partial_traj_index_.data.clear();

  if (cover_refs.empty()) {
    // RCLCPP_WARN(get_logger(), "Coverage Planning Reference Trajectory is empty!");
    return false;
  }

  polygons2Vectors(parking_lots_polygon_vectors, parking_lots);
  int parking_lot_index = 0;
  for (const auto & parking_lot : parking_lots) {
    polygon2Vector(parking_lot_polygon_vector, parking_lot);
    if (ptInPolygon(position, parking_lot_polygon_vector)) {
      break;
    }
    parking_lot_index++;
  }

  for (size_t i = 0; i < cover_refs.size(); i++) {
    lanelet::ConstLineString3d cover_ref = cover_refs.at(i);
    Point<double> first_point_in_cover_ref(
      cover_ref.basicLineString().at(0).x(), cover_ref.basicLineString().at(0).y(),
      cover_ref.basicLineString().at(0).z());
    // if(PtInPolygon(first_point_in_cover_ref, parking_lot_polygon_vector))
    // {
    //     cover_refs_in_parking_lot_index.push_back(cover_refs.at(i).id());
    // }
    cover_refs_in_parking_lot_index.push_back(cover_refs.at(i).id());
  }
  std::vector<int> cover_index_sorted = sortIndex(cover_refs_in_parking_lot_index);

  std::vector<lanelet::ConstLineString3d> sorted_cover_ref_in_parking_lot;
  for (size_t i = 0; i < cover_index_sorted.size(); i++) {
    for (size_t j = 0; j < cover_index_sorted.size(); j++) {
      if (cover_refs.at(j).id() == cover_index_sorted.at(i)) {
        sorted_cover_ref_in_parking_lot.push_back(cover_refs.at(j));
        // RCLCPP_INFO(get_logger(), "i: %d, j: %d", i, j);
      }
    }
  }

  for (size_t i = 0; i < cover_index_sorted.size(); i++) {
    // int partial_traj_index = cover_index_sorted.at(i);
    Trajectory temp_t =
      linestringToTrajectoryMsg(sorted_cover_ref_in_parking_lot.at(i));
    for (size_t j = 0; j < temp_t.points.size(); j++) {
      coverage_trajectory_.points.push_back(temp_t.points.at(j));
    }
    cover_refs_index += sorted_cover_ref_in_parking_lot.at(i).size();
    partial_traj_index_.data.push_back(cover_refs_index);
    // RCLCPP_INFO(
    //   get_logger(), "trajectory %d has %d points.\n", cover_index_sorted.at(i),
    //   sorted_cover_ref_in_parking_lot.at(i).size());
  }
  res->trajectory = coverage_trajectory_;
  res->partial_index = partial_traj_index_;
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
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr);
  global_lanelet_map_ptr_ = lanelet_map_ptr;
  is_map_loaded_ = true;
}
}  // namespace coverage_planning
