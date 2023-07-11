#include "coverage_planning/coverage_planning_core.hpp"

namespace coverage_planning
{

CoveragePlanningCore::CoveragePlanningCore()
{
  std::cout << "Coverage Planning Created." << std::endl;
}

void CoveragePlanningCore::setMap(HADMapBin::ConstSharedPtr map_bin)
{
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_bin, lanelet_map_ptr);
  global_lanelet_map_ptr_ = lanelet_map_ptr;
  std::cout << "Map Updated\n";
}

bool CoveragePlanningCore::ptInPolygon(
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

void CoveragePlanningCore::polygon2Vector(
  std::vector<Point<double>> & polygon_vector, lanelet::ConstPolygon3d polygon)
{
  polygon_vector.clear();
  for (const auto & point : polygon) {
    polygon_vector.push_back(Point<double>(point.x(), point.y(), 0));
  }
}

void CoveragePlanningCore::polygons2Vectors(
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

void CoveragePlanningCore::linestring2Vector(
  std::vector<Point<double>> & linestring_vector, lanelet::ConstLineString3d linestring)
{
  linestring_vector.clear();
  for (const auto & point : linestring) {
    linestring_vector.push_back(Point<double>(point.x(), point.y(), 0));
  }
}

void CoveragePlanningCore::linestrings2Vectors(
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

std::vector<int> CoveragePlanningCore::sortIndex(std::vector<int> index_vector)
{
  // algorithm
  std::sort(index_vector.begin(), index_vector.end());
  return index_vector;
}

Trajectory CoveragePlanningCore::linestringToTrajectoryMsg(lanelet::ConstLineString3d & linestring)
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

std::tuple<Trajectory, Int16MultiArray> CoveragePlanningCore::planTraj(const PoseStamped & goal_pose)
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
    goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
  std::vector<int> cover_refs_in_parking_lot_index;
  int cover_refs_index = 0;
  std::vector<std::vector<Point<double>>> parking_lots_polygon_vectors;
  std::vector<Point<double>> parking_lot_polygon_vector;
  Trajectory coverage_trajectory;
  Int16MultiArray partial_traj_index;

  if (cover_refs.empty()) {
    std::cout << "could not find coverage path" << std::endl;
    return tie(coverage_trajectory, partial_traj_index);
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
      }
    }
  }

  for (size_t i = 0; i < cover_index_sorted.size(); i++) {
    // int partial_traj_index = cover_index_sorted.at(i);
    Trajectory temp_t =
      linestringToTrajectoryMsg(sorted_cover_ref_in_parking_lot.at(i));
    for (size_t j = 0; j < temp_t.points.size(); j++) {
      coverage_trajectory.points.push_back(temp_t.points.at(j));
    }
    cover_refs_index += sorted_cover_ref_in_parking_lot.at(i).size();
    partial_traj_index.data.push_back(cover_refs_index);
  }
  return tie(coverage_trajectory, partial_traj_index);
}

}