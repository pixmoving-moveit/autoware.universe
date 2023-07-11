/**
 * @file coverage_planning.hpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief header file of hand-drawing coverage planning in parking lot
 * @version 0.2
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023 by Pixmoving
 * 
 */
#pragma once

#include <vector>
#include <tuple>
#include <algorithm>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/msg/mission.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>


namespace coverage_planning // namespace coverage_planning
{
using autoware_auto_planning_msgs::msg::Mission;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using std_msgs::msg::Int16MultiArray;

/**
 * @brief struct of Point
 * 
 * @tparam T 
 */
template<class T>
struct Point
{
  T _x;
  T _y;
  T _z;
  Point(T x, T y, T z):_x(x), _y(y), _z(z){};
  Point operator +(const Point &p)
  {
    return Point(this->_x+p._x, this->_y+p._y, this->_z+p._z);
  }
  Point operator -(const Point &p)
  {
    return Point(this->_x-p._x, this->_y-p._y, this->_z-p._z);
  }
  T operator *(const Point &p)
  {
    return this->_x*p._x+this->_y*p._y+this->_z*p._z;
  }
  Point<T> operator* (const T &a)
  {
    return Point<T>(this->_x*a, this->_y*a, this->_z*a);
  }
  bool operator== (const Point &p)
  {
    if(this->_x==p._x&&this->_y==p._y&&this->_z==p._z)
    {
      return true;
    }else{
      return false;
    }
  }
  void printPoint()
  {
    std::cout << "x: " << this->_x << ", y: " << this->_y << ", z: " << this->_z << std::endl;
  }
};

class CoveragePlanningCore
{
private:
  PoseStamped current_pose_;
  PoseStamped goal_pose_;
  TwistStamped current_twist_;
  lanelet::LaneletMapPtr global_lanelet_map_ptr_;
public:
  CoveragePlanningCore(/* args */);
  std::tuple<Trajectory, Int16MultiArray> planTraj(const PoseStamped & goal_pose);
  void setMap(HADMapBin::ConstSharedPtr map_bin);
  void polygon2Vector(std::vector<Point<double>> & polygon_vector, lanelet::ConstPolygon3d polygon);
  void polygons2Vectors(std::vector<std::vector<Point<double>>> &polygon_vectors, lanelet::ConstPolygons3d polygons);
  void linestring2Vector(std::vector<Point<double>> &linestring_vector, lanelet::ConstLineString3d linestring);
  void linestrings2Vectors(std::vector<std::vector<Point<double>>> &linestring_vectors, lanelet::ConstLineStrings3d linestrings);

  std::vector<int> sortIndex(std::vector<int> index_vector);

  bool ptInPolygon(const Point<double> &p, const std::vector<Point<double>> &ptPolygon);

  Trajectory linestringToTrajectoryMsg(lanelet::ConstLineString3d & linestring);
};

} // namespace coverage_planning