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

#include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/msg/mission.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <autoware_auto_planning_msgs/srv/get_trajectory.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>


namespace coverage_planning // namespace coverage_planning
{
using autoware_auto_planning_msgs::msg::Mission;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_planning_msgs::srv::GetTrajectory;
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

struct NodeParam
{
  // input
  std::string sub_map_topic; // input lanelet2 map topic
  std::string sub_current_mission_topic; // input current mission topic
  std::string sub_goal_pose_topic; // input goal pose topic
  std::string sub_current_twist_topic; // input current twist topic
  // ouput
  std::string pub_traj_topic; // output trajectory topic
  std::string pub_partial_traj_index_topic; // output partial trajectory index topic
  std::string get_coverage_trajectory_service;

  double th_arrived_distance_m; // distance(in meters) between current pose and trajectory target point
  double th_stopped_velocity_mps; // maxmium velocity(in m/s) treshold to velidate a vehicle is stopped
};



class CoveragePlanning: public rclcpp::Node
{
private:
  // subscirbers
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Mission>::SharedPtr sub_current_mission_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_current_twist_;

  // publishers
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<Int16MultiArray>::SharedPtr pub_partial_index_;

  // services
  rclcpp::Service<GetTrajectory>::SharedPtr serv_get_coverage_traj_;

  // tf
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;

  // variables
  PoseStamped current_pose_;
  PoseStamped goal_pose_;
  TwistStamped current_twist_;

  Trajectory coverage_trajectory_;
  Int16MultiArray partial_traj_index_;
  Mission current_mission_;

  lanelet::LaneletMapPtr global_lanelet_map_ptr_;
  bool is_map_loaded_ = false;

  // functions, callback

  /**
   * @brief 
   * 
   * @param msg 
   */
  void onTwist(const TwistStamped::ConstSharedPtr & msg);
  /**
   * @brief callback function to know whether if the goal point is in the parking lot
   *
   * @param msg
   */
  void onCurrentMission(const Mission::ConstSharedPtr & msg);
  /**
   * @brief callback function of autoware_lanelet2_msgs::MapBinConstPtr, convert lanelet2 map bin
   * msg to lanelet2 object
   *
   * @param msg autoware_lanelet2_msgs::MapBinConstPtr object
   */
  void onLaneletMap(const HADMapBin::ConstSharedPtr msg);
  void onGoal(const PoseStamped::ConstSharedPtr & msg);

  bool planTraj(const GetTrajectory::Request::SharedPtr req, GetTrajectory::Response::SharedPtr res);

  void polygon2Vector(std::vector<Point<double>> &polygon_vector, lanelet::ConstPolygon3d polygon);
  void polygons2Vectors(std::vector<std::vector<Point<double>>> &polygon_vectors, lanelet::ConstPolygons3d polygons);
  void linestring2Vector(std::vector<Point<double>> &linestring_vector, lanelet::ConstLineString3d linestring);
  void linestrings2Vectors(std::vector<std::vector<Point<double>>> &linestring_vectors, lanelet::ConstLineStrings3d linestrings);

  std::vector<int> sortIndex(std::vector<int> index_vector);

  bool ptInPolygon(const Point<double> &p, const std::vector<Point<double>> &ptPolygon);

  Trajectory linestringToTrajectoryMsg(lanelet::ConstLineString3d & linestring);

public:
  explicit CoveragePlanning(const rclcpp::NodeOptions & node_options);
};


} // namespace coverage_planning