#ifndef YMSM_MIDTERM2024_PLANNER_PLANNER_NODE_H_
#define YMSM_MIDTERM2024_PLANNER_PLANNER_NODE_H_

#include <cstddef>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ymsm_midterm2024_planner::planner
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  struct Vec3WithFrameId
  {
    std::string frame_id;
    tf2::Vector3 vec;
  };

  void initialize_pose(geometry_msgs::PoseWithCovarianceStamped::ConstPtr);
  
  template <std::size_t POLE_INDEX>
  void initialize_pole(geometry_msgs::PointStamped::ConstPtr pole_msg);

  template <std::size_t POLE_INDEX>
  void update_pole(geometry_msgs::PointStamped::ConstPtr pole_msg);

  void plan();

  double offset_, offset2_;
  std::array<Vec3WithFrameId, 3> relays_;
  decltype(relays_)::iterator relays_itr_;

  nav_msgs::Path path_msg_;

  std::array<geometry_msgs::PointStamped::ConstPtr, 2> pole_msgs_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher path_publisher_;
  ros::Subscriber initialpose_subscriber_;
  std::array<ros::Subscriber, 2> pole_subscribers_;
};

}

#endif