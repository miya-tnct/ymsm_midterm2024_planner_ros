#ifndef YMSM_MIDTERM2024_PLANNER_PLANNER_NODE_H_
#define YMSM_MIDTERM2024_PLANNER_PLANNER_NODE_H_

#include <cstddef>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ymsm_midterm2024_planner::planner
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  void initialize_pole_left(geometry_msgs::PointStamped::ConstPtr pole_left_msg);

  void initialize_pole_right(geometry_msgs::PointStamped::ConstPtr pole_right_msg);

  void update_pole_left(geometry_msgs::PointStamped::ConstPtr pole_left_msg);

  void update_pole_right(geometry_msgs::PointStamped::ConstPtr pole_right_msg);

  void plan();

  nav_msgs::Path path_msg_;
  geometry_msgs::PoseStamped goal_pose_;
  
  std::size_t relay_index_;

  geometry_msgs::PointStamped::ConstPtr pole_left_msg_, pole_right_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher path_publisher_;
  ros::Subscriber pole_left_subscriber_, pole_right_subscriber_;
};

}

#endif