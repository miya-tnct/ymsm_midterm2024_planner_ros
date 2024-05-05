#include "ymsm_midterm2024_planner/planner/node.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <limits>
#include <numeric>

namespace ymsm_midterm2024_planner::planner
{

Node::Node() :
  ros::NodeHandle(),
  path_msg_([]{
    decltype(path_msg_) path;
    path.header.frame_id = "map";
    return path;
  }()),
  goal_pose_([]{
    decltype(goal_pose_) pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 5.0;
    return pose;
  }()),
  relay_index_(0),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  path_publisher_(this->advertise<nav_msgs::Path>("path", 1, true)),
  pole_left_subscriber_(this->subscribe("pole_left", 1, &Node::initialize_pole_left, this)),
  pole_right_subscriber_(this->subscribe("pole_right", 1, &Node::initialize_pole_right, this))
{
}

void Node::initialize_pole_left(geometry_msgs::PointStamped::ConstPtr pole_left_msg)
{
  pole_left_msg_ = std::move(pole_left_msg);
  if (pole_right_msg_) {
    pole_left_subscriber_ = this->subscribe("pole_left", 1, &Node::update_pole_left, this);
    pole_right_subscriber_ = this->subscribe("pole_right", 1, &Node::update_pole_right, this);
    this->plan();
  }
}

void Node::initialize_pole_right(geometry_msgs::PointStamped::ConstPtr pole_right_msg)
{
  pole_right_msg_ = std::move(pole_right_msg);
  if (pole_left_msg_) {
    pole_left_subscriber_ = this->subscribe("pole_left", 1, &Node::update_pole_left, this);
    pole_right_subscriber_ = this->subscribe("pole_right", 1, &Node::update_pole_right, this);
    this->plan();
  }
}

void Node::update_pole_left(geometry_msgs::PointStamped::ConstPtr pole_left_msg)
{
  pole_left_msg_ = std::move(pole_left_msg);
  this->plan();
}

void Node::update_pole_right(geometry_msgs::PointStamped::ConstPtr pole_right_msg)
{
  pole_right_msg_ = std::move(pole_right_msg);
  this->plan();
}

void Node::plan()
{
  if (pole_left_msg_->header.frame_id != pole_right_msg_->header.frame_id) {
    return;
  }

  geometry_msgs::TransformStamped base_link_tf_msg;
  try {
    base_link_tf_msg = tf_buffer_.lookupTransform(
      "map", "base_link", ros::Time(0));
  }
  catch(...) {
    return;
  }

  tf2::Transform base_link_tf;
  tf2::fromMsg(base_link_tf_msg.transform, base_link_tf);

  tf2::Vector3 pole_right_vec, pole_left_vec;
  tf2::fromMsg(pole_left_msg_->point, pole_left_vec);
  tf2::fromMsg(pole_right_msg_->point, pole_right_vec);

  auto pole_mid_vec = (pole_left_vec + pole_right_vec) * 0.5;
  auto pole_diff_normal_vec = (pole_left_vec - pole_right_vec).normalized();
  auto pole_vertical_vec = tf2::Transform(tf2::Quaternion(0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4))) * pole_diff_normal_vec;

  auto relay_front_vec = pole_mid_vec + pole_vertical_vec;
  auto relay_back_vec = pole_mid_vec - pole_vertical_vec;

  if (relay_index_ == 0 && base_link_tf.getOrigin().distance2(relay_front_vec) < 1.0 * 1.0) {
    relay_index_ = 1;
  }

  if (relay_index_ == 1 && base_link_tf.getOrigin().distance2(relay_back_vec) < 1.0 * 1.0) {
    relay_index_ = 2;
  }


  path_msg_.header.stamp = ros::Time::now();

  auto pose_from_tf = [](auto tf) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = tf.x();
    pose.pose.position.y = tf.y();
    pose.pose.position.z = tf.z();
    return pose;
  };
  path_msg_.poses.clear();
  if (relay_index_ < 1) {
    path_msg_.poses.push_back(pose_from_tf(relay_front_vec));
  }
  if (relay_index_ < 2) {
    path_msg_.poses.push_back(pose_from_tf(relay_back_vec));
  }
  path_msg_.poses.push_back(goal_pose_);

  path_publisher_.publish(path_msg_);
}

}