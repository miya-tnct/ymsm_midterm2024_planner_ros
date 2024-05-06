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
  offset_(0.3),
  offset2_(offset_ * offset_),
  path_msg_([]{
    decltype(path_msg_) path;
    path.header.frame_id = "map";
    return path;
  }()),
  relays_({
    Vec3WithFrameId({}), 
    Vec3WithFrameId({}), 
    Vec3WithFrameId({"map", {5, 0,0}})
  }),
  relays_itr_(relays_.begin()),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  path_publisher_(this->advertise<nav_msgs::Path>("path", 1, true)),
  initialpose_subscriber_(this->subscribe("initialpose", 1, &Node::initialize_pose, this)),
  pole_subscribers_({
    this->subscribe("pole_right", 1, &Node::initialize_pole<0>, this),
    this->subscribe("pole_left", 1, &Node::initialize_pole<1>, this)
  })
{
}

void Node::initialize_pose(geometry_msgs::PoseWithCovarianceStamped::ConstPtr)
{
  relays_itr_ = relays_.begin();
  pole_subscribers_ = {
    this->subscribe("pole_right", 1, &Node::initialize_pole<0>, this),
    this->subscribe("pole_left", 1, &Node::initialize_pole<1>, this)
  };
}

template <std::size_t POLE_INDEX>
void Node::initialize_pole(geometry_msgs::PointStamped::ConstPtr pole_msg)
{
  pole_msgs_[POLE_INDEX] = std::move(pole_msg);
  for (const auto & pole_msg : pole_msgs_) {
    if (!pole_msg) {
      return;
    }
  }

  pole_subscribers_ = {
    this->subscribe("pole_right", 1, &Node::update_pole<0>, this),
    this->subscribe("pole_left", 1, &Node::update_pole<1>, this)
  };
  this->plan();
}

template <std::size_t POLE_INDEX>
void Node::update_pole(geometry_msgs::PointStamped::ConstPtr pole_msg)
{
  pole_msgs_[POLE_INDEX] = std::move(pole_msg);
  this->plan();
}

void Node::plan()
{
  relays_[0].frame_id = relays_[1].frame_id = pole_msgs_[0]->header.frame_id;
  for (const auto & pole_msg : pole_msgs_) {
    if (pole_msg->header.frame_id != relays_[0].frame_id) {
      return;
    }
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
  tf2::fromMsg(pole_msgs_[0]->point, pole_right_vec);
  tf2::fromMsg(pole_msgs_[1]->point, pole_left_vec);

  auto pole_mid_vec = (pole_left_vec + pole_right_vec) * 0.5;
  auto pole_diff_normal_vec = (pole_left_vec - pole_right_vec).normalized();
  auto pole_vertical_vec = tf2::Transform(tf2::Quaternion(0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4))) * pole_diff_normal_vec;

  relays_[0].vec = pole_mid_vec + pole_vertical_vec;
  relays_[1].vec = pole_mid_vec - pole_vertical_vec;

  while (relays_itr_->vec.distance2(base_link_tf.getOrigin()) < offset2_) {
    ++relays_itr_;
    if (relays_itr_ == relays_.end()) {
      for (auto & pole_subscriber : pole_subscribers_) {
        pole_subscriber.shutdown();
      }
      return;
    }
  }

  auto point_msg_from_vec = [](auto vec) {
    geometry_msgs::Point point_msg;
    point_msg.x = vec.x();
    point_msg.y = vec.y();
    point_msg.z = vec.z();
    return point_msg;
  };

  path_msg_.header.stamp = ros::Time::now();
  path_msg_.poses.resize(1 + std::distance(relays_itr_, relays_.end()));
  path_msg_.poses[0].pose.position = point_msg_from_vec(base_link_tf.getOrigin() + (relays_itr_->vec - base_link_tf.getOrigin()).normalized() * offset_);
  
  auto path_poses_itr_ = std::next(path_msg_.poses.begin(), 1);
  for (auto itr = relays_itr_; itr != relays_.end(); ++itr) {
    path_poses_itr_->header.frame_id = itr->frame_id;
    path_poses_itr_->pose.position = point_msg_from_vec(itr->vec);
    ++path_poses_itr_;
  }

  path_publisher_.publish(path_msg_);
}

}