#include "ymsm_midterm2024_planner/pole2path/node.h"

#include "nav_msgs/Path.h"

#include <limits>
#include <numeric>

namespace ymsm_midterm2024_planner::pole2path
{

Node::Node() :
  ros::NodeHandle(),
  point_min_({0, -2}),
  point_max_({5,  2}),
  scan_subscriber_(this->subscribe("scan", 1, &Node::convert_once, this)),
  path_publisher_(this->advertise<nav_msgs::Path>("path", 1, true))
{
}

void Node::convert(
  const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  std::vector<Point> pole_points;
  auto angle = scan_msg->angle_min;
  for (const auto & range : scan_msg->ranges)
  {
    const Point point = {
      range * std::cos(angle),
      range * std::sin(angle)
    };
    if (range >= range_min && point_min_.x < point.x && point.x < point_max_.x &&
      point_min_.y < point.y && point.y < point_max_.y) {
      pole_points.emplace_back(point);
    }
    angle += scan_msg->angle_increment;
  }

  if (pole_points.empty()) {
    return;
  }
  
  Point pole_points_sum = {};
  for (const auto & pole_point : pole_points) {
    pole_points_sum.x += pole_point.x;
    pole_points_sum.y += pole_point.y;
    // ROS_INFO("%lf, %lf", pole_point.x, pole_point.y);
  }

  Point pole_points_avg = {
    pole_points_sum.x / pole_points.size(),
    pole_points_sum.y / pole_points.size()
  };

  nav_msgs::Path path;
  path.header = scan_msg->header;
  path.poses.resize(2);
  path.poses[0].pose.position.x = pole_points_avg.x;
  path.poses[0].pose.position.y = pole_points_avg.y;
  path.poses[1].pose.position.x = 5;
  path.poses[1].pose.position.y = 0;

  path_publisher_.publish(path);
}

void Node::convert_once(
  const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  this->convert(scan_msg);
  ROS_INFO("Published");
  scan_subscriber_.shutdown();
}

}