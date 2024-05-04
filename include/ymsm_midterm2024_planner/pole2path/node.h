#ifndef YMSM_POLE2PATH_POLE2PATH_NODE_H_
#define YMSM_POLE2PATH_POLE2PATH_NODE_H_

#include <cstdint>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

namespace ymsm_midterm2024_planner::pole2path
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  void convert(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  void convert_once(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  struct Point
  {
    double x, y;
  } point_min_, point_max_;

  double range_min = 1.0;

  ros::Publisher path_publisher_;
  ros::Subscriber scan_subscriber_;
};

}

#endif