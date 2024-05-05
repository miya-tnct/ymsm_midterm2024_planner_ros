#include "ymsm_midterm2024_planner/planner/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "planner");
  ymsm_midterm2024_planner::planner::Node node;
  ros::spin();
  return 0;
}