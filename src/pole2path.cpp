#include "ymsm_pole2path/pole2path/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "pole2path");
  ymsm_pole2path::pole2path::Node node;
  ros::spin();
  return 0;
}