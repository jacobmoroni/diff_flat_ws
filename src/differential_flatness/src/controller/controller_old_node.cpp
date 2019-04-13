#include <ros/ros.h>
#include "controller_old.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;

  controller_old::ControllerOld Thing;

  ros::spin();

  return 0;
}
