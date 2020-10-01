#include <ros/ros.h>
#include <ros/package.h>

#include <radiation_utils/elist_parser.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "elist_parser_test");
  ros::NodeHandle nh = ros::NodeHandle("~");

  auto result = radiation_utils::

  return 0;
}
