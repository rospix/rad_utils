#include <radiation_utils/gridmap.h>

#include <random>

double range_min = -5;
double range_max = 40;

std::mt19937                           generator(time(0));
std::uniform_real_distribution<double> rand_dbl(range_min, range_max);
std::uniform_real_distribution<double> rand_percent(0, 1);

int main(int argc, char** argv) {

  ros::init(argc, argv, "gridmap_test");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Publisher map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("map_out", 1);

  ROS_INFO("[%s]: Test started!", ros::this_node::getName().c_str());

  /* GRID CREATION //{ */
  ROS_INFO("[%s]: Generating gridmaps of random size..", ros::this_node::getName().c_str());


  for (int i = 0; (i < 70 && ros::ok()); i++) {

    double x = rand_dbl(generator);
    double y = rand_dbl(generator);
    try {
      mapping::GridMap gm(x, y, 1, Eigen::Vector2d(0, 0), 0, "map");
      map_publisher.publish(gm.buildOccupancyGrid());
    }
    catch (std::invalid_argument ex) {
      ROS_WARN("[%s]: Invalid grid generation attempted! What's wrong: %s", ros::this_node::getName().c_str(), ex.what());
    }

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* GRID PIVOTING //{ */
  ROS_INFO("[%s]: Generating gridmaps with random origin", ros::this_node::getName().c_str());


  for (int i = 0; (i < 40 && ros::ok()); i++) {

    double           x = rand_dbl(generator);
    double           y = rand_dbl(generator);
    double           z = rand_dbl(generator);
    mapping::GridMap gm(10, 17, 1, Eigen::Vector2d(x, y), z, "map");

    map_publisher.publish(gm.buildOccupancyGrid());
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RESOLUTION CHANGE //{ */
  ROS_INFO("[%s]: Generating gridmaps with changing resolution", ros::this_node::getName().c_str());

  for (int i = 0; (i < 30 && ros::ok()); i++) {

    double           r = rand_percent(generator);
    mapping::GridMap gm(20, 20, r, Eigen::Vector2d(0, 0), 0, "map");
    for (size_t j = 0; j < gm.getNumCells(); j++) {
      double v = rand_percent(generator);
      gm.setCellValue(v, j);
    }

    map_publisher.publish(gm.buildOccupancyGrid());
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* SPECIFIC DATA PLACEMENT //{ */
  ROS_INFO("[%s]: Placing data into specific cells", ros::this_node::getName().c_str());

  mapping::GridMap gm(30, 20, 0.5, Eigen::Vector2d(0, 0), 0, "map");

  for (int r = 0; (r < gm.getRows() && ros::ok()); r++) {
    for (int c = 0; (c < gm.getColumns() && ros::ok()); c++) {

      double val = rand_dbl(generator) + (0.3 * r * c);
      gm.setCellValue(val, r, c);

      map_publisher.publish(gm.buildOccupancyGrid());
      ros::spinOnce();
      ros::Duration(0.002).sleep();
    }
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  ROS_INFO("[%s]: TEST DONE!", ros::this_node::getName().c_str());
  return 0;
}
