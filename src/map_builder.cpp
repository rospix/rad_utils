// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Trigger.h>
#include <algorithm>

#include <rad_msgs/ClusterList.h>

double width, height, resolution;
bool   outside_map = true;

// UAV position in world
double x   = 0;
double y   = 0;
double z   = 0;
double yaw = 0;

double map_origin_x = 0;
double map_origin_y = 0;

// UAV position in map grid
int gridx = 0;
int gridy = 0;

int                 rows;
int                 cols;
std::vector<double> data_raw;
std::vector<double> time_spent_at_cell;

std::string output_path;

// subscribers
ros::Subscriber data_subscriber;
ros::Subscriber odom_subscriber;

nav_msgs::OccupancyGrid map_raw;
nav_msgs::OccupancyGrid map_normalized;

ros::Publisher map_raw_publisher;
ros::Publisher map_normalized_publisher;

double     odom_dt              = 0.01;  // [s]
double     map_publish_interval = 0.1;   // [s]
ros::Timer map_publish_timer;

void getPositionInMap(double odom_x, double odom_y) {
  int    prev_x = gridx;
  int    prev_y = gridy;
  double fx     = (odom_x - map_origin_x) / resolution;
  double fy     = (odom_y - map_origin_y) / resolution;

  gridx = (int)(fx);
  gridy = (int)(fy);

  // avoid index out of array bounds
  outside_map = gridx < 0 || gridy < 0 || gridx > rows - 1 || gridy > cols - 1;


  if (prev_x == gridx && prev_y == gridy) {
    if (!outside_map) {
      time_spent_at_cell[prev_y * rows + prev_x] += odom_dt;
    }
  }
}

// is called every time new message comes in
void dataCallback(rad_msgs::ClusterList msg) {

  if (outside_map) {
    return;
  }

  int new_clusters = msg.clusters.size();
  data_raw[gridy * rows + gridx] += new_clusters;
}

// called every time new odom message comes in
void odomCallback(const nav_msgs::Odometry& odom_in) {
  /* ROS_INFO("[%s]: got odom!", ros::this_node::getName().c_str()); */
  x   = odom_in.pose.pose.position.x;
  y   = odom_in.pose.pose.position.y;
  z   = odom_in.pose.pose.position.z;
  yaw = odom_in.pose.pose.orientation.z;

  getPositionInMap(x, y);
}

void mapPublishTimer([[maybe_unused]] const ros::TimerEvent& evt) {

  std::vector<double> data_activity;

  for (unsigned int i = 0; i < data_raw.size(); i++) {
    if (time_spent_at_cell[i] < 0.01) {
      data_activity.push_back(-1);
    } else {
      data_activity.push_back(data_raw[i] / time_spent_at_cell[i]);
    }
  }

  double data_max = *std::max_element(data_activity.begin(), data_activity.end());

  for (unsigned int i = 0; i < data_activity.size(); i++) {
    if (data_activity[i] > 0) {
      map_normalized.data[i] = 255 * (data_activity[i] / data_max);
    }
  }

  map_normalized.header.seq = data_max;  // publish the normalization constant
  map_normalized_publisher.publish(map_normalized);
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "map_builder");
  ros::NodeHandle nh_ = ros::NodeHandle("~");
  nh_.getParam("height", height);
  nh_.getParam("width", width);
  nh_.getParam("resolution", resolution);
  nh_.getParam("origin/x", map_origin_x);
  nh_.getParam("origin/y", map_origin_y);

  // fill the heatmap with zero values
  map_normalized                        = nav_msgs::OccupancyGrid();
  map_normalized.info.height            = width / resolution;
  map_normalized.info.width             = height / resolution;
  map_normalized.info.resolution        = resolution;
  map_normalized.info.origin.position.x = map_origin_x;
  map_normalized.info.origin.position.y = map_origin_y;
  map_normalized.header.frame_id        = "uav46/local_origin";

  rows = (int)(height / resolution);
  cols = (int)(width / resolution);

  for (int j = 0; j < rows * cols; j++) {
    data_raw.push_back(0);
    time_spent_at_cell.push_back(0);
    map_normalized.data.push_back(0);
  }

  map_raw_publisher        = nh_.advertise<nav_msgs::OccupancyGrid>("map_raw_out", 10);
  map_normalized_publisher = nh_.advertise<nav_msgs::OccupancyGrid>("map_normalized_out", 10);

  /* // SUBSCRIBERS */
  data_subscriber = nh_.subscribe("data_in", 10, &dataCallback);
  odom_subscriber = nh_.subscribe("odom_in", 10, &odomCallback, ros::TransportHints().tcpNoDelay());

  ROS_INFO("Starting map builder fed by processed data topic %s", data_subscriber.getTopic().c_str());

  map_publish_timer = nh_.createTimer(ros::Duration(map_publish_interval), &mapPublishTimer);
  ros::spin();

  return 0;
}
