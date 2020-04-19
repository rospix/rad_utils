#ifndef BATCH_VISUALIZER_H
#define BATCH_VISUALIZER_H

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <radiation_utils/geometry.h>
#include <dynamic_reconfigure/server.h>
#include <radiation_utils/batch_visualizerConfig.h>

#define DEFAULT_ELLIPSE_POINTS 64

class BatchVisualizer {

public:
  BatchVisualizer();
  ~BatchVisualizer();
  BatchVisualizer(ros::NodeHandle &nh, std::string marker_topic_name, std::string parent_frame);

  void addPoint(Eigen::Vector3d point, double r = 0.0, double g = 1.0, double b = 0.3, double a = 1.0);
  void addRay(Ray ray, double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0);
  void addTriangle(Triangle tri, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);
  void addRectangle(Rectangle rect, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);
  void addCuboid(Cuboid cuboid, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);
  void addEllipse(Ellipse ellipse, double r = 0.0, double g = 1.0, double b = 1.0, double a = 1.0, bool filled = true, int num_points = DEFAULT_ELLIPSE_POINTS);
  void addCylinder(Cylinder cylinder, double r = 0.7, double g = 0.8, double b = 0.3, double a = 1.0, bool filled = true, bool capped = true,
                   int sides = DEFAULT_ELLIPSE_POINTS);
  void addCone(Cone cone, double r = 0.7, double g = 0.8, double b = 0.3, double a = 1.0, bool filled = true, bool capped = true,
               int sides = DEFAULT_ELLIPSE_POINTS);

  void setPointsScale(double scale);
  void setLinesScale(double scale);

  void clearBuffers();
  void clearVisuals();
  void publish();

private:
  ros::Publisher                  visual_pub;
  visualization_msgs::MarkerArray msg;

  std::string parent_frame;
  std::string marker_topic_name;

  visualization_msgs::Marker points_marker;
  visualization_msgs::Marker lines_marker;
  visualization_msgs::Marker triangles_marker;

  bool initialized = false;
  void initialize(ros::NodeHandle &nh);

  double points_scale = 0.02;
  double lines_scale  = 0.04;

  void addNullPoint();
  void addNullLine();
  void addNullTriangle();

  std::vector<Eigen::Vector3d> buildEllipse(Ellipse ellispe, int num_points = DEFAULT_ELLIPSE_POINTS);

  // dynamic reconfigure
  typedef radiation_utils::batch_visualizerConfig Config;

  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>        reconfigure_server_;

  void dynamicReconfigureCallback(Config &config, uint32_t level);
};

#endif
