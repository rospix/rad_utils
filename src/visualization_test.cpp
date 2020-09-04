#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/batch_visualizer.h>
#include <radiation_utils/marching_cubes.h>

#include <random>

double range_min = -20;
double range_max = 20;

std::mt19937                           generator(time(0));
std::uniform_real_distribution<double> rand_dbl(range_min, range_max);
std::uniform_real_distribution<double> rand_percent(0, 1);

int main(int argc, char** argv) {

  ros::init(argc, argv, "radiation_utils_test");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ROS_INFO("[%s]: Test started!", ros::this_node::getName().c_str());

  mrs_lib::BatchVisualizer bv;
  bv = mrs_lib::BatchVisualizer(nh, "test_markers", "map");

  bv.clearBuffers();
  bv.clearVisuals();
  bv.setPointsScale(0.4);

  /* RANDOM POINTS //{ */
  ROS_INFO("[%s]: Generating random points..", ros::this_node::getName().c_str());
  for (int i = 0; (i < 400 && ros::ok()); i++) {
    double          x = rand_dbl(generator);
    double          y = rand_dbl(generator);
    double          z = rand_dbl(generator);
    Eigen::Vector3d point(x, y, z);
    double          r = (x - range_min) / (range_max - range_min);
    double          g = (y - range_min) / (range_max - range_min);
    double          b = (z - range_min) / (range_max - range_min);
    bv.addPoint(point, r, g, b, 1);
    ros::spinOnce();
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RANDOM RAYS //{ */
  ROS_INFO("[%s]: Generating random rays...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    mrs_lib::Ray    ray = mrs_lib::Ray::twopointCast(point1, point2);
    double          r   = ((x1 * x2) - range_min) / (range_max - range_min);
    double          g   = ((y1 * y2) - range_min) / (range_max - range_min);
    double          b   = ((z1 * z2) - range_min) / (range_max - range_min);
    bv.addRay(ray, r, g, b, 1);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RAYS FROM ORIGIN //{ */
  ROS_INFO("[%s]: Generating rays originating from one point..", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          a = rand_dbl(generator);
    double          b = rand_dbl(generator);
    double          c = rand_dbl(generator);
    Eigen::Vector3d point(a, b, c);
    mrs_lib::Ray    ray = mrs_lib::Ray::twopointCast(Eigen::Vector3d::Zero(), point);
    double          r   = (i + 1) / 100.0;
    double          g   = 1 - r;
    bv.addRay(ray, r, g, 0, 1);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RANDOM FILLED TRIANGLES //{ */
  ROS_INFO("[%s]: Generating random filled triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double            x1 = rand_dbl(generator);
    double            y1 = rand_dbl(generator);
    double            z1 = rand_dbl(generator);
    Eigen::Vector3d   point1(x1, y1, z1);
    double            x2 = rand_dbl(generator);
    double            y2 = rand_dbl(generator);
    double            z2 = rand_dbl(generator);
    Eigen::Vector3d   point2(x2, y2, z2);
    double            x3 = rand_dbl(generator);
    double            y3 = rand_dbl(generator);
    double            z3 = rand_dbl(generator);
    Eigen::Vector3d   point3(x3, y3, z3);
    double            r = x1 * x2 * x3;
    double            g = y1 * y2 * y3;
    double            b = z1 * z2 * z3;
    mrs_lib::Triangle tri(point1, point2, point3);
    bv.addTriangle(tri, r, g, b, 1, true);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RANDOM OUTLINED TRIANGLES //{ */
  ROS_INFO("[%s]: Generating random outlined triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double            x1 = rand_dbl(generator);
    double            y1 = rand_dbl(generator);
    double            z1 = rand_dbl(generator);
    Eigen::Vector3d   point1(x1, y1, z1);
    double            x2 = rand_dbl(generator);
    double            y2 = rand_dbl(generator);
    double            z2 = rand_dbl(generator);
    Eigen::Vector3d   point2(x2, y2, z2);
    double            x3 = rand_dbl(generator);
    double            y3 = rand_dbl(generator);
    double            z3 = rand_dbl(generator);
    Eigen::Vector3d   point3(x3, y3, z3);
    double            r = x1 * x2 * x3;
    double            g = y1 * y2 * y3;
    double            b = z1 * z2 * z3;
    mrs_lib::Triangle tri(point1, point2, point3);
    bv.addTriangle(tri, r, g, b, 1, false);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RANDOM RECTANGLES //{ */
  ROS_INFO("[%s]: Generating random rectangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    double          plane_x = rand_dbl(generator);
    double          plane_y = rand_dbl(generator);
    double          plane_z = rand_dbl(generator);
    Eigen::Vector3d plane_anchor(plane_x, plane_y, plane_z);
    double          side_length2 = rand_dbl(generator) / 2.0;
    Eigen::Vector3d point3       = point2 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());
    Eigen::Vector3d point4       = point1 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());

    double             r = x1 * x2;
    double             g = y1 * y2;
    double             b = z1 * z2;
    mrs_lib::Rectangle rect(point1, point2, point3, point4);
    bv.addRectangle(rect, r, g, b, 1, true);   // draw colored faces
    bv.addRectangle(rect, 0, 0, 0, 1, false);  // draw outlines
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  bv.clearVisuals();
  //}

  /* RANDOM CUBES //{ */
  ROS_INFO("[%s]: Generating random cubes...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             s = 0.4 * rand_dbl(generator);
    Eigen::Vector3d    scale(s, s, s);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::quaternionFromEuler(x2, y2, z2);

    double          r = (x1 - range_min) / (range_max - range_min);
    double          g = (y1 - range_min) / (range_max - range_min);
    double          b = (z1 - range_min) / (range_max - range_min);
    mrs_lib::Cuboid cub(center, scale, orientation);

    bv.addCuboid(cub, r, g, b, 1, true);   // draw colored faces
    bv.addCuboid(cub, 0, 0, 0, 1, false);  // draw outlines
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  bv.clearVisuals();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* MARCHING CUBES //{ */

  int cube_size = 3;

  /* GENERATE SCALAR FIELD //{ */
  bv.setPointsScale(0.2);
  double                       threshold = 0.2;
  std::vector<Eigen::Vector3d> active_points;
  std::vector<Eigen::Vector3d> all_points;
  std::vector<double>          all_values;
  ROS_INFO("[%s]: Generating random scalar field...", ros::this_node::getName().c_str());
  for (int i = range_min; (i < range_max && ros::ok()); i += cube_size) {
    for (int j = range_min; (j < range_max && ros::ok()); j += cube_size) {
      for (int k = range_min; (k < range_max && ros::ok()); k += cube_size) {
        double          value = rand_percent(generator);
        Eigen::Vector3d point(i, j, k);
        if (value >= threshold) {
          active_points.push_back(point);
        }
        all_points.push_back(point);
        all_values.push_back(value);
        bv.addPoint(point, value, value, value, 1.0);
        bv.publish();
        ros::spinOnce();
        ros::Duration(0.0005).sleep();
      }
    }
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* TRIANGULATE MESH //{ */
  std::vector<mrs_lib::Triangle> triangles;
  ROS_INFO("[%s]: Marching cubes test...", ros::this_node::getName().c_str());
  for (int i = range_min; (i < range_max - cube_size && ros::ok()); i += cube_size) {
    for (int j = range_min; (j < range_max - cube_size && ros::ok()); j += cube_size) {
      for (int k = range_min; (k < range_max - cube_size && ros::ok()); k += cube_size) {
        Eigen::Vector3d    center(i + cube_size / 2.0, j + cube_size / 2.0, k + cube_size / 2.0);
        Eigen::Vector3d    size(cube_size, cube_size, cube_size);
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

        bv.clearBuffers();
        /* bv.clearVisuals(); */
        for (unsigned int i = 0; i < all_points.size(); i++) {
          bv.addPoint(all_points[i], all_values[i], all_values[i], all_values[i], 1.0);
        }
        mrs_lib::Cuboid c(center, size, orientation);
        bv.addCuboid(c, 0.9, 0.9, 0.8, 1.0, false);
        std::vector<int>             active_indices;
        std::vector<Eigen::Vector3d> curr_vertices = c.vertices();
        for (unsigned int m = 0; m < curr_vertices.size(); m++) {
          if (std::find(active_points.begin(), active_points.end(), curr_vertices[m]) != active_points.end()) {
            active_indices.push_back(m);
          }
        }
        MarchingCube                   mc(c, active_indices);
        std::vector<mrs_lib::Triangle> curr_triangles = mc.getTriangles();
        for (unsigned int i = 0; i < curr_triangles.size(); i++) {
          triangles.push_back(curr_triangles[i]);
        }
        for (unsigned int i = 0; i < triangles.size(); i++) {
          double r = (triangles[i].a().x() - range_min) / (range_max - range_min);
          double g = (triangles[i].a().y() - range_min) / (range_max - range_min);
          double b = (triangles[i].a().z() - range_min) / (range_max - range_min);
          bv.addTriangle(triangles[i], r, g, b, 1.0, true);
          bv.addTriangle(triangles[i], 0.0, 0.0, 0.0, 1.0, false);
        }
        bv.publish();
        ros::spinOnce();
        ros::Duration(0.002).sleep();
      }
    }
  }
  //}

  bv.clearBuffers();
  bv.clearVisuals();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());

  //}

  /* RANDOM ELLIPSES //{ */
  ROS_INFO("[%s]: Generating random ellipses...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             major       = 0.4 * rand_dbl(generator);
    double             minor       = 0.1 * rand_dbl(generator);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::quaternionFromEuler(x2, y2, z2);

    double           r = (x1 - range_min) / (range_max - range_min);
    double           g = (y1 - range_min) / (range_max - range_min);
    double           b = (z1 - range_min) / (range_max - range_min);
    mrs_lib::Ellipse el(center, orientation, major, minor);

    bv.addEllipse(el, r, g, b, 1.0, true);         // colored face
    bv.addEllipse(el, 0.0, 0.0, 0.0, 1.0, false);  // black outline
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  bv.clearVisuals();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM CYLINDERS//{ */
  ROS_INFO("[%s]: Generating random cylinders...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::quaternionFromEuler(x2, y2, z2);

    double radius = 0.3 * rand_dbl(generator);
    double height = rand_dbl(generator);

    double r = (x1 - range_min) / (range_max - range_min);
    double g = (y1 - range_min) / (range_max - range_min);
    double b = (z1 - range_min) / (range_max - range_min);

    mrs_lib::Cylinder cyl(center, radius, height, orientation);
    bv.addCylinder(cyl, r, g, b, 1.0, true, true, 12);
    bv.addCylinder(cyl, 0, 0, 0, 1, false, false, 12);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  bv.clearVisuals();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM CONES //{ */
  ROS_INFO("[%s]: Generating random cones...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d origin(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d direction(x2, y2, z2);

    double angle  = 0.02 * rand_dbl(generator);
    double height = rand_dbl(generator);

    double r = (x1 - range_min) / (range_max - range_min);
    double g = (y1 - range_min) / (range_max - range_min);
    double b = (z1 - range_min) / (range_max - range_min);

    mrs_lib::Cone cone(origin, angle, height, direction);
    bv.addCone(cone, r, g, b, 1, true, true, 12);
    bv.addCone(cone, 0, 0, 0, 1, false, false, 12);
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  /* bv.clearBuffers(); */
  /* bv.clearVisuals(); */
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RAYTRACING TEST //{ */
  ROS_INFO("[%s]: Generating random rays and triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 20 && ros::ok()); i++) {
    double            x1 = rand_dbl(generator);
    double            y1 = rand_dbl(generator);
    double            z1 = rand_dbl(generator);
    Eigen::Vector3d   point1(x1, y1, z1);
    double            x2 = rand_dbl(generator);
    double            y2 = rand_dbl(generator);
    double            z2 = rand_dbl(generator);
    Eigen::Vector3d   point2(x2, y2, z2);
    double            x3 = rand_dbl(generator);
    double            y3 = rand_dbl(generator);
    double            z3 = rand_dbl(generator);
    Eigen::Vector3d   point3(x3, y3, z3);
    mrs_lib::Triangle tri(point1, point2, point3);

    double          rx1 = rand_dbl(generator);
    double          ry1 = rand_dbl(generator);
    double          rz1 = rand_dbl(generator);
    Eigen::Vector3d ray_point1(rx1, ry1, rz1);
    double          rx2 = rand_dbl(generator);
    double          ry2 = rand_dbl(generator);
    double          rz2 = rand_dbl(generator);
    Eigen::Vector3d ray_point2(rx2, ry2, rz2);
    mrs_lib::Ray    ray = mrs_lib::Ray::twopointCast(ray_point1, ray_point2);
    bv.addRay(ray, 1.0, 0.5, 0.0, 1.0);

    if (tri.intersectionRay(ray) == boost::none) {
      bv.addTriangle(tri, 1.0, 0.0, 0.0, 1.0, true);
    } else {
      bv.addTriangle(tri, 0.0, 1.0, 0.3, 1.0, true);
      bv.addPoint(tri.intersectionRay(ray).get());
    }
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.6).sleep();
    bv.clearBuffers();
    bv.clearVisuals();
  }
  /* bv.clearBuffers(); */
  /* bv.clearVisuals(); */
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  ROS_INFO("[%s]: TEST DONE!", ros::this_node::getName().c_str());
  return 0;
}
