#pragma once

#ifndef RADIATION_GEOMETRY_H
#define RADIATION_GEOMETRY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/optional.hpp>

/* Ray //{ */
class Ray {

public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d p1, Eigen::Vector3d p2);

private:
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;

public:
  const Eigen::Vector3d p1();
  const Eigen::Vector3d p2();
  const Eigen::Vector3d direction();

public:
  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo);
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction);
};
//}

/* Triangle //{ */
class Triangle {
public:
  Triangle();
  ~Triangle();
  Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

private:
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  Eigen::Vector3d point3;

public:
  const Eigen::Vector3d a();
  const Eigen::Vector3d b();
  const Eigen::Vector3d c();

  const Eigen::Vector3d center();
  const Eigen::Vector3d normal();

  const std::vector<Eigen::Vector3d> vertices();

public:
  const boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);
};
//}

/* Rectangle //{ */
class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(std::vector<Eigen::Vector3d> points);
  Rectangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d);

private:
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
  Eigen::Vector3d point3;
  Eigen::Vector3d point4;

public:
  const Eigen::Vector3d a();
  const Eigen::Vector3d b();
  const Eigen::Vector3d c();
  const Eigen::Vector3d d();

  const Eigen::Vector3d center();
  const Eigen::Vector3d normal();

  const std::vector<Eigen::Vector3d> vertices();
  const std::vector<Triangle>        triangles();

  const boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);

  bool isFacing(Eigen::Vector3d point);
};
//}

/* Cuboid //{ */
class Cuboid {
public:
  Cuboid();
  ~Cuboid();
  Cuboid(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4, Eigen::Vector3d p5, Eigen::Vector3d p6,
         Eigen::Vector3d p7);
  Cuboid(std::vector<Eigen::Vector3d> points);
  Cuboid(Eigen::Vector3d center, Eigen::Vector3d size, Eigen::Quaterniond orientation);

private:
  enum
  {
    BOTTOM = 0,
    TOP    = 1,
    LEFT   = 2,
    RIGHT  = 3,
    FRONT  = 4,
    BACK   = 5,
  };

private:
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> lookupPoints(int face_idx);

public:
  const std::vector<Eigen::Vector3d> vertices();
  const Eigen::Vector3d              center();
  const Rectangle                    getRectangle(int face_idx);

  const std::vector<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);
};
//}

/* Ellipse //{ */
class Ellipse {
public:
  Ellipse();
  ~Ellipse();
  Ellipse(Eigen::Vector3d center, Eigen::Quaterniond orientation, double a, double b);

private:
  double             major_semi;
  double             minor_semi;
  Eigen::Vector3d    center_point;
  Eigen::Quaterniond absolute_orientation;

public:
  double                   a();
  double                   b();
  const Eigen::Vector3d    center();
  const Eigen::Quaterniond orientation();
};
//}

/* Cylinder //{ */
class Cylinder {
public:
  Cylinder();
  ~Cylinder();
  Cylinder(Eigen::Vector3d center, double radius, double height, Eigen::Quaterniond orientation);

private:
  Eigen::Vector3d    center_point;
  double             radius;
  double             height;
  Eigen::Quaterniond absolute_orientation;

public:
  enum
  {
    BOTTOM = 0,
    TOP    = 1,
  };

public:
  const Eigen::Vector3d    center();
  const Eigen::Quaterniond orientation();
  double                   r();
  double                   h();
  const Ellipse            getCap(int index);
};
//}

/* Cone //{ */
class Cone {
public:
  Cone();
  ~Cone();
  Cone(Eigen::Vector3d origin_point, double angle, double height, Eigen::Vector3d orientation);

private:
  Eigen::Vector3d origin_point;
  double          angle;
  double          height;
  Eigen::Vector3d absolute_direction;

public:
  const Eigen::Vector3d origin();
  const Eigen::Vector3d direction();
  const Eigen::Vector3d center();
  double                theta();
  double                h();
  const Ellipse         getCap();
};
//}

/* class geometry (static utility functions) //{ */
class geometry {
public:
  /* Quaternion conversions //{ */
  static Eigen::Quaterniond quaternionFromEuler(double x, double y, double z);
  static Eigen::Quaterniond quaternionFromEuler(Eigen::Vector3d euler);
  //}

  /* angle and solid angle tools //{ */
  static double haversin(double angle);
  static double invHaversin(double angle);
  static double triangleArea(double a, double b, double c);
  static double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
  static double solidAngle(double a, double b, double c);
  static double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);
  static double rectSolidAngle(Rectangle r, Eigen::Vector3d center);
  //}
};

//}

#endif
