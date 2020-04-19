#include <radiation_utils/geometry.h>

/* quaternion utils //{ */
Eigen::Quaterniond geometry::quaternionFromEuler(double x, double y, double z) {
  return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
}

Eigen::Quaterniond geometry::quaternionFromEuler(Eigen::Vector3d euler) {

  return Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
}
//}

/* Ray //{ */

/* constructors //{ */
Ray::Ray() {
  point1 = Eigen::Vector3d::Zero();
  point2 = Eigen::Vector3d::Zero();
}

Ray::Ray(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  point1 = p1;
  point2 = p2;
}

Ray::~Ray() {
}
//}

/* getters //{ */
const Eigen::Vector3d Ray::p1() {
  return point1;
}

const Eigen::Vector3d Ray::p2() {
  return point2;
}

const Eigen::Vector3d Ray::direction() {
  return (point2 - point1);
}
//}

/* raycasting //{ */
Ray Ray::twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
  return Ray(pointFrom, pointTo);
}

Ray Ray::directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
  return Ray(origin, origin + direction);
}
//}

//}

/* Triangle //{ */

/* constructors //{ */
Triangle::Triangle() {
  point1 = Eigen::Vector3d(0, 0, 0);
  point2 = Eigen::Vector3d(1, 0, 0);
  point3 = Eigen::Vector3d(0, 0, 1);
}

Triangle::Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
  point1 = a;
  point2 = b;
  point3 = c;
}

Triangle::~Triangle() {
}
//}

/* getters //{ */
const Eigen::Vector3d Triangle::a() {
  return point1;
}

const Eigen::Vector3d Triangle::b() {
  return point2;
}

const Eigen::Vector3d Triangle::c() {
  return point3;
}

const Eigen::Vector3d Triangle::normal() {
  Eigen::Vector3d n;
  n = (point2 - point1).cross(point3 - point1);
  return n.normalized();
}

const Eigen::Vector3d Triangle::center() {
  return (point1 + point2 + point3) / 3.0;
}

const std::vector<Eigen::Vector3d> Triangle::vertices() {
  std::vector<Eigen::Vector3d> vertices;
  vertices.push_back(point1);
  vertices.push_back(point2);
  vertices.push_back(point3);
  return vertices;
}
//}

/* intersectionRay //{ */
boost::optional<Eigen::Vector3d> Triangle::intersectionRay(Ray r, double epsilon) {
  // The Möller–Trumbore algorithm
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  Eigen::Vector3d v1  = point2 - point1;
  Eigen::Vector3d v2  = point3 - point1;
  Eigen::Vector3d h   = r.direction().cross(v2);
  double          res = v1.dot(h);
  if (res > -epsilon && res < epsilon) {
    return boost::none;
  }
  double          f = 1.0 / res;
  Eigen::Vector3d s = r.p1() - point1;
  double          u = f * s.dot(h);
  if (u < 0.0 || u > 1.0) {
    return boost::none;
  }
  Eigen::Vector3d q = s.cross(v1);
  double          v = f * r.direction().dot(q);
  if (v < 0.0 || u + v > 1.0) {
    return boost::none;
  }
  double t = f * v2.dot(q);
  if (t > epsilon) {
    Eigen::Vector3d ret = r.p1() + r.direction() * t;
    return ret;
  }
  return boost::none;
}
//}

//}

/* Rectangle //{ */

/* constructors //{ */
Rectangle::Rectangle() {
  point1 = Eigen::Vector3d(0, 0, 0);
  point2 = Eigen::Vector3d(1, 0, 0);
  point3 = Eigen::Vector3d(1, 1, 0);
  point4 = Eigen::Vector3d(0, 1, 0);
}

Rectangle::Rectangle(std::vector<Eigen::Vector3d> points) {
  point1 = points[0];
  point2 = points[1];
  point3 = points[2];
  point4 = points[3];
}

Rectangle::Rectangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d) {
  point1 = a;
  point2 = b;
  point3 = c;
  point4 = d;
}

Rectangle::~Rectangle() {
}
//}

/* getters //{ */
const Eigen::Vector3d Rectangle::a() {
  return point1;
}

const Eigen::Vector3d Rectangle::b() {
  return point2;
}

const Eigen::Vector3d Rectangle::c() {
  return point3;
}

const Eigen::Vector3d Rectangle::d() {
  return point4;
}

const Eigen::Vector3d Rectangle::center() {
  return (point1 + point2 + point3 + point4) / 4.0;
}

const Eigen::Vector3d Rectangle::normal() {
  Eigen::Vector3d n;
  n = (point2 - point1).cross(point4 - point1);
  return n.normalized();
}

const std::vector<Eigen::Vector3d> Rectangle::vertices() {
  std::vector<Eigen::Vector3d> vertices;
  vertices.push_back(point1);
  vertices.push_back(point2);
  vertices.push_back(point3);
  vertices.push_back(point4);
  return vertices;
}

const std::vector<Triangle> Rectangle::triangles() {
  Triangle t1(point1, point2, point3);
  Triangle t2(point1, point3, point4);

  std::vector<Triangle> triangles;
  triangles.push_back(t1);
  triangles.push_back(t2);
  return triangles;
}
//}

//}

/* Cuboid //{ */

/* constructors //{ */
Cuboid::Cuboid() {
  for (int i = 0; i < 8; i++) {
    points.push_back(Eigen::Vector3d::Zero());
  }
}

Cuboid::Cuboid(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4, Eigen::Vector3d p5, Eigen::Vector3d p6,
               Eigen::Vector3d p7) {
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
}

Cuboid::Cuboid(std::vector<Eigen::Vector3d> points) {
  this->points = points;
}

Cuboid::Cuboid(Eigen::Vector3d center, Eigen::Vector3d size, Eigen::Quaterniond orientation) {
  Eigen::Vector3d p0(-size.x() / 2, -size.y() / 2, -size.z() / 2);
  Eigen::Vector3d p1(size.x() / 2, -size.y() / 2, -size.z() / 2);
  Eigen::Vector3d p2(size.x() / 2, size.y() / 2, -size.z() / 2);
  Eigen::Vector3d p3(-size.x() / 2, size.y() / 2, -size.z() / 2);
  Eigen::Vector3d p4(-size.x() / 2, -size.y() / 2, size.z() / 2);
  Eigen::Vector3d p5(size.x() / 2, -size.y() / 2, size.z() / 2);
  Eigen::Vector3d p6(size.x() / 2, size.y() / 2, size.z() / 2);
  Eigen::Vector3d p7(-size.x() / 2, size.y() / 2, size.z() / 2);

  p0 = center + orientation * p0;
  p1 = center + orientation * p1;
  p2 = center + orientation * p2;
  p3 = center + orientation * p3;
  p4 = center + orientation * p4;
  p5 = center + orientation * p5;
  p6 = center + orientation * p6;
  p7 = center + orientation * p7;

  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
  points.push_back(p5);
  points.push_back(p6);
  points.push_back(p7);
}

Cuboid::~Cuboid() {
}
//}

/* lookupPoints //{ */
std::vector<Eigen::Vector3d> Cuboid::lookupPoints(int face_idx) {
  std::vector<Eigen::Vector3d> lookup;
  switch (face_idx) {
    case Cuboid::BOTTOM:
      lookup.push_back(points[0]);
      lookup.push_back(points[1]);
      lookup.push_back(points[2]);
      lookup.push_back(points[3]);
      break;
    case Cuboid::TOP:
      lookup.push_back(points[4]);
      lookup.push_back(points[5]);
      lookup.push_back(points[6]);
      lookup.push_back(points[7]);
      break;
    case Cuboid::LEFT:
      lookup.push_back(points[3]);
      lookup.push_back(points[0]);
      lookup.push_back(points[4]);
      lookup.push_back(points[7]);
      break;
    case Cuboid::RIGHT:
      lookup.push_back(points[2]);
      lookup.push_back(points[6]);
      lookup.push_back(points[5]);
      lookup.push_back(points[1]);
      break;
    case Cuboid::FRONT:
      lookup.push_back(points[0]);
      lookup.push_back(points[1]);
      lookup.push_back(points[5]);
      lookup.push_back(points[4]);
      break;
    case Cuboid::BACK:
      lookup.push_back(points[2]);
      lookup.push_back(points[3]);
      lookup.push_back(points[7]);
      lookup.push_back(points[6]);
      break;
  }
  return lookup;
}
//}

/* getters //{ */
const std::vector<Eigen::Vector3d> Cuboid::vertices() {
  return points;
}

const Rectangle Cuboid::getRectangle(int face_idx) {
  return Rectangle(lookupPoints(face_idx));
}

const Eigen::Vector3d Cuboid::center() {
  Eigen::Vector3d point_sum = points[0];
  for (int i = 1; i < 8; i++) {
    point_sum += points[i];
  }
  return point_sum / 8.0;
}
//}

//}

/* Ellipse //{ */

/* constructors //{ */
Ellipse::Ellipse() {
}

Ellipse::~Ellipse() {
}

Ellipse::Ellipse(Eigen::Vector3d center, Eigen::Quaterniond orientation, double a, double b) {
  center_point         = center;
  absolute_orientation = orientation;
  major_semi           = a;
  minor_semi           = b;
}
//}

/* getters //{ */
double Ellipse::a() {
  return major_semi;
}

double Ellipse::b() {
  return minor_semi;
}

const Eigen::Vector3d Ellipse::center() {
  return center_point;
}

const Eigen::Quaterniond Ellipse::orientation() {
  return absolute_orientation;
}

//}

//}

/* Cylinder //{ */

/* constructors //{ */
Cylinder::Cylinder() {
}

Cylinder::~Cylinder() {
}

Cylinder::Cylinder(Eigen::Vector3d center, double radius, double height, Eigen::Quaterniond orientation) {
  this->center_point         = center;
  this->radius               = radius;
  this->height               = height;
  this->absolute_orientation = orientation;
}
//}

/* getters //{ */
const Eigen::Vector3d Cylinder::center() {
  return center_point;
}

const Eigen::Quaterniond Cylinder::orientation() {
  return absolute_orientation;
}

double Cylinder::r() {
  return radius;
}

double Cylinder::h() {
  return height;
}

const Ellipse Cylinder::getCap(int index) {
  Ellipse         e;
  Eigen::Vector3d ellipse_center;
  switch (index) {
    case Cylinder::BOTTOM:
      ellipse_center = center() - orientation() * (Eigen::Vector3d::UnitZ() * (h() / 2.0));
      e              = Ellipse(ellipse_center, orientation(), r(), r());
      break;
    case Cylinder::TOP:
      ellipse_center = center() + orientation() * (Eigen::Vector3d::UnitZ() * (h() / 2.0));
      e              = Ellipse(ellipse_center, orientation(), r(), r());
      break;
  }
  return e;
}

//}

//}

/* Cone //{ */

/* constructors //{ */
Cone::Cone() {
}

Cone::~Cone() {
}

Cone::Cone(Eigen::Vector3d origin_point, double angle, double height, Eigen::Vector3d absolute_direction) {
  this->origin_point       = origin_point;
  this->angle              = angle;
  this->height             = height;
  this->absolute_direction = absolute_direction.normalized();
}
//}

/* getters //{ */
const Eigen::Vector3d Cone::origin() {
  return origin_point;
}

const Eigen::Vector3d Cone::direction() {
  return absolute_direction;
}

const Eigen::Vector3d Cone::center() {
  return origin() + (0.5 * h()) * direction();
}

double Cone::theta() {
  return angle;
}

double Cone::h() {
  return height;
}

const Ellipse Cone::getCap() {
  Eigen::Vector3d    ellipse_center      = origin() + direction() * h();
  Eigen::Quaterniond ellipse_orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction());
  double             cap_radius          = std::tan(theta()) * h();
  Ellipse            e(ellipse_center, ellipse_orientation, cap_radius, cap_radius);
  return e;
}

//}

//}
