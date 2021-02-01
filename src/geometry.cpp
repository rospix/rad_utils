#include <rad_utils/geometry.h>

namespace rad_utils
{

namespace geometry
{

/* rectSolidAngle() //{ */

double rectSolidAngle(mrs_lib::geometry::Rectangle r, Eigen::Vector3d center) {

  Eigen::Vector3d a = r.a() - center;
  Eigen::Vector3d b = r.b() - center;
  Eigen::Vector3d c = r.c() - center;
  Eigen::Vector3d d = r.d() - center;

  a.normalize();
  b.normalize();
  c.normalize();
  d.normalize();

  double t1 = mrs_lib::geometry::sphericalTriangleArea(a, b, c);
  double t2 = mrs_lib::geometry::sphericalTriangleArea(c, d, a);

  if (t1 != t1 || t1 < 1e-11) {
    t1 = 0.0;
  }
  if (t2 != t2 || t2 < 1e-11) {
    t2 = 0.0;
  }

  return t1 + t2;
}

//}

}  // namespace geometry

}  // namespace rad_utils
