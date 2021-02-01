#ifndef RAD_UTILS_GEOMETRY_H
#define RAD_UTILS_GEOMETRY_H

#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/shapes.h>

namespace rad_utils
{

namespace geometry
{

double rectSolidAngle(mrs_lib::geometry::Rectangle r, Eigen::Vector3d center);

}  // namespace geometry

#endif
}
