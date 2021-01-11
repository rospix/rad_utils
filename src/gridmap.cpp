#include <rad_utils/gridmap.h>
#include <algorithm>

namespace mapping
{

/* GridMap (width, height, resolution, origin, rotation, parent_frame) //{ */
GridMap::GridMap(double width, double height, double resolution, Eigen::Vector2d origin, double rotation, std::string parent_frame) {

  /* arguments sanity check //{ */
  if (width <= 0) {
    throw std::invalid_argument("tried to create grid with zero or negative width");
  }
  if (height <= 0) {
    throw std::invalid_argument("tried to create grid with zero or negative height");
  }
  if (resolution <= 0) {
    throw std::invalid_argument("tried to create grid with zero or negative resolution");
  }
  rows    = (int)(height / resolution);
  columns = (int)(width / resolution);

  if (rows < 1) {
    throw std::invalid_argument("tried to create grid with less than one row");
  }
  if (columns < 1) {
    throw std::invalid_argument("tried to create grid with less than one column");
  }
  //}


  size               = Eigen::Vector2d(width, height);
  this->origin       = origin;
  this->resolution   = resolution;
  this->parent_frame = parent_frame;
  this->rotation     = rotation;


  // initialize cells with zeros
  for (int i = 0; i < rows * columns; i++) {
    cells.push_back(0);
  }
}
//}

/* Destructor //{ */
GridMap::~GridMap() {
}
//}

/* getIndex (index) //{ */
size_t GridMap::getIndex(int index) {
  size_t idx = (size_t)index;
  if (idx >= 0 && idx < cells.size()) {
    return idx;
  }
  throw std::invalid_argument("array index out of bounds");
}
//}

/* getIndex (row, col) //{ */
size_t GridMap::getIndex(int row, int col) {
  size_t idx = col + row * columns;
  if (idx >= 0 && idx < cells.size()) {
    return idx;
  }
  throw std::invalid_argument("array index out of bounds");
}
//}

/* getIndex (coords) //{ */
size_t GridMap::getIndex(Eigen::Vector2i coords) {
  size_t idx = coords.y() + coords.x() * columns;
  if (idx >= 0 && idx < cells.size()) {
    return idx;
  }
  throw std::invalid_argument("array index out of bounds");
}
//}

/* getWidth //{ */
double GridMap::getWidth() {
  return size.x();
}
//}

/* getHeight //{ */
double GridMap::getHeight() {
  return size.y();
}
//}

/* getRows //{ */
int GridMap::getRows() {
  return rows;
}
//}

/* getColumns //{ */
int GridMap::getColumns() {
  return columns;
}
//}

/* getNumCells //{ */
size_t GridMap::getNumCells() {
  return cells.size();
}
//}

/* getValue (index) //{ */
double GridMap::getValue(int index) {
  return cells[getIndex(index)];
}
//}

/* getValue (row, col) //{ */
double GridMap::getValue(int row, int col) {
  return cells[getIndex(row, col)];
}
//}

/* getValue (coords) //{ */
double GridMap::getValue(Eigen::Vector2i coords) {
  return cells[getIndex(coords)];
}
//}

/* buildOccupancyGrid //{ */
nav_msgs::OccupancyGrid GridMap::buildOccupancyGrid() {
  nav_msgs::OccupancyGrid out;
  out.header.frame_id        = parent_frame;
  out.header.stamp           = ros::Time::now();
  out.info.width             = getWidth() / resolution;
  out.info.height            = getHeight() / resolution;
  out.info.origin.position.x = origin.x();
  out.info.origin.position.y = origin.y();
  out.info.origin.position.z = 0;
  out.info.resolution        = resolution;
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitZ());
  out.info.origin.orientation.w = q.w();
  out.info.origin.orientation.x = q.x();
  out.info.origin.orientation.y = q.y();
  out.info.origin.orientation.z = q.z();

  // normalize to int8 <0, 254> range
  double data_max = *std::max_element(cells.begin(), cells.end());
  double data_min = *std::min_element(cells.begin(), cells.end());
  data_max -= data_min;

  for (size_t i = 0; i < cells.size(); i++) {
    double cell_data;
    if (data_max != 0) {
      cell_data = ((cells[i] - data_min) / data_max) * 254.0;
    } else {
      cell_data = cells[i];
    }
    uint8_t cell_data_sample = (uint8_t)cell_data;
    out.data.push_back(cell_data_sample);
  }
  return out;
}
//}

/* setCellValue (index) //{ */
void GridMap::setCellValue(double value, int index) {
  cells[getIndex(index)] = value;
}
//}

/* setCellValue (row, col) //{ */
void GridMap::setCellValue(double value, int row, int col) {
  cells[getIndex(row, col)] = value;
}
//}

/* setCellValue (coords) //{ */
void GridMap::setCellValue(double value, Eigen::Vector2i coords) {
  cells[getIndex(coords)] = value;
}
//}

}  // namespace mapping
